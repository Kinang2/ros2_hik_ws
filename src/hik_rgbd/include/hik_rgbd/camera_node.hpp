#ifndef CAMERA_NODE_HPP
#define CAMERA_NODE_HPP

#include "common.hpp"
#include "imu_sdk_compat.hpp"
#include "mycommon.hpp"

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"

#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

class CameraNode : public rclcpp::Node
{
public:
    CameraNode(const std::string& node_name = "hik_rgbd", unsigned int device = 0)
    : Node(node_name),
      m_canceled(false),
      m_imu_callback_count(0),
      m_handle(nullptr),
      m_imu_event_enabled(false),
      m_pBGR24(nullptr),
      m_bgr24_size(0)
    {
        m_device_index = this->declare_parameter<int>("device_index", static_cast<int>(device));
        m_frame_id = this->declare_parameter<std::string>("frame_id", "camera_frame");
        m_fetch_timeout_ms = this->declare_parameter<int>("fetch_timeout_ms", 1000);
        m_resolution = this->declare_parameter<std::string>("resolution", "640x360");
        m_frame_rate = this->declare_parameter<double>("frame_rate", 15.0);
        m_publish_pointcloud = this->declare_parameter<bool>("publish_pointcloud", true);
        m_pointcloud_unit_scale = this->declare_parameter<double>("pointcloud_unit_scale", 0.001);
        m_laser_enable = this->declare_parameter<bool>("laser_enable", true);
        m_publish_imu = this->declare_parameter<bool>("publish_imu", true);
        m_imu_topic = this->declare_parameter<std::string>("imu_topic", "/_hik_rgbd/imu/data_raw");
        m_imu_frame_id = this->declare_parameter<std::string>("imu_frame_id", m_frame_id);
        m_imu_linear_acceleration_scale = this->declare_parameter<double>("imu_linear_acceleration_scale", 1.0);
        m_imu_angular_velocity_scale = this->declare_parameter<double>(
            "imu_angular_velocity_scale",
            0.017453292519943295);

        m_rgb_pub = this->create_publisher<sensor_msgs::msg::Image>("rgb/image_raw", rclcpp::SensorDataQoS());
        m_depth_raw_pub = this->create_publisher<sensor_msgs::msg::Image>("depth/image_raw_c16", rclcpp::SensorDataQoS());
        m_depth_pub = this->create_publisher<sensor_msgs::msg::Image>("depth/image_raw", rclcpp::SensorDataQoS());
        m_lir_pub = this->create_publisher<sensor_msgs::msg::Image>("lir/image_raw", rclcpp::SensorDataQoS());
        m_rir_pub = this->create_publisher<sensor_msgs::msg::Image>("rir/image_raw", rclcpp::SensorDataQoS());
        m_points_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("points", rclcpp::SensorDataQoS());
        if (m_publish_imu) {
            m_imu_pub = this->create_publisher<sensor_msgs::msg::Imu>(m_imu_topic, rclcpp::SensorDataQoS());
        }

        try {
            initCamera(static_cast<unsigned int>(m_device_index));
        } catch (...) {
            cleanupCamera();
            throw;
        }

        m_thread = std::thread(&CameraNode::loop, this);
        RCLCPP_INFO(this->get_logger(), "hik_rgbd started");
    }

    ~CameraNode() override
    {
        m_canceled.store(true);
        if (m_thread.joinable()) {
            m_thread.join();
        }

        cleanupCamera();

        if (m_pBGR24 != nullptr) {
            free(m_pBGR24);
            m_pBGR24 = nullptr;
            m_bgr24_size = 0;
        }
    }

private:
    void loop()
    {
        while (rclcpp::ok() && !m_canceled.load()) {
            const int ret = MV3D_RGBD_FetchFrame(
                m_handle,
                m_pstFrameData,
                static_cast<uint32_t>(m_fetch_timeout_ms));

            if (ret == MV3D_RGBD_OK) {
                publishFrame();
                continue;
            }

            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                5000,
                "MV3D_RGBD_FetchFrame failed: 0x%x",
                ret);
        }
    }

    void initCamera(unsigned int index)
    {
        checkOk("MV3D_RGBD_Initialize", MV3D_RGBD_Initialize());

        MV3D_RGBD_VERSION_INFO version = {};
        checkOk("MV3D_RGBD_GetSDKVersion", MV3D_RGBD_GetSDKVersion(&version));
        RCLCPP_INFO(
            this->get_logger(),
            "MV3D RGBD SDK version: %u.%u.%u",
            version.nMajor,
            version.nMinor,
            version.nRevision);

        uint32_t device_count = 0;
        checkOk("MV3D_RGBD_GetDeviceNumber", MV3D_RGBD_GetDeviceNumber(DeviceType_USB, &device_count));
        if (device_count == 0) {
            throw std::runtime_error("No Hikrobot RGB-D USB camera found");
        }
        if (index >= device_count) {
            throw std::runtime_error("device_index is out of range");
        }

        std::vector<MV3D_RGBD_DEVICE_INFO> devices(device_count);
        checkOk(
            "MV3D_RGBD_GetDeviceList",
            MV3D_RGBD_GetDeviceList(DeviceType_USB, devices.data(), device_count, &device_count));

        for (uint32_t i = 0; i < device_count; ++i) {
            RCLCPP_INFO(
                this->get_logger(),
                "Camera[%u]: serial=%s usb_protocol=%d model=%s",
                i,
                devices[i].chSerialNumber,
                devices[i].SpecialInfo.stUsbInfo.enUsbProtocol,
                devices[i].chModelName);
        }

        checkOk("MV3D_RGBD_OpenDevice", MV3D_RGBD_OpenDevice(&m_handle, &devices[index]));
        applyStartupCameraParams();
        setLaserEnable(m_laser_enable);
        if (m_publish_imu) {
            setupImuPublishing();
        }
        checkOk("MV3D_RGBD_Start", MV3D_RGBD_Start(m_handle));
    }

    void cleanupCamera()
    {
        if (m_handle != nullptr) {
            if (m_imu_event_enabled) {
                logSdkResult("MV3D_RGBD_RegisterIMUDataCallBack cancel", hik_rgbd::unregisterImuDataCallback(m_handle));
                logSdkResult("IMU EventNotification Off", hik_rgbd::setImuEventNotification(m_handle, false));
                m_imu_event_enabled = false;
            }
            logSdkResult("MV3D_RGBD_Stop", MV3D_RGBD_Stop(m_handle));
            logSdkResult("MV3D_RGBD_CloseDevice", MV3D_RGBD_CloseDevice(&m_handle));
        }
        logSdkResult("MV3D_RGBD_Release", MV3D_RGBD_Release());
    }

    void applyStartupCameraParams()
    {
        if (m_resolution == "640x360") {
            setEnumParam(MV3D_RGBD_ENUM_RESOLUTION, ResolutionType_640_360);
        } else if (m_resolution == "1280x720") {
            setEnumParam(MV3D_RGBD_ENUM_RESOLUTION, ResolutionType_1280_720);
        } else {
            RCLCPP_WARN(
                this->get_logger(),
                "Unsupported resolution '%s'; expected 640x360 or 1280x720",
                m_resolution.c_str());
        }

        if (m_frame_rate > 0.0) {
            setFloatParam(MV3D_RGBD_FLOAT_FRAMERATE, static_cast<float>(m_frame_rate));
        }
    }

    void setEnumParam(const char* key, uint32_t value)
    {
        MV3D_RGBD_PARAM param = {};
        param.enParamType = ParamType_Enum;
        param.ParamInfo.stEnumParam.nCurValue = value;
        const int ret = MV3D_RGBD_SetParam(m_handle, key, &param);
        if (ret == MV3D_RGBD_OK) {
            RCLCPP_INFO(this->get_logger(), "%s set to %u", key, value);
            return;
        }

        RCLCPP_WARN(this->get_logger(), "%s set to %u returned 0x%x; continuing", key, value, ret);
    }

    void setFloatParam(const char* key, float value)
    {
        MV3D_RGBD_PARAM param = {};
        param.enParamType = ParamType_Float;
        param.ParamInfo.stFloatParam.fCurValue = value;
        const int ret = MV3D_RGBD_SetParam(m_handle, key, &param);
        if (ret == MV3D_RGBD_OK) {
            RCLCPP_INFO(this->get_logger(), "%s set to %.3f", key, value);
            return;
        }

        RCLCPP_WARN(this->get_logger(), "%s set to %.3f returned 0x%x; continuing", key, value, ret);
    }

    void publishFrame()
    {
        int ir_index = 0;
        for (uint32_t i = 0; i < m_pstFrameData->nImageCount; ++i) {
            MV3D_RGBD_IMAGE_DATA& image = m_pstFrameData->stImageData[i];

            switch (image.enImageType) {
                case ImageType_YUV422:
                    publishRgb(image);
                    break;
                case ImageType_Depth:
                    publishDepth(image);
                    if (m_publish_pointcloud) {
                        publishPointCloud(image);
                    }
                    break;
                case ImageType_Mono8:
                    if (ir_index == 0) {
                        publishMono8(image, m_lir_pub);
                    } else if (ir_index == 1) {
                        publishMono8(image, m_rir_pub);
                    }
                    ++ir_index;
                    break;
                default:
                    break;
            }
        }
    }

    void setLaserEnable(bool enabled)
    {
        MV3D_RGBD_PARAM param = {};
        param.enParamType = ParamType_Bool;
        param.ParamInfo.bBoolParam = enabled ? TRUE : FALSE;

        const int ret = MV3D_RGBD_SetParam(m_handle, MV3D_RGBD_BOOL_LASERENABLE, &param);
        if (ret == MV3D_RGBD_OK) {
            RCLCPP_INFO(this->get_logger(), "LaserEnable set to %s", enabled ? "true" : "false");
            return;
        }

        RCLCPP_WARN(
            this->get_logger(),
            "Failed to set LaserEnable=%s: 0x%x",
            enabled ? "true" : "false",
            ret);
    }

    void setupImuPublishing()
    {
        int ret = hik_rgbd::setImuEventNotification(m_handle, true);
        if (ret != MV3D_RGBD_OK) {
            RCLCPP_WARN(this->get_logger(), "Failed to enable IMU EventNotification: 0x%x", ret);
            return;
        }
        m_imu_event_enabled = true;

        logSdkResult("MV3D_RGBD_RegisterIMUDataCallBack cancel", hik_rgbd::unregisterImuDataCallback(m_handle));
        ret = MV3D_RGBD_RegisterIMUDataCallBack(m_handle, &CameraNode::imuDataCallback, this);
        if (ret != MV3D_RGBD_OK) {
            RCLCPP_WARN(this->get_logger(), "MV3D_RGBD_RegisterIMUDataCallBack failed: 0x%x", ret);
            return;
        }

        RCLCPP_INFO(
            this->get_logger(),
            "IMU callback registered, publishing %s",
            m_imu_topic.c_str());
    }

    void publishRgb(const MV3D_RGBD_IMAGE_DATA& image_data)
    {
        if (!YUYVToBGR24_Native(image_data.pData, image_data.nWidth, image_data.nHeight)) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                5000,
                "Failed to convert YUV422 frame to BGR");
            return;
        }

        cv::Mat image(image_data.nHeight, image_data.nWidth, CV_8UC3, m_pBGR24);
        publishCvImage(image, "bgr8", m_rgb_pub);
    }

    void publishDepth(const MV3D_RGBD_IMAGE_DATA& image_data)
    {
        const uint32_t step = image_data.nWidth * sizeof(int16_t);
        const size_t expected_size = static_cast<size_t>(step) * image_data.nHeight;
        const size_t copy_size = std::min(expected_size, static_cast<size_t>(image_data.nDataLen));

        auto raw_msg = std::make_unique<sensor_msgs::msg::Image>();
        raw_msg->header.stamp = this->get_clock()->now();
        raw_msg->header.frame_id = m_frame_id;
        raw_msg->height = image_data.nHeight;
        raw_msg->width = image_data.nWidth;
        raw_msg->encoding = "16SC1";
        raw_msg->is_bigendian = false;
        raw_msg->step = step;
        raw_msg->data.resize(expected_size);
        std::memcpy(raw_msg->data.data(), image_data.pData, copy_size);
        m_depth_raw_pub->publish(std::move(raw_msg));

        cv::Mat depth_image(image_data.nHeight, image_data.nWidth, CV_16SC1, image_data.pData);
        cv::Mat temp;
        cv::convertScaleAbs(depth_image, temp, 0.05);
        cv::Mat color_image;
        cv::applyColorMap(temp, color_image, cv::COLORMAP_JET);
        publishCvImage(color_image, "bgr8", m_depth_pub);
    }

    void publishMono8(
        const MV3D_RGBD_IMAGE_DATA& image_data,
        const rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr& publisher)
    {
        cv::Mat image(image_data.nHeight, image_data.nWidth, CV_8UC1, image_data.pData);
        publishCvImage(image, "mono8", publisher);
    }

    void publishPointCloud(MV3D_RGBD_IMAGE_DATA& depth_image)
    {
        MV3D_RGBD_IMAGE_DATA point_cloud_image = {};
        const int ret = MV3D_RGBD_MapDepthToPointCloud(m_handle, &depth_image, &point_cloud_image);
        if (ret != MV3D_RGBD_OK) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                5000,
                "MV3D_RGBD_MapDepthToPointCloud failed: 0x%x",
                ret);
            return;
        }

        if (point_cloud_image.enImageType != ImageType_PointCloud || point_cloud_image.pData == nullptr) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                5000,
                "SDK returned an invalid point cloud frame");
            return;
        }

        const size_t point_count = point_cloud_image.nDataLen / (sizeof(float) * 3);
        if (point_count == 0) {
            return;
        }

        uint32_t width = point_cloud_image.nWidth;
        uint32_t height = point_cloud_image.nHeight;
        if (static_cast<size_t>(width) * height != point_count) {
            width = static_cast<uint32_t>(point_count);
            height = 1;
        }

        auto msg = std::make_unique<sensor_msgs::msg::PointCloud2>();
        msg->header.stamp = this->get_clock()->now();
        msg->header.frame_id = m_frame_id;
        msg->height = height;
        msg->width = width;
        msg->is_bigendian = false;
        msg->is_dense = false;
        msg->point_step = sizeof(float) * 3;
        msg->row_step = msg->point_step * msg->width;
        msg->fields.resize(3);
        setPointField(msg->fields[0], "x", 0);
        setPointField(msg->fields[1], "y", sizeof(float));
        setPointField(msg->fields[2], "z", sizeof(float) * 2);
        msg->data.resize(point_count * msg->point_step);

        const float* src = reinterpret_cast<const float*>(point_cloud_image.pData);
        const float scale = static_cast<float>(m_pointcloud_unit_scale);
        const float nan = std::numeric_limits<float>::quiet_NaN();

        for (size_t i = 0; i < point_count; ++i) {
            float x = src[i * 3 + 0] * scale;
            float y = src[i * 3 + 1] * scale;
            float z = src[i * 3 + 2] * scale;

            if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z) || z <= 0.0f) {
                x = nan;
                y = nan;
                z = nan;
            }

            const size_t offset = i * msg->point_step;
            std::memcpy(msg->data.data() + offset, &x, sizeof(float));
            std::memcpy(msg->data.data() + offset + sizeof(float), &y, sizeof(float));
            std::memcpy(msg->data.data() + offset + sizeof(float) * 2, &z, sizeof(float));
        }

        m_points_pub->publish(std::move(msg));
    }

    static void imuDataCallback(hik_rgbd::Mv3dRgbdImuData* data, void* user)
    {
        auto* self = static_cast<CameraNode*>(user);
        if (self == nullptr || data == nullptr) {
            return;
        }

        self->publishImu(*data);
    }

    void publishImu(const hik_rgbd::Mv3dRgbdImuData& data)
    {
        if (!m_publish_imu || !m_imu_pub || m_canceled.load()) {
            return;
        }

        auto msg = std::make_unique<sensor_msgs::msg::Imu>();
        msg->header.stamp = this->get_clock()->now();
        msg->header.frame_id = m_imu_frame_id;
        msg->orientation.w = 1.0;
        msg->orientation_covariance[0] = -1.0;

        msg->linear_acceleration.x = data.data[0] * m_imu_linear_acceleration_scale;
        msg->linear_acceleration.y = data.data[1] * m_imu_linear_acceleration_scale;
        msg->linear_acceleration.z = data.data[2] * m_imu_linear_acceleration_scale;
        msg->angular_velocity.x = data.data[3] * m_imu_angular_velocity_scale;
        msg->angular_velocity.y = data.data[4] * m_imu_angular_velocity_scale;
        msg->angular_velocity.z = data.data[5] * m_imu_angular_velocity_scale;

        const uint64_t old_count = m_imu_callback_count.fetch_add(1);
        if (old_count == 0) {
            RCLCPP_INFO(
                this->get_logger(),
                "First IMU sample: version=%u accel=(%.4f, %.4f, %.4f) angular=(%.4f, %.4f, %.4f)",
                data.nVersion,
                msg->linear_acceleration.x,
                msg->linear_acceleration.y,
                msg->linear_acceleration.z,
                msg->angular_velocity.x,
                msg->angular_velocity.y,
                msg->angular_velocity.z);
        }

        m_imu_pub->publish(std::move(msg));
    }

    void publishCvImage(
        const cv::Mat& image,
        const std::string& encoding,
        const rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr& publisher)
    {
        auto msg = std::make_unique<sensor_msgs::msg::Image>();
        msg->header.stamp = this->get_clock()->now();
        msg->header.frame_id = m_frame_id;
        msg->height = static_cast<uint32_t>(image.rows);
        msg->width = static_cast<uint32_t>(image.cols);
        msg->encoding = encoding;
        msg->is_bigendian = false;
        msg->step = static_cast<sensor_msgs::msg::Image::_step_type>(image.step);
        msg->data.assign(image.datastart, image.dataend);
        publisher->publish(std::move(msg));
    }

    static void setPointField(sensor_msgs::msg::PointField& field, const std::string& name, uint32_t offset)
    {
        field.name = name;
        field.offset = offset;
        field.datatype = sensor_msgs::msg::PointField::FLOAT32;
        field.count = 1;
    }

    void checkOk(const char* action, int ret)
    {
        if (ret == MV3D_RGBD_OK) {
            return;
        }

        throw std::runtime_error(std::string(action) + " failed: 0x" + toHex(static_cast<uint32_t>(ret)));
    }

    void logSdkResult(const char* action, int ret)
    {
        if (ret == MV3D_RGBD_OK) {
            return;
        }

        RCLCPP_WARN(this->get_logger(), "%s failed: 0x%x", action, ret);
    }

    static std::string toHex(uint32_t value)
    {
        char buffer[16] = {};
        std::snprintf(buffer, sizeof(buffer), "%08x", value);
        return std::string(buffer);
    }

    int YUYVToBGR24_Native(uint8_t* pYUV, uint32_t width, uint32_t height)
    {
        const size_t required_size = static_cast<size_t>(width) * height * 3;
        if (required_size == 0 || pYUV == nullptr) {
            return 0;
        }

        if (m_bgr24_size < required_size) {
            uint8_t* new_buffer = static_cast<uint8_t*>(realloc(m_pBGR24, required_size));
            if (new_buffer == nullptr) {
                return 0;
            }
            m_pBGR24 = new_buffer;
            m_bgr24_size = required_size;
        }

        for (uint32_t y = 0; y < height; ++y) {
            for (uint32_t x = 0; x < width; ++x) {
                const uint32_t y_index = 2 * ((y * width) + x);
                const uint32_t u_index = 4 * (((y * width) + x) >> 1) + 1;
                const uint32_t v_index = 4 * (((y * width) + x) >> 1) + 3;
                const int y_value = pYUV[y_index];
                const int u_value = pYUV[u_index] - 128;
                const int v_value = pYUV[v_index] - 128;

                const int b = clamp8(static_cast<int>(y_value + 1.732446 * u_value));
                const int g = clamp8(static_cast<int>(y_value - 0.698001 * u_value - 0.703125 * v_value));
                const int r = clamp8(static_cast<int>(y_value + 1.370705 * v_value));
                const uint32_t out_index = (y * width + x) * 3;

                m_pBGR24[out_index + 0] = static_cast<uint8_t>(b);
                m_pBGR24[out_index + 1] = static_cast<uint8_t>(g);
                m_pBGR24[out_index + 2] = static_cast<uint8_t>(r);
            }
        }

        return 1;
    }

    static int clamp8(int value)
    {
        if (value < 0) {
            return 0;
        }
        if (value > 255) {
            return 255;
        }
        return value;
    }

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr m_rgb_pub;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr m_depth_raw_pub;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr m_depth_pub;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr m_lir_pub;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr m_rir_pub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_points_pub;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr m_imu_pub;

    std::atomic<bool> m_canceled;
    std::atomic<uint64_t> m_imu_callback_count;
    std::thread m_thread;
    void* m_handle;
    bool m_imu_event_enabled;
    MV3D_RGBD_FRAME_DATA m_stFrameData = {};
    MV3D_RGBD_FRAME_DATA* m_pstFrameData = &m_stFrameData;
    uint8_t* m_pBGR24;
    size_t m_bgr24_size;

    int m_device_index;
    std::string m_frame_id;
    int m_fetch_timeout_ms;
    std::string m_resolution;
    double m_frame_rate;
    bool m_publish_pointcloud;
    double m_pointcloud_unit_scale;
    bool m_laser_enable;
    bool m_publish_imu;
    std::string m_imu_topic;
    std::string m_imu_frame_id;
    double m_imu_linear_acceleration_scale;
    double m_imu_angular_velocity_scale;
};

#endif
