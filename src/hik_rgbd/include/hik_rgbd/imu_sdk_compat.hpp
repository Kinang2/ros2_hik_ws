#ifndef HIK_RGBD_IMU_SDK_COMPAT_HPP
#define HIK_RGBD_IMU_SDK_COMPAT_HPP

#include "Mv3dRgbdApi.h"
#include "Mv3dRgbdDefine.h"

#include <cstdint>

namespace hik_rgbd
{

constexpr const char* kEventSelector = "EventSelector";
constexpr const char* kEventNotification = "EventNotification";
constexpr uint32_t kEventSelectorImuData = 0x9106;
constexpr uint32_t kEventNotificationOff = 0;
constexpr uint32_t kEventNotificationOn = 1;

// The SDK 1.2.0 Linux headers do not declare this event payload, but the
// exported library symbol and camera XML expose a 160-byte IMU event block.
struct Mv3dRgbdImuData
{
    uint32_t nVersion;
    float data[6];
    uint8_t reserved[132];
};

static_assert(sizeof(Mv3dRgbdImuData) == 160, "Unexpected Hikrobot IMU payload size");

using Mv3dRgbdImuDataCallback = void (*)(Mv3dRgbdImuData*, void*);

inline MV3D_RGBD_STATUS setImuEnumParam(HANDLE handle, const char* key, uint32_t value)
{
    MV3D_RGBD_PARAM param = {};
    param.enParamType = ParamType_Enum;
    param.ParamInfo.stEnumParam.nCurValue = value;
    return MV3D_RGBD_SetParam(handle, key, &param);
}

inline MV3D_RGBD_STATUS setImuEventNotification(HANDLE handle, bool enabled)
{
    const MV3D_RGBD_STATUS selector_ret = setImuEnumParam(handle, kEventSelector, kEventSelectorImuData);
    if (selector_ret != MV3D_RGBD_OK) {
        return selector_ret;
    }

    if (enabled) {
        const MV3D_RGBD_STATUS off_ret = setImuEnumParam(handle, kEventNotification, kEventNotificationOff);
        if (off_ret != MV3D_RGBD_OK) {
            return off_ret;
        }
    }

    return setImuEnumParam(
        handle,
        kEventNotification,
        enabled ? kEventNotificationOn : kEventNotificationOff);
}

}  // namespace hik_rgbd

extern "C" MV3D_RGBD_STATUS MV3D_RGBD_RegisterIMUDataCallBack(
    HANDLE handle,
    hik_rgbd::Mv3dRgbdImuDataCallback cbOutput,
    void* pUser);

namespace hik_rgbd
{

inline MV3D_RGBD_STATUS unregisterImuDataCallback(HANDLE handle)
{
    return MV3D_RGBD_RegisterIMUDataCallBack(handle, nullptr, nullptr);
}

}  // namespace hik_rgbd

#endif
