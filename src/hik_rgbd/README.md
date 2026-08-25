# hik_rgbd

`hik_rgbd` 是海康机器人 MV-EB435i RGB-D USB 相机的 ROS2 Jazzy 驱动包。当前功能包括
RGB 图像、深度图、左右红外图、点云、结构光开关、IMU 原始数据读取和 Madgwick 姿态滤波。

## 1. 系统要求

- Ubuntu 24.04
- ROS2 Jazzy
- 海康机器人 MV3D RGBD SDK
- USB3.0 接口和数据线

安装常用依赖：

```bash
sudo apt update
sudo apt install -y \
  git build-essential cmake python3-colcon-common-extensions libopencv-dev \
  ros-jazzy-rclcpp ros-jazzy-sensor-msgs ros-jazzy-std-msgs \
  ros-jazzy-rviz2 ros-jazzy-imu-filter-madgwick \
  ros-jazzy-rviz-imu-plugin ros-jazzy-rqt-image-view
```

如果系统还没有安装 ROS2 Jazzy，请先按 ROS2 官方文档完成 Jazzy 安装，再执行上面的依赖安装。

## 2. 拉取代码

如果你的 Git 仓库根目录就是 `hik_rgbd` 这个包：

```bash
mkdir -p ~/ros2_hik_ws/src
cd ~/ros2_hik_ws/src
git clone <你的仓库地址> hik_rgbd
```

如果你的 Git 仓库根目录是完整工作空间，里面已经包含 `src/hik_rgbd`：

```bash
git clone <你的仓库地址> ~/ros2_hik_ws
```

后续命令默认工作空间路径为：

```bash
cd ~/ros2_hik_ws
```

## 3. 配置海康 SDK

安装或解压海康 MV3D RGBD SDK。推荐路径：

```bash
/home/guo/hikrobot_sdk
```

如果放在其他路径，设置环境变量：

```bash
export HIKROBOT_MV3D_SDK=/你的/SDK/路径
```

例如：

```bash
export HIKROBOT_MV3D_SDK=/home/guo/hikrobot_sdk
```

建议写入 `~/.bashrc`：

```bash
echo 'export HIKROBOT_MV3D_SDK=/home/guo/hikrobot_sdk' >> ~/.bashrc
```

确认 SDK 文件存在：

```bash
ls $HIKROBOT_MV3D_SDK/Include/Mv3dRgbdApi.h
ls $HIKROBOT_MV3D_SDK/Lib/linux64/libMv3dRgbd.so
```

## 4. 设置 USBFS 缓冲区

EB435i 启动采集时需要较大的 USBFS 缓冲区。默认值过小时，常见错误是：

```text
MV3D_RGBD_Start failed: 0x80060005
```

临时设置，重启后失效：

```bash
echo 256 | sudo tee /sys/module/usbcore/parameters/usbfs_memory_mb
cat /sys/module/usbcore/parameters/usbfs_memory_mb
```

如果仓库中带有工作空间脚本，可以持久化设置：

```bash
cd ~/ros2_hik_ws
sudo scripts/install_usbfs_memory_service.sh
```

如果没有这个脚本，也可以手动创建 systemd 服务：

```bash
sudo tee /etc/systemd/system/hikrobot-usbfs-memory.service >/dev/null <<'EOF'
[Unit]
Description=Set USBFS memory for Hikrobot RGB-D camera
After=systemd-modules-load.service
ConditionPathExists=/sys/module/usbcore/parameters/usbfs_memory_mb

[Service]
Type=oneshot
ExecStart=/bin/sh -c 'echo 256 > /sys/module/usbcore/parameters/usbfs_memory_mb'
RemainAfterExit=yes

[Install]
WantedBy=multi-user.target
EOF

sudo systemctl daemon-reload
sudo systemctl enable --now hikrobot-usbfs-memory.service
cat /sys/module/usbcore/parameters/usbfs_memory_mb
```

## 5. 编译

```bash
cd ~/ros2_hik_ws
source /opt/ros/jazzy/setup.bash
export HIKROBOT_MV3D_SDK=${HIKROBOT_MV3D_SDK:-/home/guo/hikrobot_sdk}
colcon build --symlink-install --packages-select hik_rgbd
source install/setup.bash
```

确认包已经安装：

```bash
ros2 pkg executables hik_rgbd
```

正常应看到：

```text
hik_rgbd camera_node
hik_rgbd image_pipeline_all_in_one
```

## 6. 运行

默认启动 RGB、深度、红外、点云、IMU、Madgwick 姿态滤波和 RViz2：

```bash
cd ~/ros2_hik_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch hik_rgbd hik_rgbd_launch.py
```

不启动 RViz2：

```bash
ros2 launch hik_rgbd hik_rgbd_launch.py rviz:=false
```

启动时关闭结构光：

```bash
ros2 launch hik_rgbd hik_rgbd_launch.py laser_enable:=false
```

关闭 IMU：

```bash
ros2 launch hik_rgbd hik_rgbd_launch.py publish_imu:=false imu_filter:=false
```

满分辨率和较高帧率：

```bash
ros2 launch hik_rgbd hik_rgbd_launch.py resolution:=1280x720 frame_rate:=30.0
```

## 7. 话题

常用公开话题：

- `/rgb/image_raw`：RGB 图像，`sensor_msgs/msg/Image`，编码 `bgr8`
- `/depth/image_raw`：伪彩色深度图，`sensor_msgs/msg/Image`，编码 `bgr8`
- `/depth/image_raw_c16`：SDK 原始 C16 深度图，`sensor_msgs/msg/Image`，编码 `16SC1`
- `/lir/image_raw`：左红外图，`sensor_msgs/msg/Image`，编码 `mono8`
- `/rir/image_raw`：右红外图，`sensor_msgs/msg/Image`，编码 `mono8`
- `/points`：XYZ 点云，`sensor_msgs/msg/PointCloud2`
- `/imu/data`：Madgwick 滤波后的 IMU，`sensor_msgs/msg/Imu`，包含姿态四元数

内部隐藏话题：

- `/_hik_rgbd/imu/data_raw`：相机节点发给 Madgwick 滤波器的原始 IMU，只有加速度和角速度

普通使用时只看 `/imu/data`。如需调试隐藏话题：

```bash
ros2 topic list --include-hidden-topics
ros2 topic echo /_hik_rgbd/imu/data_raw
```

## 8. 查看图像、点云和 IMU

图像：

```bash
ros2 run rqt_image_view rqt_image_view
```

在 `rqt_image_view` 中选择：

```text
/rgb/image_raw
/depth/image_raw
/lir/image_raw
/rir/image_raw
```

点云使用 RViz2：

```text
Fixed Frame: camera_frame
Add -> By topic -> /points -> PointCloud2
```

IMU 使用 RViz2 的 IMU 插件：

```text
Fixed Frame: camera_frame
Add -> By topic -> /imu/data -> Imu
```

IMU 显示只显示姿态方向，不显示相机平移位置。平移相机时坐标轴位置不变是正常的；绕
x/y/z 轴旋转相机时，滤波后的 `/imu/data` 坐标轴才应该转动。没有磁力计输入时，yaw
会漂移，pitch 和 roll 更可信。

如果想让滤波器发布 `odom -> camera_frame` 的姿态 TF：

```bash
ros2 launch hik_rgbd hik_rgbd_launch.py imu_filter_publish_tf:=true
```

这时 RViz 的 `Fixed Frame` 需要切到 `odom`。注意这个 TF 只有姿态旋转，没有可靠平移；
单靠 IMU 无法稳定估计相机在空间中的移动距离。

## 9. Launch 参数

- `device_index`：相机序号，默认 `0`
- `frame_id`：图像和点云坐标系，默认 `camera_frame`
- `fetch_timeout_ms`：SDK 取帧超时时间，默认 `1000`
- `resolution`：分辨率，可选 `640x360` 或 `1280x720`，默认 `640x360`
- `frame_rate`：采集帧率，默认 `15.0`
- `laser_enable`：是否开启结构光投射器，默认 `true`
- `publish_pointcloud`：是否发布 `/points` 点云，默认 `true`
- `pointcloud_unit_scale`：点云单位缩放，默认 `0.001`
- `publish_imu`：是否启用相机 IMU，默认 `true`
- `raw_imu_topic`：内部原始 IMU 话题，默认 `/_hik_rgbd/imu/data_raw`
- `filtered_imu_topic`：滤波后 IMU 话题，默认 `/imu/data`
- `imu_frame_id`：IMU 消息坐标系，默认 `camera_frame`
- `imu_linear_acceleration_scale`：IMU 加速度缩放，默认 `1.0`
- `imu_angular_velocity_scale`：IMU 角速度缩放，默认 `0.017453292519943295`
- `imu_filter`：是否启动 `imu_filter_madgwick`，默认 `true`
- `imu_filter_gain`：Madgwick 滤波增益，默认 `0.1`
- `imu_filter_publish_tf`：滤波器是否发布 TF，默认 `false`
- `imu_filter_fixed_frame`：滤波器发布 TF 时使用的父坐标系，默认 `odom`
- `rviz`：是否随 launch 启动 RViz2，默认 `true`

查看所有 launch 参数：

```bash
ros2 launch hik_rgbd hik_rgbd_launch.py --show-args
```

## 10. 常见问题

`MV3D_RGBD_Start failed: 0x80060005`：
先执行 `echo 256 | sudo tee /sys/module/usbcore/parameters/usbfs_memory_mb`，再重新启动。
如果仍失败，降低分辨率或帧率。

`libMv3dRgbd.so` 找不到：
确认 `HIKROBOT_MV3D_SDK` 指向 SDK 根目录，且
`$HIKROBOT_MV3D_SDK/Lib/linux64/libMv3dRgbd.so` 存在。

RViz 里 IMU 不随相机平移：
这是正常现象。IMU 话题没有平移位置，只能显示姿态旋转。

RViz 的 `Add -> By topic` 没有 `/imu/data -> Imu`：
安装插件并重启 RViz2：

```bash
sudo apt install -y ros-jazzy-rviz-imu-plugin
```

## 11. 关键文件

- `include/hik_rgbd/camera_node.hpp`：相机节点核心代码，负责打开相机、设置参数、控制结构光、取帧、发布图像、点云和 IMU。
- `include/hik_rgbd/imu_sdk_compat.hpp`：IMU SDK 兼容层，声明公开头文件里缺失但动态库已导出的 IMU 回调接口。
- `launch/hik_rgbd_launch.py`：ROS2 launch 入口，启动相机节点、Madgwick 滤波节点和 RViz2。
- `config/hik_camera.rviz`：RViz2 配置文件，用于显示 RGB、深度、红外和 `/points`。
- `CMakeLists.txt`：构建规则，读取 `HIKROBOT_MV3D_SDK` 并链接 `libMv3dRgbd.so`。
- `package.xml`：ROS2 包信息和依赖声明。
