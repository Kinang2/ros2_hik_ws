import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    sdk_root = os.environ.get("HIKROBOT_MV3D_SDK", "/home/guo/hikrobot_sdk")
    sdk_lib_dir = os.path.join(sdk_root, "Lib", "linux64")
    existing_ld_library_path = os.environ.get("LD_LIBRARY_PATH", "")
    ld_library_path = sdk_lib_dir
    if existing_ld_library_path:
        ld_library_path = f"{sdk_lib_dir}:{existing_ld_library_path}"

    rviz_config = os.path.join(
        get_package_share_directory("hik_rgbd"),
        "config",
        "hik_camera.rviz",
    )

    return LaunchDescription([
        DeclareLaunchArgument("device_index", default_value="0"),
        DeclareLaunchArgument("frame_id", default_value="camera_frame"),
        DeclareLaunchArgument("fetch_timeout_ms", default_value="1000"),
        DeclareLaunchArgument("resolution", default_value="640x360"),
        DeclareLaunchArgument("frame_rate", default_value="15.0"),
        DeclareLaunchArgument("laser_enable", default_value="true"),
        DeclareLaunchArgument("publish_pointcloud", default_value="true"),
        DeclareLaunchArgument("pointcloud_unit_scale", default_value="0.001"),
        DeclareLaunchArgument("publish_imu", default_value="true"),
        DeclareLaunchArgument("raw_imu_topic", default_value="/_hik_rgbd/imu/data_raw"),
        DeclareLaunchArgument("filtered_imu_topic", default_value="/imu/data"),
        DeclareLaunchArgument("imu_frame_id", default_value="camera_frame"),
        DeclareLaunchArgument("imu_linear_acceleration_scale", default_value="1.0"),
        DeclareLaunchArgument("imu_angular_velocity_scale", default_value="0.017453292519943295"),
        DeclareLaunchArgument("imu_filter", default_value="true"),
        DeclareLaunchArgument("imu_filter_gain", default_value="0.1"),
        DeclareLaunchArgument("imu_filter_publish_tf", default_value="false"),
        DeclareLaunchArgument("imu_filter_fixed_frame", default_value="odom"),
        DeclareLaunchArgument("rviz", default_value="true"),
        SetEnvironmentVariable(name="LD_LIBRARY_PATH", value=ld_library_path),
        Node(
            package="hik_rgbd",
            executable="image_pipeline_all_in_one",
            name="hik_rgbd",
            output="screen",
            parameters=[{
                "device_index": ParameterValue(LaunchConfiguration("device_index"), value_type=int),
                "frame_id": LaunchConfiguration("frame_id"),
                "fetch_timeout_ms": ParameterValue(LaunchConfiguration("fetch_timeout_ms"), value_type=int),
                "resolution": LaunchConfiguration("resolution"),
                "frame_rate": ParameterValue(LaunchConfiguration("frame_rate"), value_type=float),
                "laser_enable": ParameterValue(LaunchConfiguration("laser_enable"), value_type=bool),
                "publish_pointcloud": ParameterValue(LaunchConfiguration("publish_pointcloud"), value_type=bool),
                "pointcloud_unit_scale": ParameterValue(LaunchConfiguration("pointcloud_unit_scale"), value_type=float),
                "publish_imu": ParameterValue(LaunchConfiguration("publish_imu"), value_type=bool),
                "imu_topic": LaunchConfiguration("raw_imu_topic"),
                "imu_frame_id": LaunchConfiguration("imu_frame_id"),
                "imu_linear_acceleration_scale": ParameterValue(
                    LaunchConfiguration("imu_linear_acceleration_scale"),
                    value_type=float,
                ),
                "imu_angular_velocity_scale": ParameterValue(
                    LaunchConfiguration("imu_angular_velocity_scale"),
                    value_type=float,
                ),
            }],
        ),
        Node(
            package="imu_filter_madgwick",
            executable="imu_filter_madgwick_node",
            name="imu_filter",
            output="screen",
            condition=IfCondition(LaunchConfiguration("imu_filter")),
            parameters=[{
                "stateless": False,
                "use_mag": False,
                "publish_tf": ParameterValue(LaunchConfiguration("imu_filter_publish_tf"), value_type=bool),
                "reverse_tf": False,
                "fixed_frame": LaunchConfiguration("imu_filter_fixed_frame"),
                "constant_dt": 0.0,
                "publish_debug_topics": False,
                "world_frame": "enu",
                "gain": ParameterValue(LaunchConfiguration("imu_filter_gain"), value_type=float),
                "zeta": 0.0,
                "orientation_stddev": 0.0,
            }],
            remappings=[
                ("imu/data_raw", LaunchConfiguration("raw_imu_topic")),
                ("imu/data", LaunchConfiguration("filtered_imu_topic")),
            ],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", rviz_config],
            output="screen",
            condition=IfCondition(LaunchConfiguration("rviz")),
        ),
    ])
