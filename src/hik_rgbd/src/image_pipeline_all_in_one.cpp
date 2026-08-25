#include <rclcpp/rclcpp.hpp>

#include <memory>

#include "../include/hik_rgbd/camera_node.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor executor;

  std::shared_ptr<CameraNode> camera_node = nullptr;
  try {
    camera_node = std::make_shared<CameraNode>("hik_rgbd");
  } catch (const std::exception & e) {
    fprintf(stderr, "%s Exiting ..\n", e.what());
    return 1;
  }

  executor.add_node(camera_node);

  executor.spin();

  rclcpp::shutdown();

  return 0;
}
