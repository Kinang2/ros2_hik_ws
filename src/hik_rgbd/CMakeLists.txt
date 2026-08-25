cmake_minimum_required(VERSION 3.16)
project(hik_rgbd)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV REQUIRED)

set(HIKROBOT_MV3D_SDK "$ENV{HIKROBOT_MV3D_SDK}" CACHE PATH "Path to Hikrobot MV3D RGBD SDK")
if(NOT HIKROBOT_MV3D_SDK)
  set(HIKROBOT_MV3D_SDK "/home/guo/hikrobot_sdk")
endif()

set(HIKROBOT_MV3D_INCLUDE_DIR "${HIKROBOT_MV3D_SDK}/Include")
set(HIKROBOT_MV3D_LIB_DIR "${HIKROBOT_MV3D_SDK}/Lib/linux64")

if(NOT EXISTS "${HIKROBOT_MV3D_INCLUDE_DIR}/Mv3dRgbdApi.h")
  message(FATAL_ERROR "Hikrobot SDK headers not found in ${HIKROBOT_MV3D_INCLUDE_DIR}")
endif()

find_library(HIKROBOT_MV3D_LIBRARY
  NAMES Mv3dRgbd
  PATHS "${HIKROBOT_MV3D_LIB_DIR}"
  NO_DEFAULT_PATH
  REQUIRED)

set(CMAKE_BUILD_RPATH "${HIKROBOT_MV3D_LIB_DIR}")
set(CMAKE_INSTALL_RPATH "${HIKROBOT_MV3D_LIB_DIR}")

ament_export_include_directories(include)

add_executable(image_pipeline_all_in_one
  src/image_pipeline_all_in_one.cpp)
ament_target_dependencies(image_pipeline_all_in_one
  "rclcpp"
  "sensor_msgs"
  "OpenCV")

add_executable(camera_node
  src/camera_node.cpp)
ament_target_dependencies(camera_node
  "rclcpp"
  "sensor_msgs"
  "OpenCV")

foreach(target image_pipeline_all_in_one camera_node)
  target_include_directories(${target} PRIVATE
    include/hik_rgbd
    "${HIKROBOT_MV3D_INCLUDE_DIR}"
    ${OpenCV_INCLUDE_DIRS})
  target_link_libraries(${target}
    ${HIKROBOT_MV3D_LIBRARY}
    ${OpenCV_LIBRARIES})
endforeach()


install(TARGETS
  image_pipeline_all_in_one
  camera_node
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
