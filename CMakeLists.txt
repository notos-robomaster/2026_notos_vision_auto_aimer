cmake_minimum_required(VERSION 3.16.3)

project(sp_vision)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Release)
# 针对当前平台的优化标志
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG -march=native -flto")
set(CMAKE_C_FLAGS_RELEASE "-O3 -DNDEBUG -march=native -flto")
set(CMAKE_EXE_LINKER_FLAGS_RELEASE "-flto")
message(STATUS "--------------------CMAKE_BUILD_TYPE: ${CMAKE_BUILD_TYPE}--------------------")

find_package(OpenCV REQUIRED)
find_package(fmt REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(spdlog REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(nlohmann_json REQUIRED)
set(OpenVINO_DIR "/opt/intel/openvino_2024.6.0/runtime/cmake/")
find_package(OpenVINO REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${PROJECT_SOURCE_DIR})

add_subdirectory(tools)
add_subdirectory(io)
add_subdirectory(tasks/auto_aim)

################## src ##################
add_executable(auto_aim src/auto_aim.cpp)

target_link_libraries(auto_aim ${OpenCV_LIBS} fmt::fmt yaml-cpp tools io task_auto_aim)

# 为 auto_aim 检查 ROS2 环境
find_package(ament_cmake QUIET)
find_package(rclcpp QUIET)
find_package(std_msgs QUIET)

# 如果找到 ROS2 环境，为 auto_aim 添加 ROS2 支持
if(ament_cmake_FOUND AND rclcpp_FOUND AND std_msgs_FOUND)
    target_compile_definitions(auto_aim PRIVATE HAS_ROS2)
    ament_target_dependencies(auto_aim rclcpp std_msgs)
    message(STATUS "ROS2 support enabled for auto_aim")
else()
    message(STATUS "ROS2 not found, auto_aim will compile without ROS2 support")
endif()
