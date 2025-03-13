cmake_minimum_required(VERSION 3.22)

project(ris_msgs)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_VERBOSE_MAKEFILE ON)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
    msg/Hello.msg

    srv/PingPong.srv
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()