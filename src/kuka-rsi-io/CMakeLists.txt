cmake_minimum_required(VERSION 3.28)
project(kuka_rsi_io)

set(CMAKE_CXX_STANDARD 20)

find_package(asio CONFIG REQUIRED)
find_package(spdlog CONFIG REQUIRED)
find_package(Eigen3 CONFIG REQUIRED)
find_package(pugixml CONFIG REQUIRED)

add_library(kuka_rsi_io
    src/spinlock.cpp
    src/udpserver.cpp
    src/kukarsiinterface.cpp
    src/kukarsixmlserializer.cpp

    include/skrede/rsi/spinlock.h
    include/skrede/rsi/udpserver.h
    include/skrede/rsi/kukarsiinterface.h
    include/skrede/rsi/kukarsixmlserializer.h
)

target_link_libraries(kuka_rsi_io PUBLIC spdlog::spdlog_header_only Eigen3::Eigen asio::asio pugixml::pugixml)
target_include_directories(kuka_rsi_io PUBLIC include/ PRIVATE src/)

add_executable(demo demo/main.cpp)
target_link_libraries(demo kuka_rsi_io)