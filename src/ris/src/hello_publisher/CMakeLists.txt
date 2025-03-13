add_executable(hello_publisher
    main.cpp
)

ament_target_dependencies(hello_publisher rclcpp std_srvs std_msgs ris_msgs)

target_include_directories(hello_publisher PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(hello_publisher
    atomic
    fmt::fmt-header-only
)

ament_export_targets(hello_publisher)

install(
    TARGETS hello_publisher
    EXPORT hello_publisher
    LIBRARY DESTINATION lib/${PROJECT_NAME}
    ARCHIVE DESTINATION lib/${PROJECT_NAME}
    RUNTIME DESTINATION lib/${PROJECT_NAME}
    INCLUDES DESTINATION include/${PROJECT_NAME}
)