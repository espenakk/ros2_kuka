add_executable(hello_subscriber
    main.cpp
)

ament_target_dependencies(hello_subscriber rclcpp std_srvs std_msgs ris_msgs)

target_include_directories(hello_subscriber PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(hello_subscriber
    atomic
    fmt::fmt-header-only
)

ament_export_targets(hello_subscriber)

install(
    TARGETS hello_subscriber
    EXPORT hello_subscriber
    LIBRARY DESTINATION lib/${PROJECT_NAME}
    ARCHIVE DESTINATION lib/${PROJECT_NAME}
    RUNTIME DESTINATION lib/${PROJECT_NAME}
    INCLUDES DESTINATION include/${PROJECT_NAME}
)