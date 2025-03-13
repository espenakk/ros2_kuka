add_executable(hello_client
    main.cpp
)

ament_target_dependencies(hello_client rclcpp std_srvs std_msgs ris_msgs)

target_include_directories(hello_client PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(hello_client
    atomic
    fmt::fmt-header-only
)

ament_export_targets(hello_client)

install(
    TARGETS hello_client
    EXPORT hello_client
    LIBRARY DESTINATION lib/${PROJECT_NAME}
    ARCHIVE DESTINATION lib/${PROJECT_NAME}
    RUNTIME DESTINATION lib/${PROJECT_NAME}
    INCLUDES DESTINATION include/${PROJECT_NAME}
)