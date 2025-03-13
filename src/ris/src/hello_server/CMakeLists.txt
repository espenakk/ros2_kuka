add_executable(hello_server
    main.cpp
)

ament_target_dependencies(hello_server rclcpp std_srvs std_msgs ris_msgs)

target_include_directories(hello_server PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(hello_server
    atomic
    fmt::fmt-header-only
)

ament_export_targets(hello_server)

install(
    TARGETS hello_server
    EXPORT hello_server
    LIBRARY DESTINATION lib/${PROJECT_NAME}
    ARCHIVE DESTINATION lib/${PROJECT_NAME}
    RUNTIME DESTINATION lib/${PROJECT_NAME}
    INCLUDES DESTINATION include/${PROJECT_NAME}
)