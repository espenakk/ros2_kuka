add_executable(hello_world
    main.cpp
)

ament_target_dependencies(hello_world rclcpp std_srvs std_msgs ris_msgs)

target_include_directories(hello_world PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(hello_world
    atomic
    fmt::fmt-header-only
)

ament_export_targets(hello_world)

install(
    TARGETS hello_world
    EXPORT hello_world
    LIBRARY DESTINATION lib/${PROJECT_NAME}
    ARCHIVE DESTINATION lib/${PROJECT_NAME}
    RUNTIME DESTINATION lib/${PROJECT_NAME}
    INCLUDES DESTINATION include/${PROJECT_NAME}
)