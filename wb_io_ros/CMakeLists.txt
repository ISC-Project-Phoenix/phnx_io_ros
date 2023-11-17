cmake_minimum_required(VERSION 3.8)
project(wb_io_ros)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(webots_ros2_driver REQUIRED)
find_package(pluginlib REQUIRED)

# Export the plugin configuration file
pluginlib_export_plugin_description_file(webots_ros2_driver wb_io_ros.xml)

add_library(wb_io_ros SHARED src/wb_io_ros.cpp)
target_include_directories(wb_io_ros PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)
target_compile_features(wb_io_ros PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
        wb_io_ros
        "rclcpp"
        "geometry_msgs"
        "webots_ros2_driver"
        "ackermann_msgs"
        "pluginlib"
        "nav_msgs"
)

install(TARGETS
        wb_io_ros
        EXPORT export_${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

if (BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    # the following line skips the linter which checks for copyrights
    # comment the line when a copyright and license is added to all source files
    set(ament_cmake_copyright_FOUND TRUE)
    # the following line skips cpplint (only works in a git repo)
    # comment the line when this package is in a git repo and when
    # a copyright and license is added to all source files
    set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
endif ()

ament_export_include_directories(
        include
)
ament_export_libraries(
        wb_io_ros
)
ament_export_targets(
        export_${PROJECT_NAME}
)
ament_package()
