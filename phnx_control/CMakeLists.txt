cmake_minimum_required(VERSION 3.8)
project(phnx_control)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(control_toolbox REQUIRED)
find_package(phnx_msgs REQUIRED)

add_library(phnx_control SHARED src/speed_control.cpp)
target_include_directories(phnx_control PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)
target_compile_features(phnx_control PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

set(deps
        rclcpp
        geometry_msgs
        ackermann_msgs
        nav_msgs
        control_toolbox
        phnx_msgs
)

ament_target_dependencies(
        phnx_control ${deps}
)

install(TARGETS
        phnx_control
        EXPORT export_${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

install(
        DIRECTORY include/
        DESTINATION include/
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

ament_export_targets(
        export_${PROJECT_NAME} HAS_LIBRARY_TARGET
)
ament_export_dependencies(${deps})
ament_package()
