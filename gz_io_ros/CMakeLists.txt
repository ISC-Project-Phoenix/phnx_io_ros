cmake_minimum_required(VERSION 3.8)
project(gz_io_ros)
include(FetchContent)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

FetchContent_Declare(
        libackermann
        GIT_REPOSITORY https://github.com/iscumd/libackermann.git
)
FetchContent_MakeAvailable(libackermann)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

add_executable(gz_io_ros src/gz_io_ros.cpp src/gz_io_ros_node.cpp)
target_include_directories(gz_io_ros PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)
target_compile_features(gz_io_ros PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_link_libraries(gz_io_ros libackermann)

set(dependencies
        rclcpp
        std_msgs
        geometry_msgs
        ackermann_msgs
        nav_msgs
        )
ament_target_dependencies(
        gz_io_ros
        ${dependencies}
)

install(TARGETS gz_io_ros
        DESTINATION lib/${PROJECT_NAME})

if (BUILD_TESTING)
    # Manually invoke clang format so it actually uses our file
    find_package(ament_cmake_clang_format REQUIRED)
    ament_clang_format(CONFIG_FILE ${CMAKE_CURRENT_SOURCE_DIR}/.clang-format)

    find_package(ament_cmake_gtest REQUIRED)

    # Add unit tests
    ament_add_gtest(${PROJECT_NAME}-test
            tests/unit.cpp
            src/gz_io_ros_node.cpp
            )
    ament_target_dependencies(${PROJECT_NAME}-test ${dependencies})
    target_include_directories(${PROJECT_NAME}-test PUBLIC
            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
            $<INSTALL_INTERFACE:include>)
    # you MUST link external libraries to both executables, else the headers are not used for either
    target_link_libraries(${PROJECT_NAME}-test libackermann)
endif ()

ament_package()
