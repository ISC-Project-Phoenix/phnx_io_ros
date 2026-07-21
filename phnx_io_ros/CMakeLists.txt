cmake_minimum_required(VERSION 3.8)
project(phnx_io_ros)
include(FetchContent)

set(CMAKE_POLICY_DEFAULT_CMP0077 NEW)
if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

FetchContent_Declare(
        libackermann
        GIT_REPOSITORY https://github.com/iscumd/libackermann.git
)
FetchContent_MakeAvailable(libackermann)

FetchContent_Declare(
        CppLinuxSerial
        GIT_REPOSITORY https://github.com/gbmhunter/CppLinuxSerial.git
)
SET(BUILD_TESTS FALSE)
FetchContent_MakeAvailable(CppLinuxSerial)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(robot_state_msgs REQUIRED)
find_package(phnx_msgs REQUIRED)
find_package(phnx_control REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(phnx_io_ros src/phnx_io_ros.cpp src/serial.cpp src/phnx_io_ros_node.cpp src/pid_interface.cpp)

set(dependencies
        rclcpp
        std_msgs
        geometry_msgs
        ackermann_msgs
        robot_state_msgs
        nav_msgs
        phnx_msgs
        phnx_control
)
ament_target_dependencies(
        phnx_io_ros
        ${dependencies}
)
target_include_directories(phnx_io_ros PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)
target_compile_features(phnx_io_ros PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_link_libraries(phnx_io_ros libackermann)
target_link_libraries(phnx_io_ros CppLinuxSerial::CppLinuxSerial)

install(TARGETS phnx_io_ros
        DESTINATION lib/${PROJECT_NAME})

if (BUILD_TESTING)
    # Manually invoke clang format so it actually uses our file
    find_package(ament_cmake_clang_format REQUIRED)
    ament_clang_format(CONFIG_FILE ${CMAKE_CURRENT_SOURCE_DIR}/.clang-format)

    find_package(ament_cmake_gtest REQUIRED)

    # Add unit tests
    ament_add_gtest(${PROJECT_NAME}-test
            tests/unit.cpp
            src/phnx_io_ros_node.cpp
            src/serial.cpp
            src/pid_interface.cpp
    )
    ament_target_dependencies(${PROJECT_NAME}-test ${dependencies})
    target_include_directories(${PROJECT_NAME}-test PUBLIC
            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
            $<INSTALL_INTERFACE:include>)
    # you MUST link external libraries to both executables, else the headers are not used for either
    target_link_libraries(${PROJECT_NAME}-test libackermann)
    target_link_libraries(${PROJECT_NAME}-test CppLinuxSerial::CppLinuxSerial)
endif ()

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
