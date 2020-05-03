cmake_minimum_required(VERSION 3.9.5)
project(rplidar_ros)

if (NOT CMAKE_C_STANDARD)
    set(CMAKE_C_STANDARD 99)
endif ()

# Default to C++14
if (NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif ()
if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -pedantic)
endif ()

# Create sdk library
add_library(rplidar_sdk SHARED
        sdk/src/rplidar_driver.cpp

        sdk/src/arch/linux/net_serial.cpp
        sdk/src/arch/linux/net_socket.cpp
        sdk/src/arch/linux/timer.cpp

        sdk/src/hal/thread.cpp)

target_include_directories(rplidar_sdk
        PUBLIC
        sdk/include
        sdk/src
        PRIVATE
        sdk/src/arch/linux
        sdk/src/hal)

find_package(ament_cmake_auto REQUIRED)
find_package(ament_cmake_ros REQUIRED)

# Can't use ament_target_dependencies here, so need to manually
# create targets for ros2 dependencies
list(APPEND ros2_dependencies
        rclcpp
        sensor_msgs
        std_srvs
        rclcpp_components)

# create targets from ros 2 packages
foreach (dep IN LISTS ros2_dependencies)
    find_package(${dep} REQUIRED)
    add_library(${dep} INTERFACE)
    target_link_libraries(${dep} INTERFACE ${${dep}_LIBRARIES})
    target_include_directories(${dep} INTERFACE ${${dep}_INCLUDE_DIRS})
endforeach ()

ament_auto_find_build_dependencies(REQUIRED
        rclcpp
        sensor_msgs
        std_srvs
        rclcpp_components)

add_library(rplidar_node SHARED src/RPLidarNode.cpp)
target_link_libraries(rplidar_node rclcpp sensor_msgs std_srvs rclcpp_components rplidar_sdk)
ament_target_dependencies(rplidar_node rclcpp sensor_msgs std_srvs rclcpp_components)
target_include_directories(rplidar_node PUBLIC include)

# build direct ROS 1 port
ament_auto_add_executable(rplidar_client src/client.cpp)
target_link_libraries(rplidar_client rclcpp sensor_msgs std_srvs rclcpp_components)
ament_target_dependencies(rplidar_client rclcpp sensor_msgs std_srvs rclcpp_components)

# build composition node
ament_auto_add_executable(rplidar src/rplidar.cpp)
target_link_libraries(rplidar rplidar_node)

install(TARGETS rplidar_client rplidar rplidar_sdk
        DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch rviz config
        DESTINATION share/${PROJECT_NAME})

# Install the shared library composition node
install(TARGETS rplidar_node
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin)

ament_auto_package()
