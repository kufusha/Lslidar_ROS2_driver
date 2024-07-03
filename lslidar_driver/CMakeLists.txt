cmake_minimum_required(VERSION 3.5)
project(lslidar_driver)

set(CMAKE_BUILD_TYPE Release)    #RelWithDebInfo
#add_compile_options(-std=c++11)
# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
	set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
	add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(libpcap_LIBRARIES -lpcap)
#set(FastRTPS_INCLUDE_DIR /opt/ros/foxy/include)
#set(FastRTPS_LIBRARY_RELEASE /opt/ros/foxy/lib/libfastrtps.so)

find_package(Boost REQUIRED)
find_package(angles REQUIRED)
find_package(PCL REQUIRED)
find_package(lslidar_msgs REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
#find_package(Boost REQUIRED COMPONENTS thread)
find_package(sensor_msgs REQUIRED)
find_package(rclpy REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PkgConfig REQUIRED)

set(lslidar_driver_executable lslidar_driver)
set(lslidar_driver_library ${lslidar_driver_executable}_core)
set(dependencies
  rclcpp
  rclcpp_lifecycle
  rclcpp_components
  std_msgs
  lslidar_msgs
  sensor_msgs
  pcl_conversions
)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)


# c16 Node
add_executable(${lslidar_driver_executable}
  src/lslidar_driver_node.cpp
  src/lslidar_driver.cpp
  src/flag_handler.cpp
  src/input.cpp
)
add_library(${lslidar_driver_library} SHARED
  src/lslidar_driver.cpp
  src/flag_handler.cpp
  src/input.cpp
)
target_link_libraries(${lslidar_driver_executable}
  ${rclcpp_LIBRARIES}
  ${libpcap_LIBRARIES}
  ${Boost_LIBRARIES}
)
target_link_libraries(${lslidar_driver_library}
  ${rclcpp_LIBRARIES}
  ${libpcap_LIBRARIES}
  ${Boost_LIBRARIES}
)
ament_target_dependencies(${lslidar_driver_executable}
  ${dependencies}
)
ament_target_dependencies(${lslidar_driver_library}
  ${dependencies}
)
rclcpp_components_register_nodes(${lslidar_driver_library}
  "lslidar_driver::LslidarDriver"
)

install(DIRECTORY launch params rviz_cfg
  DESTINATION share/${PROJECT_NAME}
)
install(TARGETS ${lslidar_driver_executable}
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS ${lslidar_driver_library}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/
  DESTINATION include/
)

ament_export_include_directories(
  include
  ${PCL_COMMON_INCLUDE_DIRS}
)
ament_export_libraries(${lslidar_driver_library})
ament_export_dependencies(${dependencies})

add_definitions("-D${PROJECT_NAME}_VERSION=\"${${PROJECT_NAME}_VERSION}\"")
ament_package()
