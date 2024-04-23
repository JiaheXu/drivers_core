cmake_minimum_required(VERSION 3.20)

project(drivers_core)

set(CMAKE_CXX_STANDARD 14)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)

find_package(sensor_msgs REQUIRED)
find_package(cansend REQUIRED)


include_directories(
  ${colcon_INCLUDE_DIRS}
  ${cansend_INCLUDE_DIRS}
)

add_executable(vesc_interface src/vesc_interface.cpp)
ament_target_dependencies(vesc_interface 
  rclcpp 
  std_msgs 
  geometry_msgs 
  std_srvs
  rclcpp_action
  cansend
)

ament_export_include_directories(include)
ament_export_libraries(export_${PROJECT_NAME})


install(TARGETS
  vesc_interface
  DESTINATION lib/${PROJECT_NAME}
)
# Mark directories for installation
install(
   DIRECTORY launch DESTINATION share/${PROJECT_NAME}
)

ament_package()