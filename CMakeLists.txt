cmake_minimum_required(VERSION 3.8)
project(device_pack LANGUAGES C CXX)

set(CMAKE_C_STANDARD 99)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_library(motor_hardware SHARED
  src/DEV_Config.c
  src/motor_driver.c
)
target_include_directories(motor_hardware
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_compile_definitions(motor_hardware PUBLIC USE_WIRINGPI_LIB=1)
target_link_libraries(motor_hardware PUBLIC wiringPi)

add_executable(test_motor src/test_motor.c)
target_link_libraries(test_motor motor_hardware)

add_executable(motor_node src/motor_node.cpp)
target_link_libraries(motor_node motor_hardware)
ament_target_dependencies(motor_node rclcpp geometry_msgs std_msgs)

install(TARGETS motor_hardware test_motor motor_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)

ament_export_include_directories(include)
ament_export_libraries(motor_hardware)
ament_package()
