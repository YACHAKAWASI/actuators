cmake_minimum_required(VERSION 3.16)
project(actuator_motor LANGUAGES C CXX)

# ==== Estándares y flags ====
set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
add_compile_options(-Wall -Wextra -Wno-unused-parameter -fPIC)

# ==== ROS 2 ====
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# ==== Backends (Pi 5: sin WiringPi) ====
add_compile_definitions(
  USE_WIRINGPI_LIB=0
  USE_GPIOD_LIB=1
  USE_I2CDEV_LIB=1
  USE_SPIDEV_LIB=0
  DEV_I2C=1
  DEV_SPI=0
  # Descomenta si activaste PWM por software en motor_driver.c
  # USE_SW_PWM=1
)

# ==== Includes ====
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

# ==== Librería de hardware ====
add_library(motor_hardware SHARED
  src/DEV_Config.c
  src/motor_driver.c
)
target_include_directories(motor_hardware
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
# Enlace a libgpiod y pthread (pthread se usa si activas PWM software)
target_link_libraries(motor_hardware gpiod pthread)

# ==== Ejecutable de prueba ====
#add_executable(test_motor src/test_motor.c)
#target_link_libraries(test_motor motor_hardware)  # firma plain

# ==== Nodo ROS 2 ====
add_executable(motor_node src/motor_node.cpp)
target_link_libraries(motor_node motor_hardware)  # firma plain
ament_target_dependencies(motor_node rclcpp geometry_msgs std_msgs)

# ==== Instalación ====
#install(TARGETS motor_hardware test_motor motor_node
#  ARCHIVE DESTINATION lib
  #LIBRARY DESTINATION lib
 # RUNTIME DESTINATION lib/${PROJECT_NAME}
#)
install(DIRECTORY include/ DESTINATION include)

ament_export_include_directories(include)
ament_export_libraries(motor_hardware)
ament_package()
