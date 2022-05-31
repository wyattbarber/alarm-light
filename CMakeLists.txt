cmake_minimum_required(VERSION 2.8.3)
project(alarm-light)

find_package(catkin REQUIRED COMPONENTS
  rosserial_arduino
  rosserial_client
)

catkin_package()

rosserial_generate_ros_lib(
  PACKAGE rosserial_arduino smarthome
  SCRIPT make_libraries.py
)

rosserial_configure_client(
  DIRECTORY src
  TOOLCHAIN_FILE ${ROSSERIAL_ARDUINO_TOOLCHAIN}
)

rosserial_add_client_target(src alarm-light ALL)
rosserial_add_client_target(src alarm-light-upload)