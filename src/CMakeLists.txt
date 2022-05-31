cmake_minimum_required(VERSION 2.8.3)

include_directories(${ROS_LIB_DIR})

# Remove this if using an Arduino without native USB (eg, other than Leonardo)
# add_definitions(-DUSB_CON)

#register_hardware_platform(${ARDUINO_SDK_PATH}/hardware/esp8266)

print_board_list()

generate_arduino_firmware(alarm-light
  SRCS alarm-light.ino AlarmLight.cpp ${ROS_LIB_DIR}/time.cpp
  BOARD nodemcuv2
  PORT /dev/ttyACM0
)