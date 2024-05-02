cmake_minimum_required(VERSION 3.0.2)
project(livelybot_serial)

# Find catkin
find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    livelybot_msg  
    serial
)
set(rt_LIBRARIES rt)
set(pthread_LIBRARIES pthread)
set(serial_SRCS
    include/hardware/robot.h
    include/hardware/motor.h
    src/lively_serial.cc
    src/hardware/motor.cc
)
catkin_package(
    LIBRARIES ${PROJECT_NAME}
    INCLUDE_DIRS include
    DEPENDS rt pthread
    CATKIN_DEPENDS roscpp std_msgs livelybot_msg serial
)
add_library(${PROJECT_NAME} ${serial_SRCS})
target_link_libraries(${PROJECT_NAME} rt pthread)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
# test

add_executable(testArm src/testArm.cpp ${serial_SRCS})
target_link_libraries(testArm ${catkin_LIBRARIES})

install(TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

## Install headers
install(FILES include/hardware/robot.h include/hardware/motor.h
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}/livelybot_serial)
