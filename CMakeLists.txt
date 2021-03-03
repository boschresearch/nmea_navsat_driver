cmake_minimum_required(VERSION 3.0.2)
project(nmea_navsat_driver)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
)

catkin_python_setup()

add_message_files(
  FILES
  NavSatInfo.msg
  NavSatUbloxGeoFence.msg
  NavSatUbloxPositionVelocityTime.msg
  NavSatUbloxRelPos.msg
  NavSatUbloxPubxPosition.msg
  NavSatTrimbleHeading.msg
  NavSatTrimbleMovingBase.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
  std_msgs
  rospy
)

install(PROGRAMS
   scripts/nmea_serial_driver
   scripts/nmea_socket_driver
   scripts/nmea_tcp_driver
   scripts/nmea_topic_driver
   scripts/nmea_topic_serial_reader
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
   FILES_MATCHING PATTERN "*.launch"
)

install(DIRECTORY src
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
   FILES_MATCHING PATTERN "*.yaml"
)

if (CATKIN_ENABLE_TESTING)
  find_package(roslint)
  roslint_python()
  roslint_add_test()
endif()
