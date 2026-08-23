cmake_minimum_required(VERSION 3.10)
project(common_msgs)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  sensor_msgs
  std_msgs
)

add_message_files(
  FILES
  GPSVelocity.msg
  MaxDynamics.msg
  MeasurementPosVel.msg
  MeasurementPosition.msg
  MeasurementVelocity.msg
  Mission.msg
  MissionStatus.msg
  NavigationState.msg
  PathPlanningStatus.msg
  PixhawkCMD.msg
  PixhawkServo.msg
  PositionWithCovarianceStamped.msg
  Reference.msg
  UWB_DataInfo.msg
  UWB_EchoedRangeInfo.msg
  UWB_FullNeighborDatabase.msg
  UWB_FullNeighborDatabaseEntry.msg
  UWB_FullRangeInfo.msg
  UWB_SendData.msg
  Waypoint.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  sensor_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs message_runtime sensor_msgs std_msgs
)
