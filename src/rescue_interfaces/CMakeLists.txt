cmake_minimum_required(VERSION 3.8)
project(rescue_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ArmStatus.msg"
  "msg/CartesianWaypoint.msg"
  "msg/CartesianState.msg"
  "srv/RegisterServo.srv"
  "srv/ServoCommand.srv"
  "srv/ComputeIK.srv"
  "srv/ComputeIKPose.srv"
  "srv/ServoStatus.srv"
  "srv/CartesianGoto.srv"
  "srv/CartesianTrajectory.srv"
  DEPENDENCIES std_msgs
)

ament_package()
