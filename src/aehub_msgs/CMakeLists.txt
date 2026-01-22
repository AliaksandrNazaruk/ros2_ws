cmake_minimum_required(VERSION 3.5)
project(aehub_msgs)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/BrokerConfig.msg"
  "msg/MqttEnvelope.msg"
  "msg/NavigationCommand.msg"
  "msg/CommandAck.msg"
  "msg/CommandResult.msg"
  "msg/NavigationState.msg"
  "msg/NavigationEvent.msg"
  "msg/NavigationStatus.msg"
  "msg/MapStatus.msg"
  "msg/RobotStatus.msg"
  "action/NavigationExecute.action"
  "srv/ListStations.srv"
  "srv/GetStation.srv"
  "srv/CreateStation.srv"
  "srv/UpdateStation.srv"
  "srv/DeleteStation.srv"
  DEPENDENCIES builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
