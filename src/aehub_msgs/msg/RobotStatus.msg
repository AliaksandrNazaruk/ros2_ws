## AE.HUB RobotStatus
##
## A typed "robot state mirror" snapshot published by the robot interface layer
## (e.g. base_controller / hardware driver bridge).
##
## This message is designed for readiness/health checks:
## - It is observational only (no commands)
## - It carries validity flags for each field
## - Consumers should treat invalid or stale fields as NOT_READY (policy-dependent)
##
## Note: topic name should be relative (e.g. `robot/status`) and namespaced via launch.

builtin_interfaces/Time stamp

# Source / connectivity
bool driver_connected
string source

# Pose/velocity validity (from the driver/robot API), optional for readiness
bool pose_valid
bool velocity_valid

# Safety state (required for motion readiness)
bool motors_enabled_valid
bool motors_enabled
bool estop_active_valid
bool estop_active

# Free-form details (debug/telemetry). Should not be used for decisions.
string details
