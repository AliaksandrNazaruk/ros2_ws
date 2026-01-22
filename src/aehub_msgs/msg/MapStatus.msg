# Map status / mirror output.
#
# Published by Symovo map mirror (ROS interface layer) to drive localization bringup.
# All fields are within `/robot/<id>/...` namespace via launch.

builtin_interfaces/Time stamp

# Symovo map identifier (stringified; some deployments use UUID-like strings).
string map_id

# Monotonic-ish revision from Symovo (`last_changed`), used for change detection.
int64 revision

float32 resolution
float32 offset_x
float32 offset_y

uint32 width_px
uint32 height_px

# Absolute path to YAML file suitable for nav2_map_server.
string yaml_path

# Geometry policy used for pose conversion (string enum; see map mirror params).
string pose_transform_mode
string origin_mode

# True if map artifacts are ready and consistent.
bool ready

# Human-readable error if ready==false.
string error

