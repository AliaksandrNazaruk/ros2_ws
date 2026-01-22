# NavigationStatus.msg
#
# Minimal navigation telemetry for AE.HUB UI (machine-owned backend profile).
#
# Published periodically by the navigation backend server.
#
# Conventions:
# - status: navigating|arrived|error|canceled|idle
# - progress_percent: 0..100 (best-effort)
# - eta_seconds: -1 if unknown
#

builtin_interfaces/Time stamp

string status
string command_id
string target_id

float32 progress_percent
int32 eta_seconds

float64 current_x
float64 current_y
float64 current_theta

