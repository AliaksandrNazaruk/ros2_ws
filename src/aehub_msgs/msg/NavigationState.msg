# NavigationState.msg
# ROS event message for navigation state snapshot (transport-agnostic)

# State snapshot timestamp
builtin_interfaces/Time stamp

# Public state (for external systems)
string public_state      # "idle" | "navigating" | "canceled"

# Active command (if any)
string active_command_id # optional: currently active command ID
string active_target_id  # optional: currently active target ID

# Internal state (for debugging/monitoring)
string internal_state    # optional: internal FSM state
