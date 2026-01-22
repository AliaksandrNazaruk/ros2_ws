# CommandResult.msg
# ROS event message for command result (transport-agnostic)

# Common fields
string command_id        # UUID v4 (protocol invariant)
builtin_interfaces/Time stamp  # Timestamp when result was generated

# Command identification
string target_id         # optional: target position ID (e.g., "dock_A")

# Result status
string status            # "succeeded" | "canceled" | "aborted" | "error"
string reason            # optional: error reason or message
