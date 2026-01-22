# CommandAck.msg
# ROS event message for command acknowledgment (transport-agnostic)

# Common fields
string command_id        # UUID v4 (protocol invariant)
builtin_interfaces/Time stamp  # Timestamp when ACK was generated

# Command identification
string target_id         # optional: target position ID (e.g., "dock_A")

# ACK status
string status            # "received" | "accepted" | "rejected"
string reason            # optional: rejection reason (if status == "rejected")
