# NavigationCommand.msg
# Universal command envelope for transport → executor communication
# Transport-agnostic: executor does not know about MQTT
# Business-agnostic: transport does not know about Nav2

# Common fields (ALL commands)
string command_id        # UUID v4 (protocol invariant)
string type              # "navigateTo" | "cancel" (command type)
builtin_interfaces/Time stamp  # Timestamp when command was created

# Fields for navigateTo command
string target_id         # optional: target position ID (e.g., "dock_A")
float64 x                # optional: X coordinate
float64 y                # optional: Y coordinate  
float64 theta            # optional: orientation angle (radians)

# Note: For navigateTo, either target_id OR (x, y) must be provided
# For cancel, only command_id and type are required
