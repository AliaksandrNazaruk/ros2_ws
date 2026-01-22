# NavigationEvent.msg
#
# Single event stream for navigation orchestration.
# Rule: consumers subscribe to ONE event topic only.
#
# event_type: "ack" | "result" | "state" | "health"

builtin_interfaces/Time stamp
string event_type

# Common identifiers (may be empty depending on event_type)
string command_id
string target_id

# ACK fields (event_type == "ack")
string ack_status      # received|accepted|rejected
string ack_reason

# RESULT fields (event_type == "result")
string result_status   # succeeded|canceled|aborted|error
string result_reason

# STATE fields (event_type == "state")
string public_state    # idle|navigating|canceling|succeeded|aborted|canceled|error
string internal_state

