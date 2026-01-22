# MqttEnvelope.msg
#
# Transport-agnostic envelope for MQTT transport layer.
# Transport does NOT parse JSON. Protocol adapter may parse payload_raw as JSON.

string mqtt_topic
string payload_raw
builtin_interfaces/Time received_at

