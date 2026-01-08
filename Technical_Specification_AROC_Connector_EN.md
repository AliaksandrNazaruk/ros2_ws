# Technical Specification: AROC Connector
## Detailed Status & Implementation Details

**Version:** 1.0  
**Date:** November 08, 2025  
**Author:** Aliaksandr Nazaruk  
**For:** BluPassion (Katja) & Matthias Heddinga

---

## 1. System Overview

The **AROC Connector** is a ROS2 node (C++17) running on the AE.01 robot, serving as a bridge between the cloud platform (AE.HUB) and the ROS2 ecosystem.

```
┌────────────────────────────────────────────────────────────────┐
│                        AE.HUB Cloud                             │
│  (React Frontend + Node.js Backend + MQTT Broker + Janus)      │
└──────────────────┬─────────────────┬────────────────────────────┘
                   │                 │
              WebRTC (Hot)      MQTT (Cold)
                   │                 │
┌──────────────────┴─────────────────┴────────────────────────────┐
│                     AROC Connector (ROS2 Node)                  │
│  ┌────────────────────────────────────────────────────────────┐ │
│  │  WebRTC Module      MQTT Module      REST API Module       │ │
│  │  • Video Encoding   • Command RX     • Status Endpoints    │ │
│  │  • Data Channel     • Status TX      • Config Management   │ │
│  │  • Janus Client     • JSON Parsing   • Health Checks       │ │
│  └────────────────────────────────────────────────────────────┘ │
│                             ↕                                   │
│  ┌────────────────────────────────────────────────────────────┐ │
│  │             ROS2 Jazzy Topics & Services                    │ │
│  │  • /image_raw (Camera)                                      │ │
│  │  • /cmd_vel (Motion Control)                                │ │
│  │  • /odom (Odometry)                                         │ │
│  │  • Nav2 Action Client                                       │ │
│  └────────────────────────────────────────────────────────────┘ │
└─────────────────────────────────────────────────────────────────┘
```

---

## 2. Component Status in Detail

### 2.1 WebRTC Video Streaming (Status: 70%)

**What Works:**
- ✅ ROS2 subscription to `/image_raw` topic (sensor_msgs/Image)
- ✅ Conversion from ROS Image to cv::Mat (OpenCV)
- ✅ H.264 encoding via GStreamer pipeline
- ✅ Integration with Janus Gateway REST API
- ✅ Video track publishing over WebRTC

**Implementation Details:**
```cpp
// Pseudo-code of video pipeline
class VideoStreamHandler {
  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
    cv::Mat frame = ros_image_to_cv_mat(msg);
    std::vector<uint8_t> h264_frame = encode_h264(frame);
    janus_client_->sendVideoFrame(h264_frame);
  }

  std::vector<uint8_t> encode_h264(const cv::Mat& frame) {
    // GStreamer Pipeline: appsrc ! x264enc ! h264parse ! appsink
    // Bitrate: 2000 kbps
    // Preset: ultrafast (for low latency)
    // Profile: baseline
  }
};
```

**Current Parameters:**
- **Resolution:** 1280x720 (hardcoded, should be configurable)
- **Framerate:** 30 FPS (targeted, currently ~25-28 FPS)
- **Bitrate:** 2000 kbps (hardcoded)
- **Codec:** H.264 Baseline Profile
- **Latency:** ~150-180ms (measured camera to browser)

**What's Missing (30%):**
- ⚠️ Adaptive bitrate (currently fixed at 2000 kbps)
- ⚠️ Reconnect logic on connection abort
- ⚠️ Configurable parameters (resolution, FPS, bitrate via config file)
- ⚠️ Error handling for GStreamer pipeline failures
- ⚠️ Statistics (packets lost, jitter, latency measurement)

**Next Steps:**
1. Config file integration (YAML) for video parameters
2. Implement reconnect logic
3. Latency monitoring (RTT measurement)

---

### 2.2 WebRTC Data Channel (Control) (Status: 50%)

**What Works:**
- ✅ Data Channel setup via Janus Gateway
- ✅ Receiving JSON messages via Janus REST API
- ✅ JSON parsing (nlohmann/json library)
- ✅ Basic mapping from JSON to ROS2 Twist message

**Implementation Details:**
```cpp
class DataChannelHandler {
  void onDataChannelMessage(const std::string& json_str) {
    nlohmann::json msg = nlohmann::json::parse(json_str);
    
    if (msg["type"] == "twist") {
      geometry_msgs::msg::Twist twist;
      twist.linear.x = msg["linear"]["x"];
      twist.angular.z = msg["angular"]["z"];
      
      cmd_vel_publisher_->publish(twist);
    }
  }
};
```

**Current Implementation:**
- **Polling:** Janus REST API is polled every 50ms for new messages (not ideal!)
- **Latency:** ~80-100ms from browser send to ROS2 publish
- **Frequency:** Currently only ~10 Hz due to polling limitation

**What's Missing (50%):**
- 🔴 WebSocket connection to Janus (instead of REST polling) → **Critical for < 50ms latency**
- ⚠️ Bidirectional communication (currently only RX, no feedback)
- ⚠️ Message buffering at high frequency
- ⚠️ Error handling for invalid JSON payloads
- ⚠️ Rate limiting (protection against DoS via too many messages)

**Next Steps:**
1. **Prio 1:** Migration from REST polling to WebSocket (Janus Admin API)
2. Implement bidirectional channel (feedback to operator)
3. Robust error handling

**Note for BluPassion:**  
The current implementation (REST polling) is a **proof-of-concept**. For production-ready < 50ms latency, WebSocket is mandatory. This is already on my roadmap for Sprint 1-2.

---

### 2.3 MQTT Client (Cold Path) (Status: 90%)

**What Works:**
- ✅ TLS-secured connection to Mosquitto broker
- ✅ Topic subscription: `aroc/robot/+/commands/#`
- ✅ Topic publishing: `aroc/robot/{ROBOT_ID}/status/#`
- ✅ JSON parsing for commands (nlohmann/json)
- ✅ Reconnect logic on connection abort
- ✅ QoS Level 1 (At Least Once)

**Implementation Details:**
```cpp
class MQTTHandler {
  void onCommandReceived(const std::string& topic, const std::string& payload) {
    if (topic.find("/commands/navigateTo") != std::string::npos) {
      nlohmann::json cmd = nlohmann::json::parse(payload);
      std::string target_id = cmd["target_id"];
      
      // Trigger Nav2 Action (when implemented)
      // Currently: Mock implementation
      startNavigationToTarget(target_id);
    }
  }

  void publishStatus(const std::string& status_type, const nlohmann::json& data) {
    std::string topic = "aroc/robot/" + robot_id_ + "/status/" + status_type;
    std::string payload = data.dump();
    mqtt_client_->publish(topic, payload, 1, false);
  }
};
```

**Current Configuration:**
- **Broker:** mqtts://mqtt.aroc-demo.de:8883
- **Client ID:** aroc_connector_{ROBOT_ID}_{UUID}
- **QoS:** 1 (At Least Once Delivery)
- **Keep-Alive:** 60 seconds
- **Reconnect:** Automatic with exponential backoff

**What's Missing (10%):**
- ⚠️ API finalization (final topic structure from BluPassion)
- ⚠️ Message schema validation (JSON Schema?)
- ⚠️ Telemetry data publishing (currently only commands/status)

**Next Steps:**
1. Wait for final API specification from BluPassion
2. Integration with actual Nav2 implementation
3. Add telemetry module

---

### 2.4 REST API (Status & Config) (Status: 60%)

**What Works:**
- ✅ HTTP server running on port 8080 (cpp-httplib library)
- ✅ Endpoint: `GET /api/v1/status` - Returns robot status
- ✅ Endpoint: `GET /api/v1/config` - Returns current configuration
- ✅ JSON response format

**Implementation Details:**
```cpp
class RESTAPIServer {
  void setupRoutes() {
    server_.Get("/api/v1/status", [this](const Request& req, Response& res) {
      nlohmann::json status = {
        {"robot_id", robot_id_},
        {"online", true},
        {"battery_percent", getBatteryLevel()},
        {"current_mode", getCurrentMode()},
        {"webrtc_connected", webrtc_handler_->isConnected()},
        {"mqtt_connected", mqtt_handler_->isConnected()}
      };
      res.set_content(status.dump(), "application/json");
    });
  }
};
```

**Available Endpoints:**

| Method | Endpoint | Description | Status |
|--------|----------|-------------|--------|
| GET | `/api/v1/status` | Query robot status | ✅ Functional |
| GET | `/api/v1/config` | Retrieve configuration | ✅ Functional |
| POST | `/api/v1/config` | Set configuration | ⚠️ Not implemented |
| GET | `/api/v1/health` | Health check | ⚠️ Not implemented |
| GET | `/api/v1/metrics` | Prometheus metrics | 🔴 Planned |

**What's Missing (40%):**
- ⚠️ Authentication (currently open, localhost-only)
- ⚠️ POST endpoints for configuration
- ⚠️ OpenAPI/Swagger documentation
- 🔴 Monitoring/metrics endpoint
- 🔴 CORS configuration (if called from external frontend)

**Next Steps:**
1. Auth integration (JWT token validation)
2. Create OpenAPI spec
3. Add monitoring endpoints

---

### 2.5 Authentication (Status: 30%)

**What Exists:**
- ✅ JWT token generation (POC code)
- ✅ Basic token validation logic
- ✅ Concept for API key management

**What's Missing (70%):**
- 🔴 Integration in REST API
- 🔴 Integration in WebRTC signaling
- 🔴 Integration in MQTT (username/password handling)
- 🔴 Token refresh mechanism
- 🔴 Key rotation strategy
- 🔴 Secrets management (currently hardcoded!)

**Planned Workflow:**
```
1. Robot starts → Reads API key from config file (environment variable)
2. Robot sends API key to AE.HUB backend
3. Backend validates, generates JWT token
4. Robot uses JWT for all further API calls
5. Token refresh before expiration (currently: 1h validity)
```

**Next Steps:**
1. **Sprint 1:** Basic integration (API key → JWT)
2. **Sprint 2:** Token refresh mechanism
3. **Sprint 3:** Production readiness (secrets management, key rotation)

**Workaround for MVP:**  
If auth integration is delayed: Temporary basic auth or API-key-only (not recommended, but functional).

---

### 2.6 Nav2 Integration (Status: 0%)

**Planned but not yet started:**
- 🔴 Implement ROS2 Nav2 Action Client
- 🔴 Goal sending to Nav2 stack
- 🔴 Feedback handling (progress, ETA)
- 🔴 Error handling (goal failed, obstacle detected)
- 🔴 Position mapping (position IDs → X/Y/Theta coordinates)

**Architecture Plan:**
```cpp
class Nav2Handler {
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  void navigateToPosition(const std::string& position_id) {
    auto goal_msg = NavigateToPose::Goal();
    goal_msg.pose = position_map_[position_id]; // Lookup from config
    
    auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    send_goal_options.feedback_callback = [this](auto, auto feedback) {
      publishNavigationProgress(feedback);
    };
    
    nav2_client_->async_send_goal(goal_msg, send_goal_options);
  }
};
```

**Next Steps:**
1. **Sprint 1 (Dec 2025):** Implement Nav2 Action Client
2. **Sprint 2:** Integration with MQTT command handler
3. **Sprint 2:** Testing with 5 positions

**Dependencies:**
- Position coordinates must be defined (together with customer/AROC)
- Nav2 stack must be correctly configured on robot (collision avoidance, local planner, etc.)

---

### 2.7 Auto-Align Feature (Status: 60%, but disabled)

**What Exists:**
- ✅ ArUco marker detection (OpenCV)
- ✅ Pose estimation based on marker
- ✅ Automatic alignment logic (PID controller)

**Why Disabled:**
- ⚠️ Only functional with additional camera (AE.01 currently has only one)
- ⚠️ Performance issues with simultaneous video streaming
- ⚠️ Not MVP-critical

**Future:**
- Will be reactivated in later versions when AE.01 gets second camera
- For MVP: Manual fine positioning via joystick

---

## 3. Performance Metrics (As of Nov 08, 2025)

### Measured on Nov 03, 2025 (Pilot Test Kleinostheim)

| Metric | Measured | Target | Status |
|--------|----------|--------|--------|
| **Video Latency (E2E)** | 150-180ms | < 200ms | ✅ OK |
| **Joystick Latency** | 80-100ms | < 100ms | ⚠️ Borderline |
| **MQTT Roundtrip** | 50-80ms | < 500ms | ✅ Very good |
| **CPU Usage** | 35-45% | < 60% | ✅ OK |
| **RAM Usage** | 420 MB | < 1 GB | ✅ OK |
| **Network (Downstream)** | 2.5 Mbps | < 5 Mbps | ✅ OK |
| **Network (Upstream)** | 0.1 Mbps | < 1 Mbps | ✅ OK |

**Test Scenario:**
- Network: 5G (LTE, ~30 Mbps down, ~10 Mbps up)
- Distance: ~200m between operator and robot
- Duration: 45 minutes continuous streaming

**Observations:**
- ✅ Video streaming very stable, no frame drops
- ✅ MQTT communication reliable
- ⚠️ Joystick control "acceptable", but not optimal due to polling

---

## 4. Technology Stack & Dependencies

### 4.1 Used Libraries

| Library | Version | Purpose |
|---------|---------|---------|
| **ROS2 Jazzy** | Latest | Robot middleware |
| **cpp-httplib** | v0.14.1 | HTTP server for REST API |
| **nlohmann/json** | v3.11.2 | JSON parsing & generation |
| **Eclipse Paho MQTT C++** | v1.3.1 | MQTT client |
| **OpenCV** | 4.8.0 | Image processing, encoding |
| **GStreamer** | 1.22.0 | H.264 encoding pipeline |
| **libcurl** | 7.88.1 | HTTP client (for Janus REST API) |

### 4.2 Build System

- **Build Tool:** colcon (ROS2 standard)
- **Compiler:** g++ 12.3 (C++17)
- **CMake:** 3.26
- **Package Manager:** apt (Ubuntu), rosdep (ROS dependencies)

### 4.3 System Requirements

**Hardware:**
- CPU: min. 4 cores (ARM64 or x86_64)
- RAM: min. 2 GB
- Network: Stable connection with min. 5 Mbps down, 1 Mbps up

**Software:**
- Ubuntu 24.04 LTS
- ROS2 Jazzy Jalisco
- GStreamer 1.22+
- OpenSSL 3.0+ (for TLS)

---

## 5. Configuration

### 5.1 Current Configuration (partially hardcoded)

**Config File:** `config/aroc_connector.yaml` (not yet fully implemented!)

```yaml
# Example config (partially not yet used)
robot:
  id: "fahrdummy-01"
  name: "AE.01 Fahrdummy"

mqtt:
  broker_url: "mqtts://mqtt.aroc-demo.de:8883"
  username: "${MQTT_USERNAME}"  # Environment variable
  password: "${MQTT_PASSWORD}"
  qos: 1
  keep_alive: 60

webrtc:
  janus_url: "https://janus.aroc-demo.de:8089/janus"
  janus_admin_ws: "wss://janus.aroc-demo.de:7889/admin"
  turn_server: "turn:turn.aroc-demo.de:3478"
  turn_username: "${TURN_USERNAME}"
  turn_password: "${TURN_PASSWORD}"
  video:
    resolution: "1280x720"
    framerate: 30
    bitrate: 2000000
    codec: "h264"

rest_api:
  port: 8080
  host: "0.0.0.0"

ros2:
  topics:
    image_input: "/image_raw"
    cmd_vel_output: "/cmd_vel"
    odom_input: "/odom"

positions:
  - id: "position_A"
    name: "Shelf A"
    x: 5.0
    y: 3.0
    theta: 0.0
  - id: "position_B"
    name: "Packing Station"
    x: 2.0
    y: 7.5
    theta: 1.57
  # ... more positions
```

**Problem:** Currently many values are still hardcoded in the code. Complete config file integration is planned for Sprint 1.

---

## 6. Known Issues & Limitations

### Critical (must be solved for MVP):

1. **WebRTC Data Channel Latency:** REST polling instead of WebSocket → too slow
   - **Impact:** Joystick control feels "laggy"
   - **Solution:** WebSocket migration (Prio 1 for Sprint 1)

2. **No Authentication:** All endpoints open
   - **Impact:** Security risk
   - **Solution:** JWT integration (Sprint 1-2)

3. **Hardcoded Parameters:** Config file not fully utilized
   - **Impact:** Difficult to deploy on different robots
   - **Solution:** Config management (Sprint 1)

### Medium (should be solved):

4. **No Reconnect Logic for WebRTC Abort**
   - **Impact:** On brief network interruptions, must reconnect manually
   - **Solution:** Automatic reconnect (Sprint 2)

5. **Missing Monitoring/Metrics**
   - **Impact:** No visibility in production
   - **Solution:** Prometheus metrics endpoint (Sprint 3)

### Low (nice-to-have):

6. **Auto-Align Feature Disabled**
   - **Impact:** Fine positioning manual
   - **Workaround:** Joystick control sufficient for MVP

---

## 7. API Documentation (Draft)

### 7.1 MQTT Topics (complete)

**Command Topics (Platform → Robot):**

| Topic | Payload | Description |
|-------|---------|-------------|
| `aroc/robot/{ROBOT_ID}/commands/navigateTo` | [see SRS Ch. 4.1] | Drive to position |
| `aroc/robot/{ROBOT_ID}/commands/cancel` | [see SRS Ch. 4.1] | Cancel |
| `aroc/robot/{ROBOT_ID}/commands/estop` | `{"timestamp": "..."}` | Emergency stop |

**Status Topics (Robot → Platform):**

| Topic | Payload | Description |
|-------|---------|-------------|
| `aroc/robot/{ROBOT_ID}/status/navigation` | [see SRS Ch. 4.1] | Navigation status |
| `aroc/robot/{ROBOT_ID}/status/system` | `{"cpu": float, "ram": float, "battery": int}` | System status |
| `aroc/robot/{ROBOT_ID}/status/connection` | `{"webrtc": bool, "mqtt": bool}` | Connection status |

### 7.2 REST API Endpoints (available)

**Base URL:** `http://{ROBOT_IP}:8080/api/v1`

| Method | Endpoint | Response | Status |
|--------|----------|----------|--------|
| GET | `/status` | [see Ch. 2.4] | ✅ Functional |
| GET | `/config` | [see Ch. 2.4] | ✅ Functional |
| GET | `/health` | `{"status": "ok"}` | ⚠️ Planned |

---

## 8. Testing & Quality Assurance

### 8.1 Tests Performed (As of Nov 08, 2025)

- ✅ **Unit Tests:** Basic functions (JSON parsing, topic subscriptions)
- ✅ **Integration Tests:** MQTT → ROS2 → MQTT loop
- ✅ **Field Test:** Pilot test Nov 03, 2025 in Kleinostheim (45 min, stable)
- ⚠️ **Stress Tests:** Not yet performed (planned for Sprint 2)
- ⚠️ **Security Tests:** Not yet performed

### 8.2 Test Environment

- **Simulation:** Gazebo Classic (for development without hardware)
- **Hardware:** AE.01 robot in Kleinostheim
- **Network:** 5G LTE (test), Ethernet (development)

---

## 9. Deployment

### 9.1 Installation on AE.01

```bash
# Install dependencies
sudo apt update
sudo apt install ros-jazzy-desktop libopencv-dev gstreamer1.0-tools

# Build AROC Connector
cd /home/aroc/aroc_ws
colcon build --packages-select aroc_connector

# Start
source install/setup.bash
ros2 run aroc_connector aroc_connector_node
```

### 9.2 Systemd Service (for autostart)

```ini
[Unit]
Description=AROC Connector Node
After=network.target

[Service]
Type=simple
User=aroc
WorkingDirectory=/home/aroc/aroc_ws
ExecStart=/bin/bash -c 'source /home/aroc/aroc_ws/install/setup.bash && ros2 run aroc_connector aroc_connector_node'
Restart=always
RestartSec=10

[Install]
WantedBy=multi-user.target
```

---

## 10. Contact & Support

For technical questions about this document:

**Aliaksandr Nazaruk**  
Email: (via Matthias Heddinga)  
Preferred communication: In writing (email, shared docs)

**Matthias Heddinga** (Coordinator)  
Email: matthias.heddinga@aroc-technologies.com  
Role: Technical coordination, translation, project management

---

**End of Technical Specification**

**Version:** 1.0  
**Date:** November 08, 2025  
**Next Update:** After Sprint 1 (end of December 2025)
