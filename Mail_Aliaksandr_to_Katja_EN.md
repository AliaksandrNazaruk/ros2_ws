# Email Draft: Aliaksandr to Katja (BluPassion)
## Technical Status & Proposal for Written Coordination

**From:** Aliaksandr Nazaruk (Lead Developer - AROC Connector)  
**To:** Katja (BluPassion)  
**CC:** Matthias Heddinga (AROC Technologies GmbH)  
**Subject:** AE.HUB MVP - Technical Status & Written Coordination Proposal

---

## Email Text

Hi Katja,

I hope this email finds you well! Matthias asked me to give you a detailed technical status of our current AROC Connector implementation and suggest an efficient way for our collaboration going forward.

**Important note on communication:**  
To avoid misunderstandings and ensure precise technical coordination, I propose that we primarily **communicate in writing** - via email, shared documents, and technical diagrams. This allows both of us:
- Time for thorough consideration and precise formulation
- Documented agreements as reference
- Asynchronous clarification without timezone constraints
- Clear technical specifications we can both refer to

Of course, Matthias can coordinate between us whenever needed for complex topics.

---

## 1. Current Implementation Status (Honest Assessment)

Here's a transparent overview of where the AROC Connector stands today:

| Component | Status | Completion | Notes |
|-----------|--------|------------|-------|
| **WebRTC Video Streaming** | ✅ Functional | 70% | H.264 encoding works, improving stability |
| **WebRTC Control (Data Channel)** | ⚠️ In Development | 50% | Message passing via Janus Gateway, adaptation in progress |
| **MQTT Cold Path** | ✅ Mostly Complete | 90% | Waiting for final API specification |
| **REST API** | ⚠️ Functional | 60% | Core functions available, refining documentation |
| **Authentication** | 🔴 Early Stage | 30% | Token generation exists, integration pending |
| **Nav2 Integration** | 🔴 Not Started | 0% | Planned for Sprint 1-2 |
| **Auto-Align Feature** | ⚠️ Exists | 60% | Available but currently disabled |

**What this means for you:**

✅ **You can start development immediately** - Video API, MQTT, and basic REST API are functional

⚠️ **Some areas still have rough edges** - Hardcoded parameters, missing auth, documentation gaps

🔴 **Production readiness:** Needs 2-4 weeks of parallel work on identified weaknesses

---

## 2. Technical Architecture Overview

### 2.1 Dual-Path Communication Architecture

Our system uses two parallel communication paths:

**Hot Path (WebRTC) - Real-time:**
```
AE.Cockpit (Browser) ←→ Janus Gateway ←→ AROC Connector (ROS2)
                    ↑                              ↑
                  Video (H.264)            Control Commands
                  < 200ms Latency           (JSON via Data Channel)
```

**Cold Path (MQTT) - Asynchronous:**
```
AE.HUB Backend ←→ MQTT Broker (Mosquitto) ←→ AROC Connector (ROS2)
                        ↑                              ↑
                  Commands                     Status Updates
                  (navigateTo)                 (Position, Progress)
```

### 2.2 Current Technology Stack

**AROC Connector (Robot Side):**
- Language: C++17
- Framework: ROS2 Jazzy
- WebRTC: Janus Gateway Integration
- MQTT Client: Eclipse Paho
- Video: H.264 Encoding via GStreamer
- OS: Ubuntu 24.04

**Infrastructure (DMZ):**
- TURN Server: coturn
- MQTT Broker: Mosquitto with TLS
- Ports: 443, 8883 (MQTT), 3478/5349 (TURN)

---

## 3. What's Already Working (As of November 2025)

### ✅ Implemented and Tested:

1. **Video Streaming:**
   - ROS2 Topic `/image_raw` subscription working
   - H.264 encoding stable
   - Transmission via WebRTC Video Track
   - Successful test on 03.11.2025 in Kleinostheim

2. **MQTT Communication:**
   - Secure TLS connection to broker
   - Topic structure: `aroc/robot/{ROBOT_ID}/commands/...`
   - JSON parsing for commands
   - Status publishing functional

3. **REST API (Basic):**
   - Query robot status
   - Retrieve basic configuration
   - API running on port 8080

### ⚠️ In Progress / Needs Improvement:

1. **WebRTC Data Channel:**
   - Bidirectional communication via Janus
   - Receive joystick data (JSON)
   - Mapping to ROS2 `/cmd_vel` topic
   - **Status:** Basic function available, optimization in progress

2. **Authentication:**
   - JWT token validation
   - API key management
   - **Status:** Concept available, integration pending

3. **Configuration:**
   - Many parameters still hardcoded
   - Config file management being implemented

### 🔴 Not Yet Started:

1. **Nav2 Navigation:**
   - Integration with ROS2 Nav2 Stack
   - Autonomous navigation to positions
   - **Planned:** Sprint 1-2 (December 2025)

2. **Production Readiness:**
   - Error handling
   - Logging and monitoring
   - Health checks
   - **Planned:** Sprint 3-4 (January 2026)

---

## 4. API Specification & Documentation

### 4.1 MQTT Interface (Cold Path)

**Command Topic Structure:**
```
aroc/robot/{ROBOT_ID}/commands/navigateTo
```

**Example Command Payload (JSON):**
```json
{
  "command_id": "d290f1ee-6c54-4b01-90e6-d701748f0851",
  "timestamp": "2025-11-08T14:30:00Z",
  "target_id": "position_A",
  "priority": "normal"
}
```

**Status Topic Structure:**
```
aroc/robot/{ROBOT_ID}/status/navigation
```

**Example Status Payload (JSON):**
```json
{
  "robot_id": "fahrdummy-01",
  "timestamp": "2025-11-08T14:30:05Z",
  "status": "navigating",
  "target_id": "position_A",
  "progress_percent": 45,
  "current_position": {
    "x": 2.5,
    "y": 3.8,
    "theta": 0.75
  }
}
```

### 4.2 WebRTC Interface (Hot Path)

**Signaling Process:**
1. Client sends SDP Offer to Janus Gateway
2. AROC Connector receives Offer via Janus API
3. Connector sends SDP Answer back
4. ICE Candidates are exchanged
5. P2P connection established

**Data Channel Format for Joystick:**
```json
{
  "type": "twist",
  "linear": {
    "x": 0.5,
    "y": 0.0,
    "z": 0.0
  },
  "angular": {
    "x": 0.0,
    "y": 0.0,
    "z": 0.2
  },
  "timestamp": "2025-11-08T14:30:10.123Z"
}
```

Frequency: 10-20 Hz (recommended: 20 Hz for smooth control)

### 4.3 REST API (Status & Configuration)

**Base URL:** `http://{ROBOT_IP}:8080/api/v1`

**Available Endpoints:**

1. **Status Query:**
   ```
   GET /api/v1/status
   ```
   Response:
   ```json
   {
     "robot_id": "fahrdummy-01",
     "online": true,
     "battery_percent": 87,
     "current_mode": "idle",
     "webrtc_connected": false,
     "mqtt_connected": true
   }
   ```

2. **Configuration Retrieval:**
   ```
   GET /api/v1/config
   ```
   Response:
   ```json
   {
     "janus_server": "wss://janus.aroc-demo.de:8989",
     "mqtt_broker": "mqtts://mqtt.aroc-demo.de:8883",
     "robot_id": "fahrdummy-01"
   }
   ```

**Note:** Detailed API documentation (OpenAPI/Swagger) is currently being created and will follow as a separate document.

---

## 5. Proposed Coordination Process

To ensure efficient and precise technical collaboration, I propose the following **written coordination process**:

### 5.1 Initial API Clarification (Next 5-7 Days)

**My Proposal:**
1. **You send me your technical questions** via email
   - Which additional API endpoints do you need?
   - Which JSON fields are missing in the payloads?
   - Which authentication method do you prefer?

2. **I respond in writing** with:
   - Detailed technical specifications
   - Code examples where necessary
   - Architecture diagrams for visualization

3. **We iterate** via shared documents:
   - Google Doc / Markdown file with API specification
   - Each makes change proposals
   - Matthias can mediate when unclear

4. **Finalization:**
   - Joint "API Contract Document"
   - Confirmed by both sides
   - Serves as binding development basis

### 5.2 Ongoing Coordination During Development

**Communication Channels:**
- **Email:** For basic questions and status updates
- **Shared Documents:** For detailed technical specifications
- **Diagrams:** For architecture and data flows
- **Matthias as Coordinator:** For complex coordination

**Rhythm:**
- **Weekly Status Update:** Every Friday via email
- **Ad-hoc Clarifications:** Anytime via email with 24h response time

### 5.3 Escalation Path

For unclear points or blockers:
1. Detailed email with problem description
2. Matthias coordinates between us
3. If necessary: Short video tutorial from me with screen recording
4. Last resort: Meeting with Matthias as interpreter

---

## 6. Timeline & Next Steps

### Proposed Schedule:

**CW 45/2025 (04.-08.11.2025):**
- ✅ This email with status overview
- ⏳ Your feedback with initial questions

**CW 46/2025 (11.-15.11.2025):**
- API clarification via document
- Finalization of "API Contract"
- Matthias creates final specification

**CW 47-50/2025 (18.11.-13.12.2025):**
- **Your work:** Backend development, basic UI, authentication
- **My work:** Make connector production-ready, auth integration, config management
- **Integration:** REST API + MQTT (already functional)

**CW 51/2025 - CW 4/2026 (16.12.2025-24.01.2026):**
- **Your work:** WebRTC integration, control UI, navigation features
- **My work:** Nav2 integration, Data Channel optimization
- **Integration:** WebRTC Signaling + Data Channel

**CW 5-8/2026 (27.01.-21.02.2026):**
- End-to-end testing
- Bug fixing
- Performance optimization
- Demo preparation

**MVP Demo Date:** February 28, 2026

### My Next Actions:

1. **Immediately:** Send this email
2. **Upon your response:** Answer your questions in detail
3. **This week:** Finalize API documentation (OpenAPI)
4. **Next week:** Start auth integration

---

## 7. Attachments (see separate documents)

I've prepared the following detailed documents for you:

1. **Technical Specification AROC Connector** (`Technical_Specification_AROC_Connector_EN.md`)
   - Complete component overview
   - Implementation details
   - Performance metrics

2. **Architecture Diagrams** (`Architecture_Diagrams.pdf`)
   - System overview
   - Data flow Hot Path & Cold Path
   - Network topology

3. **API Reference (Draft)** (`API_Reference_Draft_EN.md`)
   - All endpoints with examples
   - Error codes
   - Authentication

---

## 8. Closing Remarks

I'm very much looking forward to collaborating with you and your team at BluPassion. The architecture is solid, and I'm confident we'll develop a robust solution together.

Please let me know if you:
- Need additional information
- Have specific technical questions
- Need adjustments to the API
- Want to change the proposed coordination process

I typically respond within 24 hours to emails.

**Next Steps:**
1. ✅ You read this email and the attachments
2. ⏳ You send me your initial questions and comments
3. ⏳ We finalize the API specification together
4. ✅ Let's get started with parallel development!

Best regards,

**Aliaksandr Nazaruk**  
Lead Developer - AROC Connector  
X-elence Automation GmbH  
On behalf of AROC Technologies GmbH

---

**CC:** Matthias Heddinga (AROC Technologies GmbH)  
**Attachments:** 
- Technical_Specification_AROC_Connector_EN.md
- Architecture_Diagrams.pdf  
- API_Reference_Draft_EN.md
