# Software Requirements Specification: AE.HUB MVP "Driving Dummy"
## For BluPassion Development Partnership

**Version:** Draft 0.3  
**Date:** November 08, 2025  
**Client:** AROC Technologies GmbH  
**Recipient:** BluPassion (Software Partner)  
**Project Lead:** Matthias Heddinga  
**Lead Developer (AROC):** Aliaksandr Nazaruk

---

## 📋 Document Status

| Version | Date | Changes | Author |
|---------|------|---------|--------|
| 1.0 | 21.07.2025 | Initial version (concept phase) | Matthias + AI |
| 1.1 | 21.08.2025 | Refinement after workshop | Matthias |
| 0.3 | 08.11.2025 | Realistic draft after pilot test & status review | Matthias + Claude |

**Note:** This version (Draft 0.3) reflects the actual implementation status after the successful pilot test on 03.11.2025 and Aliaksandr's detailed status report. All timelines and requirements have been adjusted to realistic, achievable values.

---

## 🎯 Executive Summary

AROC Technologies is developing **AE.HUB**, a cloud-native platform for on-demand "Human-in-the-Loop" robot control. The MVP ("Driving Dummy") will validate core functionalities: secure WebRTC remote control, task-based navigation via MQTT, and an intuitive user interface for remote operators.

**Status Quo (November 2025):**
- AROC Connector (robot side): 60-70% implemented
- Pilot test successful (03.11.2025)
- BluPassion takes over cloud platform development
- Target MVP demo: **February 28, 2026**

**Your Scope:**
You develop the cloud-native AE.HUB platform (backend + frontend), while AROC internally completes the robot connector. Clear API interfaces enable parallel development.

---

## 1. Background & Context

### 1.1 The Problem

Modern automation reaches its limits:
- Unpredictable situations (obstacles, disturbances)
- Complex decisions in dynamic environments
- High-mix-low-volume scenarios
- Expensive on-site interventions

**AE.HUB Solution:** On-demand remote operators intervene when autonomy falls short - a "Human Meta System" for robotics.

### 1.2 Technical Context

**Existing Components (AROC-side):**
- AE.01 robot hardware (mobile robot with camera and manipulation)
- ROS2 Jazzy as robot middleware
- AROC Connector (C++ software on the robot)
- DMZ infrastructure (TURN server, MQTT broker)

**Your Task (BluPassion):**
- Cloud-native backend (microservices)
- React frontend (AE.Cockpit)
- WebRTC signaling server
- Authentication & user management
- Deployment on Microsoft Azure

### 1.3 MVP Scope Definition

**The MVP does NOT include:**
- ❌ Multi-robot management
- ❌ AI-assisted task distribution (AE.Task)
- ❌ Community platform (AE.Crew)
- ❌ Advanced data analytics (AE.Data)
- ❌ Complex manipulation tasks
- ❌ Mobile apps

**The MVP focuses on:**
- ✅ **Single-Robot Control:** One operator controls one robot
- ✅ **Core Teleoperation:** Video + manual joystick control
- ✅ **Basic Navigation:** Drive to 5 predefined positions
- ✅ **Stable WebRTC:** Latency < 200ms for smooth control
- ✅ **Secure Architecture:** Authentication, TLS encryption
- ✅ **Demo-Ready:** Presentable live demonstration end of February 2026

---

## 2. Architecture & Technology Stack

### 2.1 Guiding Architecture Principles

These five principles are **mandatory**:

1. **API-First:** Every function programmatically accessible via stable, documented REST API
2. **Cloud-Native:** Designed for Microsoft Azure, containerized, scalable
3. **Microservices:** No monoliths - small, independent, loosely coupled services
4. **Security by Design:** OAuth 2.0, RBAC, encryption, audit logs, GDPR compliance
5. **Multi-Tenancy Ready:** Architecture prepared for multi-tenant operation (though MVP only 1 tenant)

### 2.2 Proposed Technology Stack

| Domain | Technology | Rationale |
|--------|------------|-----------|
| **Backend (Real-time)** | Node.js (LTS) / NestJS | Performance for I/O-intensive ops, ideal for WebRTC signaling |
| **Backend (AI/Data)** | Python 3.11+ / FastAPI | For future AI/ML modules, fast, type-safe |
| **Frontend** | React.js 18+ / TypeScript | Industry standard, large ecosystem, type safety |
| **3D Visualization** | react-three-fiber / ecctrl | Declarative 3D engine for React, digital twin |
| **Database (Relational)** | PostgreSQL 16+ | Robust, ACID-compliant, extensible |
| **Database (Time-Series)** | TimescaleDB (PG Extension) | Performance for robot telemetry |
| **Caching** | Redis | In-memory for sessions & caching |
| **Infrastructure** | Docker, Kubernetes (AKS) | Container orchestration on Azure |
| **IaC & CI/CD** | Terraform, GitHub Actions | Infrastructure as Code, automated deployments |
| **Communication (Hot)** | WebRTC | Ultra-low latency for video & teleoperation |
| **Communication (Cold)** | MQTT v5 | Scalable, reliable for asynchronous communication |

**Flexibility:** This stack is our proposal based on best practices. If you prefer alternatives for technical reasons, we're open to discussions. **Only hard requirement:** Interfaces to AROC Connector (WebRTC + MQTT) must be maintained.

### 2.3 Dual Communication Architecture

This is the **strategic core** and **non-negotiable**:

#### Hot Path - WebRTC (Real-time)
```
┌─────────────┐         ┌──────────────┐         ┌──────────────────┐
│ AE.Cockpit  │◄───────►│   Janus      │◄───────►│ AROC Connector   │
│  (Browser)  │         │   Gateway    │         │   (ROS2)         │
└─────────────┘         └──────────────┘         └──────────────────┘
       ▲                       ▲                          ▲
    Video (H.264)         WebRTC P2P              ROS2 /image_raw
    < 200ms Latency        + Data Channel          ROS2 /cmd_vel
```

**Usage:**
- Live video streaming from robot (H.264, < 200ms latency)
- Real-time joystick control (10-20 Hz, JSON via Data Channel)
- Critical for user experience

**Components:**
- **Your Signaling Server:** Coordinate WebRTC connection setup
- **Janus Gateway:** TURN/STUN server for NAT traversal (already provided by AROC)
- **AROC Connector:** Receives video, sends control commands to ROS2

#### Cold Path - MQTT (Asynchronous)
```
┌─────────────┐         ┌──────────────┐         ┌──────────────────┐
│ AE.HUB      │◄───────►│   MQTT       │◄───────►│ AROC Connector   │
│  Backend    │         │   Broker     │         │   (ROS2)         │
└─────────────┘         └──────────────┘         └──────────────────┘
       ▲                       ▲                          ▲
  Commands              Topic Structure           Nav2 Integration
  (navigateTo)          QoS, TLS                  Status Updates
```

**Usage:**
- Task commands (e.g., "Drive to Position A")
- Status updates (position, progress, errors)
- Telemetry data
- Not time-critical

**Strategic Significance:** MQTT is the technological basis of the VDA 5050 standard for AGV interoperability. This architectural decision positions AE.HUB for future fleet control.

---

## 3. Functional Requirements MVP

### 3.1 Core Function 1: Secure Connection & Teleoperation

**User Story:**  
As a remote operator, I want to securely log in and establish a live video connection with lowest latency to the robot to observe it in real-time.

**Flow:**
1. Operator opens AE.Cockpit in browser
2. Login with username/password (JWT authentication)
3. Robot selection (MVP: only "fahrdummy-01" available)
4. Click "Connect" initiates WebRTC handshake via signaling server
5. After successful P2P connection: Live video in `<video>` element
6. Latency display in UI (target: < 200ms)

**Technical Details:**
- **Frontend:** React component with WebRTC client logic
- **Backend:** Node.js signaling server (WebSocket or Socket.io)
- **Protocol:** WebRTC standard (SDP Offer/Answer, ICE Candidates)
- **Video:** H.264 encoded, provided by AROC Connector
- **Quality Target:** 1280x720 @ 25-30 FPS, < 200ms end-to-end

**Acceptance Criteria:**
- [ ] Login works with correct credentials
- [ ] WebRTC connection is successfully established
- [ ] Video is displayed smoothly (< 200ms latency measurable)
- [ ] On connection loss: automatic reconnect or clear error message
- [ ] UI shows connection status (connecting / connected / disconnected)

---

### 3.2 Core Function 2: Manual Robot Control

**User Story:**  
As a remote operator, I want to manually control the robot with a virtual joystick to position it precisely or avoid obstacles.

**Flow:**
1. After successful WebRTC connection: Joystick UI appears
2. Operator moves virtual joystick (mouse/touch)
3. Frontend converts joystick position to JSON format:
   ```json
   {
     "type": "twist",
     "linear": { "x": 0.5, "y": 0.0, "z": 0.0 },
     "angular": { "x": 0.0, "y": 0.0, "z": 0.2 },
     "timestamp": "2025-11-08T14:30:10.123Z"
   }
   ```
4. JSON is sent at 10-20 Hz via WebRTC Data Channel
5. AROC Connector receives, deserializes, publishes to ROS2 `/cmd_vel`
6. Robot moves accordingly

**Technical Details:**
- **Frontend:** Virtual joystick (e.g., nipplejs library or custom React component)
- **Data Format:** JSON corresponding to ROS2 `geometry_msgs/Twist` structure
- **Frequency:** 10-20 Hz (recommended: 20 Hz for smooth control)
- **Transport:** WebRTC Data Channel (bidirectional, low latency)
- **Feedback:** Optional - robot can send confirmation back

**Acceptance Criteria:**
- [ ] Virtual joystick is intuitive to operate
- [ ] Robot responds smoothly to joystick inputs (< 100ms reaction time noticeable)
- [ ] Joystick sends continuously during movement
- [ ] On joystick release: Robot stops immediately (0-values sent)
- [ ] Emergency stop button present and functional

---

### 3.3 Core Function 3: Task-Based Position Navigation

**User Story:**  
As a remote operator, I want to command the robot to autonomously drive to a predefined position and see progress in real-time.

**Flow:**
1. UI shows list with **5 predefined positions** (e.g., "Shelf A", "Packing Station", "Charging Station", etc.)
2. Operator clicks button "Drive to Shelf A"
3. Frontend sends MQTT command:
   ```
   Topic: aroc/robot/fahrdummy-01/commands/navigateTo
   Payload: {
     "command_id": "d290f1ee-6c54-4b01-90e6-d701748f0851",
     "timestamp": "2025-11-08T14:30:00Z",
     "target_id": "position_A",
     "priority": "normal"
   }
   ```
4. AROC Connector receives command, starts ROS2 Nav2 Action
5. During drive: Connector continuously sends status updates via MQTT:
   ```
   Topic: aroc/robot/fahrdummy-01/status/navigation
   Payload: {
     "status": "navigating",
     "target_id": "position_A",
     "progress_percent": 45,
     "current_position": { "x": 2.5, "y": 3.8, "theta": 0.75 },
     "eta_seconds": 15
   }
   ```
6. Frontend subscribes to status topic, shows progress bar + ETA
7. On arrival: Status = "arrived", success message in UI

**Technical Details:**
- **Positions:** 5 predefined coordinates (hardcoded in MVP)
- **Transport:** MQTT over TLS (Port 8883)
- **Backend:** MQTT client service subscribes to status topic, pushes updates to frontend (WebSocket)
- **Navigation:** AROC Connector uses ROS2 Nav2 Stack (autonomous, not your task)
- **Error Handling:** On error (obstacle, timeout) - Status = "error" with error code

**Acceptance Criteria:**
- [ ] List with 5 positions is displayed
- [ ] Click on position sends correct MQTT command
- [ ] Progress bar shows drive progress in real-time
- [ ] On arrival: UI shows "Arrived at Shelf A"
- [ ] On error: UI shows error message with retry option

---

### 3.4 Non-Functional Requirements

#### Performance
- **WebRTC Video Latency:** < 200ms end-to-end (measured)
- **Joystick Latency:** < 100ms reaction time (subjectively noticeable)
- **MQTT Response Time:** < 500ms for status updates
- **UI Responsiveness:** 60 FPS, no stuttering

#### Security
- **Authentication:** JWT tokens, refresh mechanism
- **Transport:** All connections over TLS (HTTPS, WSS, MQTTS)
- **Authorization:** RBAC - only authorized users may control robots
- **Secrets Management:** No credentials in code, use Azure Key Vault
- **Audit Logging:** All control operations are logged

#### Availability
- **Uptime:** 95% for MVP (no SLA, but aspired)
- **Reconnect:** Automatic on connection loss (max. 3 attempts)
- **Error Handling:** Graceful degradation - on MQTT failure, WebRTC remains functional

#### Scalability (MVP Range)
- **Concurrency:** 1 operator can control 1 robot (MVP limitation)
- **Multi-Tenant Ready:** Architecture prepared, but MVP only 1 tenant
- **Database:** Design for > 10,000 events/day

---

## 4. Interface Specification

### 4.1 MQTT Interface (Cold Path)

**Broker Details (provided by AROC):**
- Host: `mqtts://mqtt.aroc-demo.de`
- Port: 8883 (MQTT over TLS)
- Authentication: Username/Password (will be provided by AROC)
- QoS: 1 (At Least Once) for all topics

**Topic Structure:**

| Direction | Topic Pattern | Description |
|-----------|---------------|-------------|
| **Platform → Robot** | `aroc/robot/{ROBOT_ID}/commands/{COMMAND_TYPE}` | Commands to robot |
| **Robot → Platform** | `aroc/robot/{ROBOT_ID}/status/{STATUS_TYPE}` | Status updates from robot |
| **Robot → Platform** | `aroc/robot/{ROBOT_ID}/telemetry` | Sensor/telemetry data |

**Command Payloads (JSON):**

1. **navigateTo:**
   ```json
   {
     "command_id": "uuid-v4",
     "timestamp": "ISO-8601",
     "target_id": "string",
     "priority": "normal|high|emergency"
   }
   ```

2. **cancel:**
   ```json
   {
     "command_id": "uuid-v4",
     "timestamp": "ISO-8601",
     "reason": "string"
   }
   ```

**Status Payloads (JSON):**

1. **navigation:**
   ```json
   {
     "robot_id": "string",
     "timestamp": "ISO-8601",
     "status": "idle|navigating|arrived|error",
     "target_id": "string|null",
     "progress_percent": 0-100,
     "current_position": {
       "x": float,
       "y": float,
       "theta": float
     },
     "eta_seconds": int|null,
     "error_code": "string|null",
     "error_message": "string|null"
   }
   ```

### 4.2 WebRTC Interface (Hot Path)

**Signaling Server (Your Responsibility):**
- Protocol: WebSocket or Socket.io
- Endpoint: `wss://ae-hub.aroc-demo.de/signaling`
- Authentication: JWT token in header

**Signaling Flow:**
1. Client → Server: `{"type": "offer", "robot_id": "...", "sdp": "..."}`
2. Server → Robot: Forward offer
3. Robot → Server: `{"type": "answer", "robot_id": "...", "sdp": "..."}`
4. Server → Client: Forward answer
5. ICE Candidates are exchanged bidirectionally

**Data Channel (Joystick Control):**
- Channel Name: "control"
- Reliable: false (Unordered for lowest latency)
- Payload Format: JSON (see Core Function 2)
- Frequency: 10-20 Hz

**TURN/STUN Server (provided by AROC):**
- STUN: `stun:turn.aroc-demo.de:3478`
- TURN: `turn:turn.aroc-demo.de:3478`
- TURN (TLS): `turns:turn.aroc-demo.de:5349`
- Credentials: Username/Password (will be provided)

### 4.3 REST API (Status & Configuration)

**Your API Endpoints:**

1. **Authentication:**
   ```
   POST /api/v1/auth/login
   Body: { "username": "...", "password": "..." }
   Response: { "access_token": "...", "refresh_token": "...", "expires_in": 3600 }
   ```

2. **Robot List:**
   ```
   GET /api/v1/robots
   Response: [
     { "robot_id": "fahrdummy-01", "name": "AE.01 Fahrdummy", "online": true, "status": "idle" }
   ]
   ```

3. **Robot Status:**
   ```
   GET /api/v1/robots/{robot_id}/status
   Response: { ... } (see MQTT Status Payload)
   ```

4. **Position List:**
   ```
   GET /api/v1/robots/{robot_id}/positions
   Response: [
     { "position_id": "position_A", "name": "Shelf A", "coordinates": { "x": 5.0, "y": 3.0, "theta": 0.0 } },
     ...
   ]
   ```

**Documentation:** OpenAPI/Swagger specification will be created and provided by you.

---

## 5. Sprint Plan & Milestones

### Total Duration: 12 Weeks (6 Sprints of 2 weeks each)
**Start:** CW 46/2025 (November 11, 2025)  
**MVP Demo:** February 28, 2026

### Sprint 0: Kick-off & Setup (CW 46: Nov 11-15, 2025)

**Goals:**
- Joint finalization of API specification (MQTT/WebRTC)
- Set up Azure basic infrastructure (Terraform)
- Set up CI/CD pipeline for frontend & backend
- Development environments for both teams

**Deliverables:**
- [ ] Final "API Contract Document" (confirmed by both sides)
- [ ] Azure resources deployed (Resource Group, AKS Cluster, PostgreSQL, Redis)
- [ ] GitHub repos created with CI/CD
- [ ] Development access for both teams

**Effort:** 5 PD (AROC + BluPassion together)

---

### Sprint 1: Backend Foundations (CW 47-48: Nov 18-29, 2025)

**Goals:**
- Implement user authentication (JWT)
- MQTT broker connection & command handling
- Basic REST API (login, robot status)

**Deliverables:**
- [ ] Login endpoint works, JWT tokens are generated
- [ ] Backend can send MQTT commands to robot
- [ ] Backend can receive MQTT status from robot
- [ ] REST API `/api/v1/robots` and `/api/v1/robots/{id}/status` functional

**Effort:** 10 PD  
**Parallel (AROC):** Aliaksandr finalizes MQTT implementation, fixes hardcoded parameters

---

### Sprint 2: "Cold Path" Ready (CW 49-50: Dec 2-13, 2025)

**Goals:**
- Frontend: Implement login UI and static task list
- Frontend: MQTT client integration for command sending & status display
- End-to-end test: Command from UI → Backend → MQTT → Robot → Status → UI

**Deliverables:**
- [ ] Login UI works, JWT token is stored
- [ ] List with 5 positions is displayed
- [ ] Click on position → Robot drives (end-to-end works)
- [ ] Status updates are displayed in UI (progress bar)

**Effort:** 10 PD  
**Parallel (AROC):** Aliaksandr integrates Nav2, tests autonomous navigation to 5 positions

---

### Sprint 3: Backend "Hot Path" (CW 51-52: Dec 16-27, 2025)

**Goals:**
- Implementation of WebRTC signaling server (Node.js)
- Creation of basic AE.Cockpit layout
- Integration with Janus Gateway

**Deliverables:**
- [ ] Signaling server works (Offer/Answer/ICE exchange)
- [ ] Backend can communicate with Janus Gateway
- [ ] Basic UI layout for AE.Cockpit is ready (header, video area, control area)

**Effort:** 10 PD  
**Parallel (AROC):** Aliaksandr optimizes WebRTC video streaming, tests stability

---

### Sprint 4: "Hot Path" Ready (CW 1-2/2026: Jan 6-17, 2026)

**Goals:**
- Frontend: WebRTC client integration for video reception
- Frontend: Implementation of virtual joystick & Data Channel control
- End-to-end test: Live video + manual control

**Deliverables:**
- [ ] Live video is displayed in UI (< 200ms latency measured)
- [ ] Virtual joystick implemented
- [ ] Joystick data is sent via Data Channel
- [ ] Robot responds to joystick inputs

**Effort:** 10 PD  
**Parallel (AROC):** Aliaksandr finalizes Data Channel, integration with `/cmd_vel`

---

### Sprint 5: Integration & Polish (CW 3-4/2026: Jan 20-31, 2026)

**Goals:**
- End-to-end testing of all features
- Improve error handling
- UI/UX polish (loading states, error messages, transitions)
- Performance optimization

**Deliverables:**
- [ ] All 3 core functions work stably end-to-end
- [ ] Error handling implemented (reconnect, error messages)
- [ ] UI is intuitive and responsive
- [ ] Performance goals achieved (latency < 200ms)

**Effort:** 8 PD  
**Parallel (AROC):** Aliaksandr conducts stress tests, fixes bugs

---

### Sprint 6: Demo Preparation (CW 5-8/2026: Feb 3-28, 2026)

**Goals:**
- Bug fixing based on testing
- Prepare & rehearse demo scenario
- Finalize documentation (user manual, API docs, deployment guide)
- Backup plans for demo

**Deliverables:**
- [ ] All critical bugs fixed
- [ ] Demo scenario is ready (script, positions, test user)
- [ ] Documentation complete
- [ ] MVP demo-ready on Feb 28, 2026

**Effort:** 5 PD  
**Parallel (AROC):** Aliaksandr creates demo setup, hardware check

---

### Total Effort (BluPassion): ~58 PD (Person-Days)
With a team of 2-3 developers: **19-29 PD per person** over 12 weeks

---

## 6. Roles & Responsibilities

### BluPassion Team

| Role | Responsibility | Effort (PD) |
|------|----------------|-------------|
| **Full-Stack Developer 1** | Backend (Node.js), WebRTC signaling, MQTT integration | 25-30 |
| **Full-Stack Developer 2** | Frontend (React), UI/UX, WebRTC client, 3D visualization | 25-30 |
| **DevOps Engineer** | Azure infrastructure, CI/CD, monitoring, deployment | 8-10 |

**Optional:** If fewer resources available, a single senior full-stack dev with DevOps skills can cover all areas (effort then ~40-50 PD over 12 weeks).

### AROC Team

| Role | Responsibility |
|------|----------------|
| **Aliaksandr (Lead Dev)** | Complete AROC Connector, ROS2 integration, testing |
| **Matthias (CEO/PM)** | Project coordination, API finalization, stakeholder management |

---

## 7. Risks & Mitigations

| Risk | Probability | Impact | Mitigation |
|------|------------|--------|------------|
| **NAT Traversal Problems (WebRTC)** | Medium | High | TURN server mandatory (already provided by AROC) |
| **Latency > 200ms** | Low | High | Edge deployment option prepared, Janus already in Germany |
| **API Changes During Development** | Medium | Medium | API versioning strategy, early coordination |
| **Authentication Integration Delayed** | Medium | Medium | MVP workaround: Basic Auth, later JWT integration |
| **Nav2 Integration More Complex** | Low | Medium | AROC-side, already planned, fallback: show manual control |
| **Language Barrier in Coordination** | Medium | Low | Written communication, Matthias as coordinator |

---

## 8. Success Criteria & Acceptance

### Acceptance Criteria for MVP Demo (Feb 28, 2026):

**Must Criteria (Knockout):**
- [ ] Operator can log in and is authenticated
- [ ] Live video from robot is displayed (< 200ms latency)
- [ ] Operator can manually control robot with joystick
- [ ] Operator can send robot to at least 3 of 5 positions
- [ ] Status updates are displayed in real-time
- [ ] System runs stably over 30-minute demo session without crash

**Should Criteria (Nice-to-have):**
- [ ] All 5 positions work
- [ ] Latency consistently under 150ms
- [ ] UI is polished and intuitive
- [ ] Automatic reconnect on connection loss works

**Demo Scenario:**
1. Login as "demo-operator"
2. Robot selection "fahrdummy-01"
3. Establish WebRTC connection (< 5 seconds)
4. Show brief manual control (joystick)
5. Command: "Drive to Shelf A" → Robot navigates autonomously
6. During navigation: Show status updates
7. On arrival: Manually "pick" an object (simulated)
8. Command: "Drive to Packing Station" → Robot navigates back
9. Logout

**Duration:** ~10 minutes per demo run

---

## 9. Budget & Resources

### Development Costs (BluPassion Estimate)

**Assumptions:**
- Team: 2 full-stack developers + 0.5 DevOps
- Timeframe: 12 weeks
- Total effort: ~58 PD

**Rough Cost Estimate:**
- At €800/PD: ~€46,400
- At €1,000/PD: ~€58,000

**AROC Provides (no effort for BluPassion):**
- Janus Gateway (TURN/STUN)
- MQTT broker
- DMZ infrastructure
- AROC Connector (ROS2 software)
- AE.01 hardware

**BluPassion Needs (to be clarified with AROC):**
- Azure subscription (or AROC provides?)
- GitHub organization/repos
- Domain & SSL certificates
- Monitoring tools (optional: Application Insights)

---

## 10. Next Steps

### Immediately (Next 7 Days):

1. **BluPassion reads this specification** and asks questions/comments
2. **Technical coordination:** Written exchange about open API details
3. **Matthias finalizes "API Contract Document"**
4. **BluPassion creates project setup** (repos, Azure resources, CI/CD)

### Kick-off Sprint 0 (CW 46: Nov 11-15, 2025):

1. **Monday:** Joint API finalization (written or brief kick-off call)
2. **Tuesday-Friday:** Parallel work - Azure setup (BluPassion) + AROC finalizes MQTT (Aliaksandr)
3. **Friday:** Sprint 0 review - API contract approved, infra ready

### Communication:

- **Primary:** Email & shared documents (Google Docs / Markdown)
- **Weekly:** Friday status update via email (both sides)
- **Coordinator:** Matthias Heddinga for complex questions
- **Escalation:** For blockers - email to Matthias, response < 24h

---

## 11. Attachments

**Following documents will be provided additionally:**

1. **API Contract Document** (will be finalized in Sprint 0)
2. **Architecture Diagrams** (system overview, data flow, network topology)
3. **AROC Connector Status Report** (detailed status from Aliaksandr)
4. **Azure Infrastructure Requirements** (resource list, sizing, costs)

---

## 12. Contacts

### AROC Technologies GmbH

**Project Lead:**
- Matthias Heddinga (CEO)
- Email: matthias.heddinga@aroc-technologies.com
- Role: Overall responsibility, coordination, API finalization

**Lead Developer:**
- Aliaksandr Nazaruk
- Role: AROC Connector, ROS2 integration, technical questions
- **Note:** Communication primarily in writing (language barrier), Matthias mediates when needed

### BluPassion

**Contact:**
- Katja (Role TBD)
- Email: [TBD]

---

## 13. Closing Remarks

This SRS (Draft 0.3) reflects the realistic status after the successful pilot test on 03.11.2025. All information is based on facts - we don't sugarcoat, but openly name what works and what still needs to be done.

**The good news:** The architecture is solid, the core components (WebRTC video, MQTT) already work at 60-90%, and the interfaces are clearly defined. Parallel development is possible.

**The challenge:** 12 weeks is ambitious, but with focused MVP scope (only 3 core functions, 5 positions, 1 robot) and clear task division, the goal is realistic.

**We look forward to collaborating with BluPassion and are confident we'll develop a robust MVP together.**

---

**End of Software Requirements Specification**

**Version:** Draft 0.3  
**Date:** November 08, 2025  
**Status:** For review by BluPassion  
**Next Version:** After feedback from BluPassion
