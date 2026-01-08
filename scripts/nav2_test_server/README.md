# Nav2 Testing Server

FastAPI server for monitoring, testing, and analyzing Nav2 and navigation node.

## Features

- **ROS2 Monitoring**: Real-time monitoring of ROS2 topics, nodes, and Nav2 status
- **MQTT Integration**: Send navigation commands and receive status updates via MQTT
- **Data Collection**: Collect and store navigation history and metrics
- **Analysis**: Analyze navigation performance and system metrics
- **Web Dashboard**: Interactive web interface for visualization

## Installation

1. Install dependencies:
```bash
cd /home/boris/ros2_ws/scripts
pip install -r requirements_test_server.txt
```

2. Configure settings (optional):
Create a `.env` file in `nav2_test_server/` directory:
```bash
MQTT_BROKER=82.165.177.194
MQTT_PORT=8883
MQTT_USERNAME=bridge_user
MQTT_PASSWORD=your_password
ROBOT_ID=robot_001
CONFIG_SERVICE_URL=http://localhost:7900
CONFIG_SERVICE_API_KEY=your_api_key
```

## Usage

### Start the server:

```bash
cd /home/boris/ros2_ws
./scripts/start_test_server.sh
```

Or manually:
```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
cd scripts/nav2_test_server
uvicorn main:app --host 0.0.0.0 --port 8000 --reload
```

### Access the dashboard:

- Dashboard: http://localhost:8000/dashboard
- API Documentation: http://localhost:8000/docs
- API Root: http://localhost:8000/

## API Endpoints

### Monitoring

- `GET /api/monitor/status` - System status
- `GET /api/monitor/topics` - All monitored topics
- `GET /api/monitor/topics/{topic_name}` - Specific topic data
- `GET /api/monitor/nodes` - Active ROS2 nodes
- `GET /api/monitor/nav2` - Nav2 action server status
- `GET /api/monitor/tf` - TF transforms
- `GET /api/monitor/health` - Health check

### Testing

- `POST /api/test/navigate` - Send navigation command
- `POST /api/test/cancel` - Cancel navigation
- `GET /api/test/history` - Command history
- `POST /api/test/sequence` - Send sequence of commands

### Analysis

- `GET /api/analysis/stats` - Navigation statistics
- `GET /api/analysis/topics` - Topic analysis
- `GET /api/analysis/nav2` - Nav2 analysis
- `GET /api/analysis/metrics` - System metrics
- `GET /api/analysis/report` - Full analysis report

### MQTT

- `GET /api/mqtt/status` - MQTT connection status
- `GET /api/mqtt/last_status` - Last navigation status from MQTT

### Process Management

- `POST /api/process/nav2/start` - Start Nav2 and base_controller
- `POST /api/process/nav2/stop` - Stop Nav2 and base_controller
- `POST /api/process/nav2/restart` - Restart Nav2 and base_controller
- `GET /api/process/nav2/status` - Get Nav2 process status
- `POST /api/process/base_controller/start` - Start base_controller only
- `POST /api/process/base_controller/stop` - Stop base_controller
- `POST /api/process/base_controller/restart` - Restart base_controller
- `GET /api/process/base_controller/status` - Get base_controller process status
- `GET /api/process/status` - Get status of all managed processes

## Examples

### Send navigation command:

```bash
curl -X POST http://localhost:8000/api/test/navigate \
  -H "Content-Type: application/json" \
  -d '{"target_id": "position_A", "priority": "normal"}'
```

### Get system status:

```bash
curl http://localhost:8000/api/monitor/status
```

### Get navigation statistics:

```bash
curl http://localhost:8000/api/analysis/stats
```

## Architecture

- **ROS2Monitor**: Monitors ROS2 topics, nodes, and Nav2 status
- **MQTTTestClient**: Sends commands and receives status via MQTT
- **DataCollector**: Collects and stores navigation data
- **NavigationAnalyzer**: Analyzes data and provides statistics
- **FastAPI App**: REST API and web dashboard

## Requirements

- ROS2 Jazzy
- Python 3.12+
- FastAPI, Uvicorn
- rclpy, paho-mqtt, requests

## Notes

- The server requires ROS2 to be initialized (rclpy.init())
- MQTT configuration can be fetched from Config Service or set directly
- Dashboard updates automatically every 2-5 seconds
- All navigation commands are logged in history

