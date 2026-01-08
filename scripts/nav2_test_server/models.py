"""
Pydantic models for API requests and responses
"""

from typing import Optional, List, Dict, Any
from datetime import datetime
from pydantic import BaseModel, Field
from enum import Enum


class NavigationStatus(str, Enum):
    """Navigation status enum"""
    IDLE = "idle"
    NAVIGATING = "navigating"
    ARRIVED = "arrived"
    ERROR = "error"


class Priority(str, Enum):
    """Command priority enum"""
    NORMAL = "normal"
    HIGH = "high"
    EMERGENCY = "emergency"


# Request Models
class NavigateCommand(BaseModel):
    """Navigate command request"""
    target_id: str = Field(..., description="Target position ID (position_A, position_B, etc.)")
    priority: Priority = Field(default=Priority.NORMAL, description="Command priority")


class CancelCommand(BaseModel):
    """Cancel command request"""
    reason: Optional[str] = Field(default=None, description="Cancel reason")


class SequenceCommand(BaseModel):
    """Sequence of navigation commands"""
    commands: List[NavigateCommand] = Field(..., description="List of navigation commands")


# Response Models
class TopicData(BaseModel):
    """Topic data response"""
    name: str
    type: str
    last_message_time: Optional[datetime] = None
    message_count: int = 0
    frequency_hz: Optional[float] = None
    data: Optional[Dict[str, Any]] = None


class NodeInfo(BaseModel):
    """Node information"""
    name: str
    namespace: str
    publishers: List[str] = []
    subscribers: List[str] = []
    services: List[str] = []
    actions: List[str] = []


class Nav2Status(BaseModel):
    """Nav2 status"""
    action_server_available: bool
    action_server_name: str = "navigate_to_pose"
    lifecycle_state: Optional[str] = None
    active_goal: bool = False
    goal_id: Optional[str] = None


class TFTransform(BaseModel):
    """TF transform"""
    parent_frame: str
    child_frame: str
    translation: Dict[str, float]
    rotation: Dict[str, float]
    available: bool


class SystemStatus(BaseModel):
    """Overall system status"""
    ros2_connected: bool
    mqtt_connected: bool
    nav2_available: bool
    base_controller_running: bool
    navigation_node_running: bool
    timestamp: datetime = Field(default_factory=datetime.now)


class NavigationHistoryEntry(BaseModel):
    """Navigation history entry"""
    command_id: str
    target_id: str
    priority: str
    timestamp: datetime
    status: NavigationStatus
    result: Optional[str] = None
    error_code: Optional[str] = None
    error_message: Optional[str] = None
    duration_seconds: Optional[float] = None


class NavigationStats(BaseModel):
    """Navigation statistics"""
    total_commands: int
    successful: int
    failed: int
    cancelled: int
    average_duration_seconds: Optional[float] = None
    success_rate: float
    error_breakdown: Dict[str, int] = {}


class TopicAnalysis(BaseModel):
    """Topic analysis"""
    name: str
    frequency_hz: float
    message_count: int
    last_message_age_seconds: float
    is_active: bool


class Metrics(BaseModel):
    """System metrics"""
    topics: List[TopicAnalysis]
    navigation_stats: NavigationStats
    system_uptime_seconds: float
    timestamp: datetime = Field(default_factory=datetime.now)


class MQTTStatus(BaseModel):
    """MQTT connection status"""
    connected: bool
    broker: Optional[str] = None
    port: Optional[int] = None
    last_status: Optional[Dict[str, Any]] = None
    last_status_time: Optional[datetime] = None


# Process Management Models
class StartNav2Request(BaseModel):
    """Request to start Nav2"""
    map_file: Optional[str] = Field(default=None, description="Path to map YAML file")
    symovo_endpoint: str = Field(default="https://192.168.1.100", description="Symovo API endpoint")
    amr_id: int = Field(default=15, description="AMR ID")
    use_scan_converter: bool = Field(default=True, description="Enable scan converter")
    config_service_api_key: Optional[str] = Field(default=None, description="API key for Config Service (required for navigation_integrated_node)")
    robot_id: str = Field(default="robot_001", description="Robot ID for MQTT topics")
    config_service_url: str = Field(default="http://localhost:7900", description="URL of Config Service")


class StartBaseControllerRequest(BaseModel):
    """Request to start base_controller"""
    driver_endpoint: str = Field(default="https://192.168.1.100", description="Symovo API endpoint")
    amr_id: int = Field(default=15, description="AMR ID")
    tls_verify: bool = Field(default=False, description="Enable TLS verification")


class ProcessStatusResponse(BaseModel):
    """Process status response"""
    name: str
    status: str
    running: bool
    pid: Optional[int] = None
    logs: List[str] = []


# Position Management Models
class PositionData(BaseModel):
    """Position data model with validation"""
    x: float = Field(..., description="X coordinate in meters", ge=-1000.0, le=1000.0)
    y: float = Field(..., description="Y coordinate in meters", ge=-1000.0, le=1000.0)
    theta: float = Field(..., description="Orientation angle in radians", ge=-3.14159, le=3.14159)
    description: str = Field(default="", description="Position description", max_length=256)


class CreatePositionRequest(BaseModel):
    """Request to create or update a position"""
    position_id: str = Field(..., description="Position ID (e.g., 'position_A')", min_length=1, max_length=64, pattern="^[a-zA-Z0-9_]+$")
    x: float = Field(..., description="X coordinate in meters", ge=-1000.0, le=1000.0)
    y: float = Field(..., description="Y coordinate in meters", ge=-1000.0, le=1000.0)
    theta: float = Field(..., description="Orientation angle in radians", ge=-3.14159, le=3.14159)
    description: str = Field(default="", description="Position description", max_length=256)


class PositionResponse(BaseModel):
    """Position response model"""
    position_id: str
    x: float
    y: float
    theta: float
    description: str


class PositionListResponse(BaseModel):
    """List of positions response"""
    positions: List[PositionResponse]
    count: int


class PositionDeleteResponse(BaseModel):
    """Position deletion response"""
    success: bool
    message: str
    position_id: Optional[str] = None

