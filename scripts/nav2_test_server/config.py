"""
Configuration for Nav2 Testing Server
"""

from typing import Optional
try:
    from pydantic_settings import BaseSettings
except ImportError:
    # Fallback for older pydantic
    from pydantic import BaseSettings


class Config(BaseSettings):
    """Configuration settings for the testing server"""
    
    # ROS2
    ros2_namespace: str = ""
    
    # MQTT (can be overridden by Config Service)
    mqtt_broker: Optional[str] = None
    mqtt_port: int = 8883
    mqtt_username: Optional[str] = None
    mqtt_password: Optional[str] = None
    mqtt_use_tls: bool = True
    mqtt_tls_insecure: bool = True
    robot_id: str = "robot_001"
    
    # Config Service (for fetching MQTT configuration)
    config_service_url: Optional[str] = None
    config_service_api_key: Optional[str] = None
    
    # Data collection
    history_size: int = 1000
    topic_buffer_size: int = 100
    
    # Server
    server_host: str = "0.0.0.0"
    server_port: int = 8000
    
    # Robot Service API (for position management)
    robot_service_url: str = "http://localhost:8110"
    robot_service_timeout: float = 10.0
    
    class Config:
        env_file = ".env"
        env_file_encoding = "utf-8"

