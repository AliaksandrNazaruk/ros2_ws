"""
Nav2 Testing Server – Production Grade main.py

• No blocking calls in event loop
• No globals
• Explicit lifecycle
• ROS2 / MQTT / Process isolation
"""

from __future__ import annotations

import asyncio
import threading
import os
import sys
from datetime import datetime
from contextlib import asynccontextmanager
from typing import Optional, Dict, Any, List
from concurrent.futures import ThreadPoolExecutor

import rclpy
from fastapi import FastAPI, HTTPException, Request, BackgroundTasks
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import HTMLResponse, FileResponse
from fastapi.staticfiles import StaticFiles

# ---------------------------------------------------------------------
# Path bootstrap
# ---------------------------------------------------------------------
BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if BASE_DIR not in sys.path:
    sys.path.insert(0, BASE_DIR)

# Note: PositionRegistry is no longer used - positions are managed via robot_service API

# ---------------------------------------------------------------------
# Imports (project)
# ---------------------------------------------------------------------
from nav2_test_server.config import Config
from nav2_test_server.models import (
    NavigateCommand, CancelCommand, SequenceCommand,
    SystemStatus, TopicData, NodeInfo, Nav2Status, TFTransform,
    NavigationHistoryEntry, NavigationStats, TopicAnalysis, Metrics,
    MQTTStatus, NavigationStatus,
    StartNav2Request, StartBaseControllerRequest, ProcessStatusResponse
)
from nav2_test_server.ros2_monitor import ROS2Monitor
from nav2_test_server.mqtt_client import MQTTTestClient
from nav2_test_server.data_collector import DataCollector
from nav2_test_server.analysis import NavigationAnalyzer
from nav2_test_server.process_manager import ProcessManager

# ---------------------------------------------------------------------
# App Context
# ---------------------------------------------------------------------
class AppContext:
    def __init__(self, config: Config):
        self.config = config
        self.executor = ThreadPoolExecutor(max_workers=8)

        self.ros: Optional[ROS2Service] = None
        self.mqtt: Optional[MQTTService] = None
        self.process: Optional[ProcessService] = None

        self.collector: Optional[DataCollector] = None
        self.analyzer: Optional[NavigationAnalyzer] = None

    async def shutdown(self):
        if self.mqtt:
            await self.mqtt.stop()
        if self.ros:
            await self.ros.stop()
        self.executor.shutdown(wait=False)

# ---------------------------------------------------------------------
# ROS2 Service
# ---------------------------------------------------------------------
class ROS2Service:
    def __init__(self, ctx: AppContext):
        self.ctx = ctx
        self.monitor: Optional[ROS2Monitor] = None
        self.executor_ros = None
        self.thread: Optional[threading.Thread] = None
        
        # Cache for ROS2 introspection (updated in background)
        self._topics_cache: Dict[str, Any] = {}
        self._topics_ts: Optional[str] = None
        self._nodes_cache: List[Dict[str, Any]] = []
        self._nodes_ts: Optional[str] = None
        self._cache_lock = threading.Lock()
        
        # Background updater threads
        self._topics_updater_thread: Optional[threading.Thread] = None
        self._nodes_updater_thread: Optional[threading.Thread] = None
        self._updater_running = threading.Event()

    async def start(self):
        await asyncio.get_running_loop().run_in_executor(
            self.ctx.executor, self._init_blocking
        )

    def _init_blocking(self):
        import json
        import time
        # #region agent log
        try:
            with open('/home/boris/ros2_ws/.cursor/debug.log', 'a') as f:
                log_entry = json.dumps({"sessionId":"debug-session","runId":"run1","hypothesisId":"E","location":"main.py:100","message":"_init_blocking started","data":{"rclpy_ok":rclpy.ok()},"timestamp":int(time.time()*1000)}) + '\n'
                f.write(log_entry)
                f.flush()
        except Exception as ex:
            pass
        # #endregion
        if not rclpy.ok():
            rclpy.init()

        self.monitor = ROS2Monitor(
            topic_buffer_size=self.ctx.config.topic_buffer_size
        )
        # #region agent log
        try:
            with open('/home/boris/ros2_ws/.cursor/debug.log', 'a') as f:
                log_entry = json.dumps({"sessionId":"debug-session","runId":"run1","hypothesisId":"E","location":"main.py:111","message":"ROS2Monitor created","data":{"monitor_exists":self.monitor is not None},"timestamp":int(time.time()*1000)}) + '\n'
                f.write(log_entry)
                f.flush()
        except: pass
        # #endregion

        self.executor_ros = rclpy.executors.SingleThreadedExecutor()
        self.executor_ros.add_node(self.monitor)

        self.thread = threading.Thread(
            target=self._spin, daemon=True
        )
        self.thread.start()
        
        # Start background updaters for ROS2 introspection
        self._updater_running.set()
        
        self._topics_updater_thread = threading.Thread(
            target=self._topics_updater, daemon=True
        )
        self._topics_updater_thread.start()
        
        self._nodes_updater_thread = threading.Thread(
            target=self._nodes_updater, daemon=True
        )
        self._nodes_updater_thread.start()
        # #region agent log
        try:
            with open('/home/boris/ros2_ws/.cursor/debug.log', 'a') as f:
                log_entry = json.dumps({"sessionId":"debug-session","runId":"run1","hypothesisId":"E","location":"main.py:135","message":"_init_blocking completed","data":{"threads_started":True},"timestamp":int(time.time()*1000)}) + '\n'
                f.write(log_entry)
                f.flush()
        except: pass
        # #endregion

    def _spin(self):
        while rclpy.ok():
            self.executor_ros.spin_once(timeout_sec=0.1)
    
    def _topics_updater(self):
        """Background thread that updates topics cache periodically."""
        import time
        # Wait for ROS2 to initialize
        time.sleep(1.0)
        while self._updater_running.is_set() and rclpy.ok():
            try:
                if self.monitor:
                    data = self.monitor.get_all_topics()
                    with self._cache_lock:
                        self._topics_cache = data
                        self._topics_ts = datetime.utcnow().isoformat()
            except Exception as e:
                with self._cache_lock:
                    self._topics_cache = {
                        "__error__": str(e),
                        "timestamp": datetime.utcnow().isoformat()
                    }
            time.sleep(2.0)  # Update every 2 seconds
    
    def _nodes_updater(self):
        """
        Background thread that updates nodes cache periodically.
        
        Updates the nodes cache every 2 seconds by querying ROS2 node graph.
        Errors are logged and cached to prevent cache corruption.
        """
        import time
        # Wait for ROS2 to initialize
        time.sleep(1.0)
        while self._updater_running.is_set() and rclpy.ok():
            try:
                if self.monitor:
                    data = self.monitor.get_nodes()
                    with self._cache_lock:
                        self._nodes_cache = data
                        self._nodes_ts = datetime.utcnow().isoformat()
            except Exception as e:
                self.get_logger().error(f"Error in _nodes_updater: {e}")
                with self._cache_lock:
                    self._nodes_cache = [{
                        "__error__": str(e),
                        "timestamp": datetime.utcnow().isoformat()
                    }]
            time.sleep(2.0)  # Update every 2 seconds

    async def stop(self):
        if self.monitor:
            # Stop background updaters
            self._updater_running.clear()
            
            # Wait a bit for threads to finish current iteration
            if self._topics_updater_thread:
                self._topics_updater_thread.join(timeout=1.0)
            if self._nodes_updater_thread:
                self._nodes_updater_thread.join(timeout=1.0)
            
            await asyncio.get_running_loop().run_in_executor(
                self.ctx.executor, self._shutdown_blocking
            )

    def _shutdown_blocking(self):
        self.monitor.stop()
        self.monitor.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    # ---- exposed async API ----
    async def get_nodes(self):
        """Returns cached nodes data (updated in background)."""
        with self._cache_lock:
            return self._nodes_cache.copy() if self._nodes_cache else []

    async def get_all_topics(self):
        """Returns cached topics data (updated in background)."""
        with self._cache_lock:
            return self._topics_cache.copy() if self._topics_cache else {}

    async def get_nav2_status(self):
        return await asyncio.get_running_loop().run_in_executor(
            self.ctx.executor, self.monitor.get_nav2_status
        )

    async def get_tf(self, parent: str, child: str):
        return await asyncio.get_running_loop().run_in_executor(
            self.ctx.executor,
            self.monitor.get_tf_transform,
            parent,
            child
        )

# ---------------------------------------------------------------------
# MQTT Service
# ---------------------------------------------------------------------
class MQTTService:
    def __init__(self, ctx: AppContext):
        self.ctx = ctx
        self.client: Optional[MQTTTestClient] = None

    async def start(self):
        await asyncio.get_running_loop().run_in_executor(
            self.ctx.executor, self._connect_blocking
        )

    def _connect_blocking(self):
        self.client = MQTTTestClient(self.ctx.config)
        self.client.on_status_callback = self._on_status
        self.client.connect()

    def _on_status(self, status: Dict[str, Any]):
        if not self.ctx.collector:
            return
        try:
            self.ctx.collector.update_navigation_status(
                command_id=status.get("command_id"),
                status=NavigationStatus(status.get("status")),
                error_code=status.get("error_code"),
                error_message=status.get("error_message")
            )
        except Exception:
            pass

    async def stop(self):
        if self.client:
            await asyncio.get_running_loop().run_in_executor(
                self.ctx.executor, self.client.disconnect
            )

# ---------------------------------------------------------------------
# Process Service
# ---------------------------------------------------------------------
class ProcessService:
    def __init__(self, ctx: AppContext):
        self.ctx = ctx
        self.manager = ProcessManager(
            workspace_path=os.path.dirname(BASE_DIR)
        )

    async def start_nav2(self, req: StartNav2Request):
        return await asyncio.get_running_loop().run_in_executor(
            self.ctx.executor,
            self.manager.start_nav2,
            req.map_file,
            req.symovo_endpoint,
            req.amr_id,
            req.use_scan_converter,
            req.config_service_api_key,
            req.robot_id,
            req.config_service_url
        )

    async def stop_nav2(self):
        return await asyncio.get_running_loop().run_in_executor(
            self.ctx.executor, self.manager.stop_nav2
        )

    async def status(self, name: str):
        return await asyncio.get_running_loop().run_in_executor(
            self.ctx.executor,
            self.manager.get_status,
            name
        )

# ---------------------------------------------------------------------
# FastAPI lifespan
# ---------------------------------------------------------------------
config = Config()

@asynccontextmanager
async def lifespan(app: FastAPI):
    ctx = AppContext(config)
    app.state.ctx = ctx

    ctx.collector = DataCollector(
        history_size=config.history_size,
        topic_buffer_size=config.topic_buffer_size
    )
    ctx.analyzer = NavigationAnalyzer(ctx.collector, None)

    # Note: Position management is handled directly by robot_service API
    # No need for position client in this server

    ctx.ros = ROS2Service(ctx)
    ctx.mqtt = MQTTService(ctx)
    ctx.process = ProcessService(ctx)

    await asyncio.gather(
        ctx.ros.start(),
        ctx.mqtt.start()
    )

    yield

    await ctx.shutdown()

# ---------------------------------------------------------------------
# FastAPI app
# ---------------------------------------------------------------------
app = FastAPI(
    title="Nav2 Testing Server",
    version="2.0.0",
    lifespan=lifespan
)

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_methods=["*"],
    allow_headers=["*"],
    allow_credentials=True,
)

# ---------------------------------------------------------------------
# Static
# ---------------------------------------------------------------------
STATIC_DIR = os.path.join(os.path.dirname(__file__), "static")
if os.path.exists(STATIC_DIR):
    app.mount("/static", StaticFiles(directory=STATIC_DIR), name="static")

# ---------------------------------------------------------------------
# Routes
# ---------------------------------------------------------------------
@app.get("/api/monitor/health")
async def health(request: Request):
    ctx: AppContext = request.app.state.ctx
    return {
        "status": "ok",
        "ros2": rclpy.ok(),
        "mqtt": bool(ctx.mqtt and ctx.mqtt.client and ctx.mqtt.client.is_connected),
        "timestamp": datetime.utcnow().isoformat()
    }

@app.get("/api/monitor/nodes", response_model=List[NodeInfo])
async def nodes(request: Request):
    """
    Get list of all ROS2 nodes with their publishers, subscribers, services, and actions.
    
    Returns cached node information updated in background thread.
    """
    ctx: AppContext = request.app.state.ctx
    raw = await ctx.ros.get_nodes()
    return [
        NodeInfo(
            name=n["name"],
            namespace=n.get("namespace", ""),
            publishers=n.get("publishers", []),
            subscribers=n.get("subscribers", []),
            services=n.get("services", []),
            actions=n.get("actions", [])
        ) for n in raw
    ]

@app.get("/api/monitor/topics", response_model=Dict[str, TopicData])
async def topics(request: Request):
    ctx: AppContext = request.app.state.ctx
    data = await ctx.ros.get_all_topics()
    return {
        k: TopicData(
            name=k,
            type=v.get("type", "unknown"),
            last_message_time=v.get("last_message_time"),
            message_count=v.get("message_count", 0),
            frequency_hz=v.get("frequency_hz"),
            data=v.get("data")
        ) for k, v in data.items()
    }

@app.get("/api/process/nav2/status", response_model=ProcessStatusResponse)
async def nav2_status(request: Request):
    ctx: AppContext = request.app.state.ctx
    return ProcessStatusResponse(**await ctx.process.status("nav2"))

@app.post("/api/process/nav2/activate_lifecycle")
async def activate_nav2_lifecycle(request: Request):
    """
    Manually activate Nav2 lifecycle nodes
    
    This endpoint can be used to activate lifecycle nodes if autostart fails.
    """
    ctx: AppContext = request.app.state.ctx
    results = await asyncio.get_running_loop().run_in_executor(
        ctx.executor,
        ctx.process.manager.activate_nav2_lifecycle_nodes
    )
    return {
        "success": all(r.get('success', False) for r in results.values()),
        "results": results
    }

# ---------------------------------------------------------------------
# Entry
# ---------------------------------------------------------------------
@app.get("/")
async def dashboard():
    return FileResponse("static/index.html")

@app.get("/monitoring")
async def monitoring():
    return FileResponse("static/monitoring.html")
@app.get("/api/monitor/status")
async def legacy_status(request: Request):
    ctx = request.app.state.ctx
    return {
        "ros2": rclpy.ok(),
        "mqtt": bool(ctx.mqtt and ctx.mqtt.client and ctx.mqtt.client.is_connected),
        "timestamp": datetime.utcnow().isoformat()
    }

# ---------------------------------------------------------------------
# Position Management API
# ---------------------------------------------------------------------
# Note: Position management endpoints have been removed.
# Positions are managed directly through robot_service API:
# - GET /api/v1/robot/robot_positions/list
# - POST /api/v1/robot/robot_positions/save
# - POST /api/v1/robot/robot_positions/delete?position_id=...
# - POST /api/v1/robot/robot_positions/run?position_id=...
#
# Use robot_service API directly instead of proxying through this server.

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(
        "main:app",
        host=config.server_host,
        port=config.server_port,
        proxy_headers=True,
        forwarded_allow_ips="*",
        timeout_keep_alive=30,
        workers=1
    )
