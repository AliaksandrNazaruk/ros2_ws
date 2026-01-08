"""
Process Manager

Manages ROS2 launch processes for Nav2 and base_controller.
"""

import subprocess
import os
import signal
import threading
import time
import psutil
from typing import Optional, Dict, Any
from datetime import datetime
from enum import Enum


class ProcessStatus(str, Enum):
    """Process status enum"""
    STOPPED = "stopped"
    STARTING = "starting"
    RUNNING = "running"
    STOPPING = "stopping"
    ERROR = "error"


class ProcessManager:
    """Manages ROS2 launch processes"""
    
    def __init__(self, workspace_path: str = "/home/boris/ros2_ws"):
        self.workspace_path = workspace_path
        self.processes: Dict[str, subprocess.Popen] = {}
        self.process_status: Dict[str, ProcessStatus] = {}
        self.process_logs: Dict[str, list] = {}
        self.lock = threading.Lock()
        
        # Initialize status
        self.process_status['nav2'] = ProcessStatus.STOPPED
        self.process_status['base_controller'] = ProcessStatus.STOPPED
    
    def _get_ros2_env(self) -> Dict[str, str]:
        """Get ROS2 environment variables"""
        env = os.environ.copy()
        
        # Source ROS2
        try:
            result = subprocess.run(
                ['bash', '-c', 'source /opt/ros/jazzy/setup.bash && env'],
                capture_output=True,
                text=True,
                timeout=5
            )
            for line in result.stdout.split('\n'):
                if '=' in line and any(key in line for key in ['ROS_', 'AMENT_', 'PYTHON']):
                    key, value = line.split('=', 1)
                    env[key] = value
        except Exception:
            pass
        
        # Source workspace
        try:
            result = subprocess.run(
                ['bash', '-c', f'source {self.workspace_path}/install/setup.bash && env'],
                capture_output=True,
                text=True,
                timeout=5
            )
            for line in result.stdout.split('\n'):
                if '=' in line and any(key in line for key in ['ROS_', 'AMENT_', 'PYTHON']):
                    key, value = line.split('=', 1)
                    env[key] = value
        except Exception:
            pass
        
        return env
    
    def start_nav2(self, map_file: Optional[str] = None, 
                   symovo_endpoint: str = "https://192.168.1.100",
                   amr_id: int = 15,
                   use_scan_converter: bool = True,
                   config_service_api_key: Optional[str] = None,
                   robot_id: str = "robot_001",
                   config_service_url: str = "http://localhost:7900") -> bool:
        """
        Start Nav2 and base_controller using symovo_nav2.launch.py
        
        Args:
            map_file: Path to map YAML file (optional)
            symovo_endpoint: Symovo API endpoint
            amr_id: AMR ID
            use_scan_converter: Enable scan converter
            config_service_api_key: API key for Config Service (required for navigation_integrated_node)
            robot_id: Robot ID for MQTT topics (default: robot_001)
            config_service_url: URL of Config Service (default: http://localhost:7900)
        
        Returns:
            True if started successfully, False otherwise
        """
        with self.lock:
            if 'nav2' in self.processes and self.processes['nav2'].poll() is None:
                return False  # Already running
            
            self.process_status['nav2'] = ProcessStatus.STARTING
            self.process_logs['nav2'] = []
        
        try:
            # Build launch command
            # Try to find launch file in workspace or use package name
            launch_file = os.path.join(self.workspace_path, 'launch', 'symovo_nav2.launch.py')
            if os.path.exists(launch_file):
                # Use full path
                cmd = ['ros2', 'launch', launch_file]
            else:
                # Try to use package (if installed)
                cmd = ['ros2', 'launch', 'symovo_nav2', 'symovo_nav2.launch.py']
            
            cmd.extend(['symovo_endpoint:=' + symovo_endpoint])
            cmd.extend(['amr_id:=' + str(amr_id)])
            cmd.extend(['use_scan_converter:=' + str(use_scan_converter).lower()])
            cmd.extend(['robot_id:=' + robot_id])
            cmd.extend(['config_service_url:=' + config_service_url])
            
            if map_file:
                cmd.extend(['map_file:=' + map_file])
            
            # Add config_service_api_key if provided (required for navigation_integrated_node)
            if config_service_api_key:
                cmd.extend(['config_service_api_key:=' + config_service_api_key])
            
            # Start process
            env = self._get_ros2_env()
            # Create new process group for better control
            process = subprocess.Popen(
                cmd,
                cwd=self.workspace_path,
                env=env,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
                preexec_fn=os.setsid if os.name != 'nt' else None  # Create new process group
            )
            
            with self.lock:
                self.processes['nav2'] = process
                self.process_status['nav2'] = ProcessStatus.RUNNING
            
            # Start log reader thread
            threading.Thread(
                target=self._read_process_logs,
                args=('nav2', process),
                daemon=True
            ).start()
            
            return True
            
        except Exception as e:
            with self.lock:
                self.process_status['nav2'] = ProcessStatus.ERROR
                self.process_logs['nav2'] = [f"Error starting Nav2: {str(e)}"]
            return False
    
    def start_base_controller(self, driver_endpoint: str = "https://192.168.1.100",
                              amr_id: int = 15,
                              tls_verify: bool = False) -> bool:
        """
        Start base_controller node only
        
        Args:
            driver_endpoint: Symovo API endpoint
            amr_id: AMR ID
            tls_verify: Enable TLS verification
        
        Returns:
            True if started successfully, False otherwise
        """
        with self.lock:
            if 'base_controller' in self.processes and self.processes['base_controller'].poll() is None:
                return False  # Already running
            
            self.process_status['base_controller'] = ProcessStatus.STARTING
            self.process_logs['base_controller'] = []
        
        try:
            cmd = ['ros2', 'run', 'base_controller', 'base_controller_node']
            cmd.extend(['--ros-args'])
            cmd.extend(['-p', f'driver_endpoint:={driver_endpoint}'])
            cmd.extend(['-p', f'amr_id:={amr_id}'])
            cmd.extend(['-p', f'tls_verify:={str(tls_verify).lower()}'])
            
            env = self._get_ros2_env()
            # Create new process group for better control
            process = subprocess.Popen(
                cmd,
                cwd=self.workspace_path,
                env=env,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
                preexec_fn=os.setsid if os.name != 'nt' else None  # Create new process group
            )
            
            with self.lock:
                self.processes['base_controller'] = process
                self.process_status['base_controller'] = ProcessStatus.RUNNING
            
            # Start log reader thread
            threading.Thread(
                target=self._read_process_logs,
                args=('base_controller', process),
                daemon=True
            ).start()
            
            return True
            
        except Exception as e:
            with self.lock:
                self.process_status['base_controller'] = ProcessStatus.ERROR
                self.process_logs['base_controller'] = [f"Error starting base_controller: {str(e)}"]
            return False
    
    def stop_nav2(self) -> bool:
        """Stop Nav2 and base_controller process"""
        return self._stop_process('nav2')
    
    def stop_base_controller(self) -> bool:
        """Stop base_controller process"""
        return self._stop_process('base_controller')
    
    def _stop_process(self, process_name: str) -> bool:
        """Stop a process"""
        with self.lock:
            if process_name not in self.processes:
                # Check if process is actually running by PID
                # This handles case where process was started but not tracked
                return True
            
            process = self.processes.get(process_name)
            if process is None:
                self.process_status[process_name] = ProcessStatus.STOPPED
                if process_name in self.processes:
                    del self.processes[process_name]
                return True
            
            # Check if already stopped
            if process.poll() is not None:
                self.process_status[process_name] = ProcessStatus.STOPPED
                del self.processes[process_name]
                return True
            
            self.process_status[process_name] = ProcessStatus.STOPPING
            pid = process.pid
        
        try:
            # For launch processes, we need to kill the entire process tree
            # First try graceful termination
            try:
                # Try to kill process group (more effective for launch files)
                if os.name != 'nt':
                    try:
                        os.killpg(os.getpgid(pid), signal.SIGTERM)
                    except (OSError, ProcessLookupError):
                        process.terminate()
                else:
                    process.terminate()
            except (OSError, ProcessLookupError):
                process.terminate()
            
            # Wait for graceful shutdown
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                # Force kill if not terminated
                try:
                    if os.name != 'nt':
                        os.killpg(os.getpgid(pid), signal.SIGKILL)
                    else:
                        process.kill()
                    process.wait(timeout=2)
                except (OSError, ProcessLookupError, subprocess.TimeoutExpired):
                    pass
            
            # Also kill child processes (for launch files)
            if process_name == 'nav2':
                # Kill any remaining ros2 launch processes and their children
                try:
                    # Use psutil to find and kill process tree
                    parent = psutil.Process(pid)
                    children = parent.children(recursive=True)
                    for child in children:
                        try:
                            child.terminate()
                        except (psutil.NoSuchProcess, psutil.AccessDenied):
                            pass
                    # Wait a bit
                    time.sleep(0.5)
                    # Force kill remaining
                    for child in children:
                        try:
                            child.kill()
                        except (psutil.NoSuchProcess, psutil.AccessDenied):
                            pass
                except (psutil.NoSuchProcess, psutil.AccessDenied, Exception):
                    pass
                
                # Kill all launch-related processes
                try:
                    subprocess.run(['pkill', '-f', 'symovo_nav2.launch.py'], timeout=2)
                    time.sleep(0.5)
                    # Kill all base_controller and scan_converter from launch
                    subprocess.run(['pkill', '-f', 'base_controller_node.*__node:=base_controller'], timeout=2)
                    subprocess.run(['pkill', '-f', 'symovo_scan_converter.*__node:=symovo_scan_converter'], timeout=2)
                except Exception:
                    pass
            
            # For base_controller, also kill any orphaned processes
            if process_name == 'base_controller':
                try:
                    # Kill any base_controller_node processes that might be orphaned
                    # But only those started via API (with driver_endpoint parameter)
                    subprocess.run(['pkill', '-f', 'base_controller_node.*driver_endpoint'], timeout=2)
                except Exception:
                    pass
            
            with self.lock:
                self.process_status[process_name] = ProcessStatus.STOPPED
                if process_name in self.processes:
                    del self.processes[process_name]
            
            return True
            
        except Exception as e:
            with self.lock:
                self.process_status[process_name] = ProcessStatus.ERROR
                self.process_logs[process_name] = self.process_logs.get(process_name, []) + [f"Error stopping: {str(e)}"]
            return False
    
    def restart_nav2(self, **kwargs) -> bool:
        """Restart Nav2"""
        self.stop_nav2()
        time.sleep(1)
        return self.start_nav2(**kwargs)
    
    def restart_base_controller(self, **kwargs) -> bool:
        """Restart base_controller"""
        self.stop_base_controller()
        time.sleep(1)
        return self.start_base_controller(**kwargs)
    
    def _read_process_logs(self, process_name: str, process: subprocess.Popen):
        """Read process logs in background"""
        try:
            while True:
                line = process.stdout.readline()
                if not line:
                    if process.poll() is not None:
                        break
                    continue
                
                with self.lock:
                    if process_name not in self.process_logs:
                        self.process_logs[process_name] = []
                    self.process_logs[process_name].append(line.strip())
                    # Keep only last 100 lines
                    if len(self.process_logs[process_name]) > 100:
                        self.process_logs[process_name] = self.process_logs[process_name][-100:]
        except Exception as e:
            # Log error but don't crash
            with self.lock:
                if process_name not in self.process_logs:
                    self.process_logs[process_name] = []
                self.process_logs[process_name].append(f"Log reader error: {str(e)}")
    
    def get_status(self, process_name: str) -> Dict[str, Any]:
        """Get process status"""
        with self.lock:
            status = self.process_status.get(process_name, ProcessStatus.STOPPED)
            process = self.processes.get(process_name)
            
            is_running = False
            pid = None
            if process and process.poll() is None:
                is_running = True
                pid = process.pid
            
            return {
                'name': process_name,
                'status': status.value,
                'running': is_running,
                'pid': pid,
                'logs': self.process_logs.get(process_name, [])[-20:]  # Last 20 lines
            }
    
    def get_all_status(self) -> Dict[str, Dict[str, Any]]:
        """Get status of all processes"""
        return {
            'nav2': self.get_status('nav2'),
            'base_controller': self.get_status('base_controller')
        }
    
    def cleanup_orphaned_processes(self) -> Dict[str, int]:
        """Clean up orphaned base_controller and scan_converter processes"""
        cleaned = {'base_controller': 0, 'symovo_scan_converter': 0}
        
        try:
            # Get tracked PIDs first (with lock, but quickly)
            tracked_pids = set()
            with self.lock:
                for proc in self.processes.values():
                    if proc and proc.poll() is None:
                        try:
                            tracked_pids.add(proc.pid)
                        except Exception:
                            pass
            
            # Find all base_controller processes (with timeout)
            try:
                result = subprocess.run(
                    ['pgrep', '-f', 'base_controller_node'],
                    capture_output=True,
                    text=True,
                    timeout=1
                )
                if result.returncode == 0 and result.stdout.strip():
                    pids = []
                    for pid_str in result.stdout.strip().split('\n'):
                        if pid_str.strip():
                            try:
                                pid = int(pid_str.strip())
                                if pid not in tracked_pids:
                                    pids.append(pid)
                            except ValueError:
                                continue
                    
                    # Kill processes (with individual timeouts)
                    for pid in pids:
                        try:
                            proc = psutil.Process(pid)
                            proc.terminate()
                            # Don't sleep for each process - use timeout
                            cleaned['base_controller'] += 1
                        except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.TimeoutExpired):
                            pass
                        except Exception:
                            pass
                    
                    # Wait a bit and force kill remaining
                    if pids:
                        time.sleep(0.5)
                        for pid in pids:
                            try:
                                proc = psutil.Process(pid)
                                if proc.is_running():
                                    proc.kill()
                            except (psutil.NoSuchProcess, psutil.AccessDenied):
                                pass
                            except Exception:
                                pass
            except subprocess.TimeoutExpired:
                pass
            except Exception as e:
                # Log but don't fail
                pass
            
            # Find all symovo_scan_converter processes (with timeout)
            try:
                result = subprocess.run(
                    ['pgrep', '-f', 'symovo_scan_converter'],
                    capture_output=True,
                    text=True,
                    timeout=1
                )
                if result.returncode == 0 and result.stdout.strip():
                    pids = []
                    for pid_str in result.stdout.strip().split('\n'):
                        if pid_str.strip():
                            try:
                                pids.append(int(pid_str.strip()))
                            except ValueError:
                                continue
                    
                    # Kill all scan converters (they should only run from launch)
                    for pid in pids:
                        try:
                            proc = psutil.Process(pid)
                            proc.terminate()
                            cleaned['symovo_scan_converter'] += 1
                        except (psutil.NoSuchProcess, psutil.AccessDenied):
                            pass
                        except Exception:
                            pass
                    
                    # Wait and force kill
                    if pids:
                        time.sleep(0.5)
                        for pid in pids:
                            try:
                                proc = psutil.Process(pid)
                                if proc.is_running():
                                    proc.kill()
                            except (psutil.NoSuchProcess, psutil.AccessDenied):
                                pass
                            except Exception:
                                pass
            except subprocess.TimeoutExpired:
                pass
            except Exception as e:
                # Log but don't fail
                pass
                
        except Exception as e:
            # Don't let cleanup crash the server
            pass
        
        return cleaned
    
    def activate_nav2_lifecycle_nodes(self) -> Dict[str, Any]:
        """
        Activate Nav2 lifecycle nodes manually
        
        Returns:
            Dictionary with activation results for each node
        """
        try:
            import rclpy
            from lifecycle_msgs.srv import ChangeState
            from lifecycle_msgs.msg import Transition
            
            # Initialize ROS2 if not already done
            if not rclpy.ok():
                rclpy.init()
            
            results = {}
            lifecycle_nodes = [
                'controller_server',
                'planner_server',
                'recoveries_server',
                'bt_navigator',
                'waypoint_follower',
                'velocity_smoother',
            ]
            
            # Create a temporary node for service calls
            temp_node = rclpy.create_node('lifecycle_activator')
            
            for node_name in lifecycle_nodes:
                service_name = f'{node_name}/change_state'
                client = temp_node.create_client(ChangeState, service_name)
                
                if not client.wait_for_service(timeout_sec=2.0):
                    results[node_name] = {
                        'success': False,
                        'error': 'Service not available'
                    }
                    continue
                
                # Request transition to ACTIVE (id=3)
                request = ChangeState.Request()
                request.transition.id = Transition.TRANSITION_ACTIVATE
                request.transition.label = 'activate'
                
                try:
                    future = client.call_async(request)
                    rclpy.spin_until_future_complete(temp_node, future, timeout_sec=5.0)
                    
                    if future.done():
                        response = future.result()
                        if response and response.success:
                            results[node_name] = {
                                'success': True,
                                'message': 'Activated successfully'
                            }
                        else:
                            results[node_name] = {
                                'success': False,
                                'error': 'Transition failed'
                            }
                    else:
                        results[node_name] = {
                            'success': False,
                            'error': 'Timeout waiting for response'
                        }
                except Exception as e:
                    results[node_name] = {
                        'success': False,
                        'error': str(e)
                    }
            
            temp_node.destroy_node()
            return results
        except Exception as e:
            return {
                'error': f'Failed to activate lifecycle nodes: {str(e)}'
            }

