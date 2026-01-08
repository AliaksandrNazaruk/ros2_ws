"""
Data Collector

Collects and stores data from ROS2 topics and navigation history.
"""

import threading
from typing import Dict, List, Optional, Any, Deque
from datetime import datetime
from collections import deque
from .models import NavigationHistoryEntry, NavigationStatus


class DataCollector:
    """Collects and stores navigation data"""
    
    def __init__(self, history_size: int = 1000, topic_buffer_size: int = 100):
        self.history_size = history_size
        self.topic_buffer_size = topic_buffer_size
        
        # Navigation history
        self.navigation_history: Deque[NavigationHistoryEntry] = deque(maxlen=history_size)
        
        # Topic data buffers
        self.topic_buffers: Dict[str, Deque[Dict[str, Any]]] = {}
        
        # Metrics
        self.metrics: Dict[str, Any] = {
            'start_time': datetime.now(),
            'topic_message_counts': {},
            'topic_last_times': {},
            'navigation_commands_sent': 0,
            'navigation_successful': 0,
            'navigation_failed': 0,
            'navigation_cancelled': 0
        }
        
        self.lock = threading.Lock()
    
    def add_navigation_command(self, command_id: str, target_id: str, priority: str):
        """Add navigation command to history"""
        with self.lock:
            entry = NavigationHistoryEntry(
                command_id=command_id,
                target_id=target_id,
                priority=priority,
                timestamp=datetime.now(),
                status=NavigationStatus.NAVIGATING,
                result=None,
                error_code=None,
                error_message=None,
                duration_seconds=None
            )
            self.navigation_history.append(entry)
            self.metrics['navigation_commands_sent'] += 1
    
    def update_navigation_status(self, command_id: str, status: NavigationStatus,
                                error_code: Optional[str] = None,
                                error_message: Optional[str] = None):
        """Update navigation status for a command"""
        with self.lock:
            # Find entry by command_id
            for entry in reversed(self.navigation_history):
                if entry.command_id == command_id:
                    entry.status = status
                    entry.error_code = error_code
                    entry.error_message = error_message
                    
                    # Calculate duration
                    if status in [NavigationStatus.ARRIVED, NavigationStatus.ERROR]:
                        duration = (datetime.now() - entry.timestamp).total_seconds()
                        entry.duration_seconds = duration
                        entry.result = "success" if status == NavigationStatus.ARRIVED else "failed"
                    
                    # Update metrics
                    if status == NavigationStatus.ARRIVED:
                        self.metrics['navigation_successful'] += 1
                    elif status == NavigationStatus.ERROR:
                        self.metrics['navigation_failed'] += 1
                    elif status == NavigationStatus.IDLE and entry.status == NavigationStatus.NAVIGATING:
                        self.metrics['navigation_cancelled'] += 1
                    
                    break
    
    def add_topic_data(self, topic_name: str, data: Dict[str, Any]):
        """Add data from a topic"""
        with self.lock:
            if topic_name not in self.topic_buffers:
                self.topic_buffers[topic_name] = deque(maxlen=self.topic_buffer_size)
            
            self.topic_buffers[topic_name].append({
                'timestamp': datetime.now(),
                'data': data
            })
            
            # Update metrics
            self.metrics['topic_message_counts'][topic_name] = \
                self.metrics['topic_message_counts'].get(topic_name, 0) + 1
            self.metrics['topic_last_times'][topic_name] = datetime.now()
    
    def get_history(self, limit: Optional[int] = None) -> List[NavigationHistoryEntry]:
        """Get navigation history"""
        with self.lock:
            history = list(self.navigation_history)
            if limit:
                return history[-limit:]
            return history
    
    def get_topic_data(self, topic_name: str, limit: Optional[int] = None) -> List[Dict[str, Any]]:
        """Get data from a topic buffer"""
        with self.lock:
            if topic_name not in self.topic_buffers:
                return []
            
            data = list(self.topic_buffers[topic_name])
            if limit:
                return data[-limit:]
            return data
    
    def get_metrics(self) -> Dict[str, Any]:
        """Get collected metrics"""
        with self.lock:
            uptime = (datetime.now() - self.metrics['start_time']).total_seconds()
            
            # Calculate topic frequencies
            topic_frequencies = {}
            for topic_name, count in self.metrics['topic_message_counts'].items():
                if topic_name in self.metrics['topic_last_times']:
                    last_time = self.metrics['topic_last_times'][topic_name]
                    age = (datetime.now() - last_time).total_seconds()
                    if age > 0:
                        # Approximate frequency (simplified)
                        topic_frequencies[topic_name] = count / max(age, 1.0)
            
            return {
                'uptime_seconds': uptime,
                'navigation_commands_sent': self.metrics['navigation_commands_sent'],
                'navigation_successful': self.metrics['navigation_successful'],
                'navigation_failed': self.metrics['navigation_failed'],
                'navigation_cancelled': self.metrics['navigation_cancelled'],
                'topic_message_counts': dict(self.metrics['topic_message_counts']),
                'topic_frequencies': topic_frequencies,
                'topic_last_times': {
                    k: v.isoformat() for k, v in self.metrics['topic_last_times'].items()
                }
            }
    
    def clear_history(self):
        """Clear navigation history"""
        with self.lock:
            self.navigation_history.clear()
            self.metrics['navigation_commands_sent'] = 0
            self.metrics['navigation_successful'] = 0
            self.metrics['navigation_failed'] = 0
            self.metrics['navigation_cancelled'] = 0
    
    def clear_topic_data(self, topic_name: Optional[str] = None):
        """Clear topic data buffers"""
        with self.lock:
            if topic_name:
                if topic_name in self.topic_buffers:
                    self.topic_buffers[topic_name].clear()
                    self.metrics['topic_message_counts'].pop(topic_name, None)
                    self.metrics['topic_last_times'].pop(topic_name, None)
            else:
                self.topic_buffers.clear()
                self.metrics['topic_message_counts'].clear()
                self.metrics['topic_last_times'].clear()

