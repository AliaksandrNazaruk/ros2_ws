"""
Navigation Analyzer

Analyzes navigation data and provides statistics.
"""

from typing import Dict, List, Any, Optional
from datetime import datetime
from collections import defaultdict
from .data_collector import DataCollector
from .ros2_monitor import ROS2Monitor
from .models import NavigationStats, TopicAnalysis, Metrics


class NavigationAnalyzer:
    """Analyzes navigation data and provides statistics"""
    
    def __init__(self, data_collector: DataCollector, ros2_monitor: ROS2Monitor):
        self.data_collector = data_collector
        self.ros2_monitor = ros2_monitor
    
    def analyze_navigation_history(self) -> NavigationStats:
        """Analyze navigation history and return statistics"""
        history = self.data_collector.get_history()
        
        if not history:
            return NavigationStats(
                total_commands=0,
                successful=0,
                failed=0,
                cancelled=0,
                average_duration_seconds=None,
                success_rate=0.0,
                error_breakdown={}
            )
        
        successful = 0
        failed = 0
        cancelled = 0
        durations = []
        error_breakdown = defaultdict(int)
        
        for entry in history:
            if entry.status.value == "arrived":
                successful += 1
                if entry.duration_seconds:
                    durations.append(entry.duration_seconds)
            elif entry.status.value == "error":
                failed += 1
                if entry.error_code:
                    error_breakdown[entry.error_code] += 1
            elif entry.status.value == "idle" and entry.result is None:
                # Check if this was a cancellation
                cancelled += 1
        
        total = len(history)
        success_rate = (successful / total * 100.0) if total > 0 else 0.0
        avg_duration = sum(durations) / len(durations) if durations else None
        
        return NavigationStats(
            total_commands=total,
            successful=successful,
            failed=failed,
            cancelled=cancelled,
            average_duration_seconds=avg_duration,
            success_rate=success_rate,
            error_breakdown=dict(error_breakdown)
        )
    
    def analyze_topics(self) -> List[TopicAnalysis]:
        """Analyze ROS2 topics"""
        topics_info = self.ros2_monitor.get_all_topics()
        analyses = []
        
        current_time = datetime.now()
        
        for topic_name, info in topics_info.items():
            last_msg_time = info.get('last_message_time')
            if last_msg_time:
                if isinstance(last_msg_time, datetime):
                    age_seconds = (current_time - last_msg_time).total_seconds()
                else:
                    age_seconds = 0.0
            else:
                age_seconds = float('inf')
            
            frequency = info.get('frequency_hz', 0.0)
            message_count = info.get('message_count', 0)
            is_active = age_seconds < 5.0  # Consider active if last message < 5 seconds ago
            
            analyses.append(TopicAnalysis(
                name=topic_name,
                frequency_hz=frequency,
                message_count=message_count,
                last_message_age_seconds=age_seconds if age_seconds != float('inf') else -1.0,
                is_active=is_active
            ))
        
        return analyses
    
    def analyze_nav2(self) -> Dict[str, Any]:
        """Analyze Nav2 status and performance"""
        nav2_status = self.ros2_monitor.get_nav2_status()
        
        # Get nodes to check Nav2 components
        nodes = self.ros2_monitor.get_nodes()
        nav2_nodes = [n for n in nodes if any(
            nav2_name in n['name'] for nav2_name in 
            ['bt_navigator', 'controller_server', 'planner_server', 'amcl', 'map_server']
        )]
        
        return {
            'action_server_available': nav2_status.get('action_server_available', False),
            'active_goal': nav2_status.get('active_goal', False),
            'nav2_nodes_running': len(nav2_nodes),
            'nav2_nodes': [n['name'] for n in nav2_nodes]
        }
    
    def get_statistics(self) -> Dict[str, Any]:
        """Get comprehensive statistics"""
        nav_stats = self.analyze_navigation_history()
        topic_analyses = self.analyze_topics()
        nav2_analysis = self.analyze_nav2()
        metrics = self.data_collector.get_metrics()
        
        return {
            'navigation': nav_stats.dict(),
            'topics': [ta.dict() for ta in topic_analyses],
            'nav2': nav2_analysis,
            'metrics': metrics,
            'timestamp': datetime.now().isoformat()
        }
    
    def get_full_report(self) -> Dict[str, Any]:
        """Get full analysis report"""
        stats = self.get_statistics()
        history = self.data_collector.get_history(limit=50)  # Last 50 entries
        
        return {
            'statistics': stats,
            'recent_history': [entry.dict() for entry in history],
            'system_status': {
                'ros2_connected': True,  # If monitor is running, ROS2 is connected
                'topics_monitored': len(self.ros2_monitor.topic_types),
                'nodes_available': len(self.ros2_monitor.get_nodes())
            },
            'generated_at': datetime.now().isoformat()
        }

