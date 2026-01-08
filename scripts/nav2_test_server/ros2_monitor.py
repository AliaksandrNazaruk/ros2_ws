"""
ROS2 Monitor

Monitors ROS2 topics, nodes, actions, and TF transforms.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import LaserScan
from tf2_ros import TransformListener, Buffer
from tf2_ros.transform_broadcaster import TransformBroadcaster
from std_msgs.msg import String
import threading
import time
from typing import Dict, Optional, Any, List
from datetime import datetime
from collections import deque
import json


class ROS2Monitor(Node):
    """Monitor ROS2 topics, nodes, and actions"""
    
    def __init__(self, topic_buffer_size: int = 100):
        super().__init__('nav2_test_server_monitor')
        
        self.topic_buffer_size = topic_buffer_size
        self.topic_data: Dict[str, deque] = {}
        self.topic_types: Dict[str, str] = {}
        self.topic_frequencies: Dict[str, float] = {}
        self.last_message_times: Dict[str, datetime] = {}
        self.message_counts: Dict[str, int] = {}
        
        # Thread safety
        self.lock = threading.Lock()
        
        # TF buffer
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Nav2 Action Client for monitoring
        self.nav2_action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # Subscribe to key topics
        self._subscribe_to_topics()
        
        # Start monitoring thread
        self.monitoring = True
        self.monitor_thread = threading.Thread(target=self._monitor_loop, daemon=True)
        self.monitor_thread.start()
    
    def _subscribe_to_topics(self):
        """Subscribe to important topics"""
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Odometry
        self.create_subscription(
            Odometry,
            '/odom',
            lambda msg: self._on_odom(msg),
            qos_profile
        )
        self.topic_types['/odom'] = 'nav_msgs/msg/Odometry'
        
        # Command velocity
        self.create_subscription(
            Twist,
            '/cmd_vel',
            lambda msg: self._on_cmd_vel(msg),
            qos_profile
        )
        self.topic_types['/cmd_vel'] = 'geometry_msgs/msg/Twist'
        
        # Laser scan
        self.create_subscription(
            LaserScan,
            '/scan',
            lambda msg: self._on_scan(msg),
            qos_profile
        )
        self.topic_types['/scan'] = 'sensor_msgs/msg/LaserScan'
        
        # AMCL pose
        self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            lambda msg: self._on_amcl_pose(msg),
            qos_profile
        )
        self.topic_types['/amcl_pose'] = 'geometry_msgs/msg/PoseWithCovarianceStamped'
        
        # Map
        self.create_subscription(
            OccupancyGrid,
            '/map',
            lambda msg: self._on_map(msg),
            qos_profile
        )
        self.topic_types['/map'] = 'nav_msgs/msg/OccupancyGrid'
    
    def _on_odom(self, msg: Odometry):
        """Handle odometry message"""
        with self.lock:
            if '/odom' not in self.topic_data:
                self.topic_data['/odom'] = deque(maxlen=self.topic_buffer_size)
            
            data = {
                'timestamp': datetime.now().isoformat(),
                'position': {
                    'x': msg.pose.pose.position.x,
                    'y': msg.pose.pose.position.y,
                    'z': msg.pose.pose.position.z
                },
                'orientation': {
                    'x': msg.pose.pose.orientation.x,
                    'y': msg.pose.pose.orientation.y,
                    'z': msg.pose.pose.orientation.z,
                    'w': msg.pose.pose.orientation.w
                },
                'velocity': {
                    'linear': {
                        'x': msg.twist.twist.linear.x,
                        'y': msg.twist.twist.linear.y,
                        'z': msg.twist.twist.linear.z
                    },
                    'angular': {
                        'x': msg.twist.twist.angular.x,
                        'y': msg.twist.twist.angular.y,
                        'z': msg.twist.twist.angular.z
                    }
                }
            }
            self.topic_data['/odom'].append(data)
            self.last_message_times['/odom'] = datetime.now()
            self.message_counts['/odom'] = self.message_counts.get('/odom', 0) + 1
    
    def _on_cmd_vel(self, msg: Twist):
        """Handle command velocity message"""
        with self.lock:
            if '/cmd_vel' not in self.topic_data:
                self.topic_data['/cmd_vel'] = deque(maxlen=self.topic_buffer_size)
            
            data = {
                'timestamp': datetime.now().isoformat(),
                'linear': {
                    'x': msg.linear.x,
                    'y': msg.linear.y,
                    'z': msg.linear.z
                },
                'angular': {
                    'x': msg.angular.x,
                    'y': msg.angular.y,
                    'z': msg.angular.z
                }
            }
            self.topic_data['/cmd_vel'].append(data)
            self.last_message_times['/cmd_vel'] = datetime.now()
            self.message_counts['/cmd_vel'] = self.message_counts.get('/cmd_vel', 0) + 1
    
    def _on_scan(self, msg: LaserScan):
        """Handle laser scan message"""
        with self.lock:
            if '/scan' not in self.topic_data:
                self.topic_data['/scan'] = deque(maxlen=self.topic_buffer_size)
            
            data = {
                'timestamp': datetime.now().isoformat(),
                'angle_min': msg.angle_min,
                'angle_max': msg.angle_max,
                'angle_increment': msg.angle_increment,
                'range_min': msg.range_min,
                'range_max': msg.range_max,
                'ranges_count': len(msg.ranges),
                'intensities_count': len(msg.intensities) if msg.intensities else 0
            }
            self.topic_data['/scan'].append(data)
            self.last_message_times['/scan'] = datetime.now()
            self.message_counts['/scan'] = self.message_counts.get('/scan', 0) + 1
    
    def _on_amcl_pose(self, msg: PoseWithCovarianceStamped):
        """Handle AMCL pose message"""
        with self.lock:
            if '/amcl_pose' not in self.topic_data:
                self.topic_data['/amcl_pose'] = deque(maxlen=self.topic_buffer_size)
            
            data = {
                'timestamp': datetime.now().isoformat(),
                'position': {
                    'x': msg.pose.pose.position.x,
                    'y': msg.pose.pose.position.y,
                    'z': msg.pose.pose.position.z
                },
                'orientation': {
                    'x': msg.pose.pose.orientation.x,
                    'y': msg.pose.pose.orientation.y,
                    'z': msg.pose.pose.orientation.z,
                    'w': msg.pose.pose.orientation.w
                },
                'covariance': list(msg.pose.covariance)
            }
            self.topic_data['/amcl_pose'].append(data)
            self.last_message_times['/amcl_pose'] = datetime.now()
            self.message_counts['/amcl_pose'] = self.message_counts.get('/amcl_pose', 0) + 1
    
    def _on_map(self, msg: OccupancyGrid):
        """Handle map message"""
        with self.lock:
            if '/map' not in self.topic_data:
                self.topic_data['/map'] = deque(maxlen=1)  # Only keep latest map
            
            data = {
                'timestamp': datetime.now().isoformat(),
                'width': msg.info.width,
                'height': msg.info.height,
                'resolution': msg.info.resolution,
                'origin': {
                    'x': msg.info.origin.position.x,
                    'y': msg.info.origin.position.y,
                    'z': msg.info.origin.position.z
                }
            }
            self.topic_data['/map'] = deque([data], maxlen=1)
            self.last_message_times['/map'] = datetime.now()
            self.message_counts['/map'] = self.message_counts.get('/map', 0) + 1
    
    def _monitor_loop(self):
        """Background monitoring loop"""
        last_frequency_check = time.time()
        check_interval = 5.0  # seconds
        
        while self.monitoring:
            time.sleep(1.0)
            
            # Calculate frequencies periodically
            current_time = time.time()
            if current_time - last_frequency_check >= check_interval:
                self._calculate_frequencies()
                last_frequency_check = current_time
    
    def _calculate_frequencies(self):
        """Calculate topic publication frequencies"""
        with self.lock:
            for topic_name, count in self.message_counts.items():
                if topic_name in self.last_message_times:
                    # Reset count for next period
                    self.message_counts[topic_name] = 0
                    # Frequency is approximate based on message count over interval
                    # This is a simplified calculation
                    if count > 0:
                        self.topic_frequencies[topic_name] = count / 5.0  # messages per second
    
    def get_topic_data(self, topic_name: str) -> Optional[Dict[str, Any]]:
        """
        Get latest data from a monitored topic.
        
        Args:
            topic_name: Name of the topic to get data from
            
        Returns:
            Dictionary containing the latest message data, or None if no data available
        """
        with self.lock:
            if topic_name in self.topic_data and len(self.topic_data[topic_name]) > 0:
                return self.topic_data[topic_name][-1]
            return None
    
    def get_all_topics(self) -> Dict[str, Any]:
        """
        Get information about all monitored topics.
        
        Returns:
            Dictionary mapping topic names to their metadata:
            - type: Message type (e.g., 'nav_msgs/msg/Odometry')
            - last_message_time: Timestamp of last received message
            - message_count: Number of messages received (reset periodically)
            - frequency_hz: Estimated publication frequency
            - data: Latest message data (if available)
        """
        with self.lock:
            topics = {}
            for topic_name in self.topic_types.keys():
                last_msg = self.get_topic_data(topic_name)
                topics[topic_name] = {
                    'type': self.topic_types.get(topic_name, 'unknown'),
                    'last_message_time': self.last_message_times.get(topic_name),
                    'message_count': self.message_counts.get(topic_name, 0),
                    'frequency_hz': self.topic_frequencies.get(topic_name),
                    'data': last_msg
                }
            return topics
    
    def get_nodes(self) -> List[Dict[str, Any]]:
        """
        Get list of active ROS2 nodes with their publishers, subscribers, services, and actions.
        
        Returns:
            List of dictionaries containing node information:
            - name: Full node name (with namespace)
            - namespace: Node namespace
            - publishers: List of topic names published by this node
            - subscribers: List of topic names subscribed by this node
            - services: List of service names provided by this node
            - actions: List of action names (servers and clients) used by this node
            
        Raises:
            No exceptions are raised; errors are logged and empty list is returned.
        """
        try:
            node_names_and_namespaces = self.get_node_names_and_namespaces()
            nodes = []
            seen_names = set()  # Track unique node names
            
            for node_name, namespace in node_names_and_namespaces:
                # Full node name includes namespace
                full_name = namespace + node_name if namespace else node_name
                if not full_name.startswith('/'):
                    full_name = '/' + full_name
                
                # Deduplicate - only add if we haven't seen this exact name
                if full_name not in seen_names:
                    seen_names.add(full_name)
                    
                    # Query node graph for publishers, subscribers, services, and actions
                    publishers = []
                    subscribers = []
                    services = []
                    actions = []
                    
                    # Get publishers
                    try:
                        pub_names_types = self.get_publisher_names_and_types_by_node(node_name, namespace or '')
                        publishers = [name for name, _ in pub_names_types]
                    except Exception as e:
                        self.get_logger().debug(f"Error getting publishers for {full_name}: {e}")
                        publishers = []
                    
                    # Get subscribers
                    try:
                        sub_names_types = self.get_subscriber_names_and_types_by_node(node_name, namespace or '')
                        subscribers = [name for name, _ in sub_names_types]
                    except Exception as e:
                        self.get_logger().debug(f"Error getting subscribers for {full_name}: {e}")
                        subscribers = []
                    
                    # Get services (both servers and clients)
                    try:
                        service_names_types = self.get_service_names_and_types_by_node(node_name, namespace or '')
                        services = [name for name, _ in service_names_types]
                    except Exception as e:
                        self.get_logger().debug(f"Error getting services for {full_name}: {e}")
                        services = []
                    
                    # Get actions (both servers and clients)
                    try:
                        action_server_names_types = self.get_action_server_names_and_types_by_node(node_name, namespace or '')
                        action_client_names_types = self.get_action_client_names_and_types_by_node(node_name, namespace or '')
                        # Combine and deduplicate action names
                        action_names = set()
                        for name, _ in action_server_names_types:
                            action_names.add(name)
                        for name, _ in action_client_names_types:
                            action_names.add(name)
                        actions = list(action_names)
                    except Exception as e:
                        self.get_logger().debug(f"Error getting actions for {full_name}: {e}")
                        actions = []
                    
                    node_data = {
                        'name': full_name,
                        'namespace': namespace,
                        'publishers': publishers,
                        'subscribers': subscribers,
                        'services': services,
                        'actions': actions
                    }
                    nodes.append(node_data)
            
            return nodes
        except Exception as e:
            self.get_logger().error(f"Error getting nodes: {e}")
            return []
    
    def get_nav2_status(self) -> Dict[str, Any]:
        """
        Get Nav2 action server status.
        
        Returns:
            Dictionary containing:
            - action_server_available: Whether the Nav2 action server is available
            - action_server_name: Name of the action server ('navigate_to_pose')
            - active_goal: Whether there is an active navigation goal (simplified check)
            - goal_id: ID of active goal (None if not available)
        """
        try:
            server_available = self.nav2_action_client.wait_for_server(timeout_sec=0.1)
            
            # Check for active goals (simplified - would need more complex logic)
            active_goal = False
            goal_id = None
            
            return {
                'action_server_available': server_available,
                'action_server_name': 'navigate_to_pose',
                'active_goal': active_goal,
                'goal_id': goal_id
            }
        except Exception as e:
            self.get_logger().error(f"Error checking Nav2 status: {e}")
            return {
                'action_server_available': False,
                'action_server_name': 'navigate_to_pose',
                'active_goal': False,
                'goal_id': None
            }
    
    def get_tf_transform(self, parent_frame: str, child_frame: str) -> Optional[Dict[str, Any]]:
        """
        Get TF transform between two frames.
        
        Args:
            parent_frame: Parent frame name
            child_frame: Child frame name
            
        Returns:
            Dictionary containing transform information:
            - parent_frame: Parent frame name
            - child_frame: Child frame name
            - translation: Translation vector (x, y, z)
            - rotation: Rotation quaternion (x, y, z, w)
            - available: Whether the transform is available
            
            Returns identity transform if transform is not available.
        """
        try:
            transform = self.tf_buffer.lookup_transform(
                parent_frame,
                child_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1)
            )
            
            return {
                'parent_frame': parent_frame,
                'child_frame': child_frame,
                'translation': {
                    'x': transform.transform.translation.x,
                    'y': transform.transform.translation.y,
                    'z': transform.transform.translation.z
                },
                'rotation': {
                    'x': transform.transform.rotation.x,
                    'y': transform.transform.rotation.y,
                    'z': transform.transform.rotation.z,
                    'w': transform.transform.rotation.w
                },
                'available': True
            }
        except Exception as e:
            self.get_logger().debug(f"TF transform not available from {parent_frame} to {child_frame}: {e}")
            return {
                'parent_frame': parent_frame,
                'child_frame': child_frame,
                'translation': {'x': 0.0, 'y': 0.0, 'z': 0.0},
                'rotation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0},
                'available': False
            }
    
    def stop(self):
        """Stop monitoring"""
        self.monitoring = False
        if self.monitor_thread.is_alive():
            self.monitor_thread.join(timeout=2.0)

