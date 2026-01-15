#!/usr/bin/env python3

"""
Symovo Scan Converter Node

Converts PNG laser scan from Symovo API to ROS2 sensor_msgs/LaserScan.
Periodically requests /v0/agv/{id}/scan.png and publishes on /scan topic.

⚠️  IMPORTANT DISCLAIMER (AE.HUB MVP Requirement):
This converter uses a PLACEHOLDER implementation for PNG→LaserScan conversion.
The actual conversion logic depends on Symovo's PNG encoding format, which may
not be documented or may differ from the assumptions made here.

LIMITATIONS:
- The conversion assumes a polar plot representation (may not be accurate)
- Distance mapping is simplified and may not reflect real sensor data
- This implementation is intended for DEMO/TESTING purposes only
- For production use, verify the PNG format with Symovo documentation
- Consider using actual laser scan data if available via other API endpoints

USE AT YOUR OWN RISK in production environments.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import cv2
import numpy as np
import requests
import ssl
from urllib.parse import urljoin
import math


class SymovoScanConverter(Node):
    def __init__(self):
        super().__init__('symovo_scan_converter')
        
        # Parameters
        self.declare_parameter('symovo_endpoint', 'https://192.168.1.100')
        self.declare_parameter('amr_id', 15)
        self.declare_parameter('update_rate', 10.0)  # Hz
        self.declare_parameter('tls_verify', True)  # Default True for production security
        self.declare_parameter('demo_mode', True)  # DEPRECATED: Use tls_verify instead
        self.declare_parameter('frame_id', 'laser')
        
        # Scanner parameters (adjust based on your Symovo AMR scanner specs)
        self.declare_parameter('angle_min', -math.pi)  # -180 degrees
        self.declare_parameter('angle_max', math.pi)   # +180 degrees
        self.declare_parameter('angle_increment', math.pi / 360.0)  # 0.5 degrees
        self.declare_parameter('range_min', 0.1)  # meters
        self.declare_parameter('range_max', 10.0)  # meters
        self.declare_parameter('scan_time', 0.1)  # seconds
        self.declare_parameter('time_increment', 0.0)
        self.declare_parameter('warn_on_placeholder', True)  # Warn about placeholder implementation
        
        # Get parameters
        self.endpoint = self.get_parameter('symovo_endpoint').value
        self.amr_id = self.get_parameter('amr_id').value
        self.update_rate = self.get_parameter('update_rate').value
        demo_mode = self.get_parameter('demo_mode').value
        tls_verify_param = self.get_parameter('tls_verify').value
        
        # TLS verification logic:
        # - If tls_verify is explicitly True, use secure TLS (production mode)
        # - If tls_verify is False, use insecure TLS (demo mode)
        # - demo_mode parameter is deprecated but kept for backward compatibility
        #   If demo_mode=False and tls_verify=False, warn user
        self.tls_insecure = not tls_verify_param
        
        # Warn if insecure TLS is used
        if self.tls_insecure:
            if not demo_mode:
                self.get_logger().warn(
                    '⚠️  INSECURE TLS MODE: tls_verify=False. '
                    'This is unsafe for production! Set tls_verify:=true for secure connections.'
                )
            else:
                self.get_logger().info(
                    'Demo mode: Using insecure TLS (tls_verify=False). '
                    'For production, set tls_verify:=true and demo_mode:=false'
                )
        else:
            self.get_logger().info('✅ Secure TLS mode enabled (tls_verify=True)')
        self.frame_id = self.get_parameter('frame_id').value
        
        # Scanner parameters
        self.angle_min = self.get_parameter('angle_min').value
        self.angle_max = self.get_parameter('angle_max').value
        self.angle_increment = self.get_parameter('angle_increment').value
        self.range_min = self.get_parameter('range_min').value
        self.range_max = self.get_parameter('range_max').value
        self.scan_time = self.get_parameter('scan_time').value
        self.time_increment = self.get_parameter('time_increment').value
        warn_on_placeholder = self.get_parameter('warn_on_placeholder').value
        
        # Create HTTP session
        self.session = requests.Session()
        if self.tls_insecure:
            self.session.verify = False
            import urllib3
            urllib3.disable_warnings(urllib3.exceptions.InsecureRequestWarning)
        
        # Publisher
        self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)
        
        # Timer for periodic updates
        timer_period = 1.0 / self.update_rate
        self.timer = self.create_timer(timer_period, self.update_scan)
        
        self.get_logger().info(
            f'Symovo Scan Converter started: AMR ID={self.amr_id}, '
            f'endpoint={self.endpoint}, rate={self.update_rate} Hz'
        )
        
        # Warn about placeholder implementation
        if warn_on_placeholder:
            self.get_logger().warn(
                '⚠️  PLACEHOLDER IMPLEMENTATION: PNG→LaserScan conversion uses simplified assumptions. '
                'This may not reflect real sensor data. For production use, verify PNG format with '
                'Symovo documentation. See OpenAPI spec: /v0/amr/{id}/scan.png endpoint.'
            )
        
    def update_scan(self):
        """Request scan from Symovo API and publish as LaserScan with improved validation"""
        try:
            # Validate endpoint and AMR ID before request
            if not self.endpoint or not isinstance(self.endpoint, str):
                self.get_logger().error(f'Invalid endpoint: {self.endpoint}')
                return
            
            if not isinstance(self.amr_id, int) or self.amr_id < 0:
                self.get_logger().error(f'Invalid AMR ID: {self.amr_id}')
                return
            
            # Request scan PNG
            # Symovo API (matches base_controller endpoint family):
            # - status/pose: /v0/agv/{id}
            # - scan image: /v0/agv/{id}/scan.png
            url = f'{self.endpoint}/v0/agv/{self.amr_id}/scan.png'
            
            # Validate URL format
            if not url.startswith(('http://', 'https://')):
                self.get_logger().error(f'Invalid URL format: {url}')
                return
            
            try:
                response = self.session.get(url, timeout=5.0)
            except requests.exceptions.Timeout:
                self.get_logger().warn(f'Timeout requesting scan from {url}')
                return
            except requests.exceptions.ConnectionError as e:
                self.get_logger().warn(f'Connection error requesting scan: {e}')
                return
            except requests.exceptions.RequestException as e:
                # rclpy logger doesn't support exc_info kwarg; log message only
                self.get_logger().error(f'Request error: {e}')
                return
            
            # Validate response
            if response.status_code != 200:
                # According to OpenAPI spec:
                # 404: AMR with given ID does not exist
                # 503: AMR is not ready to answer this request. Try again.
                if response.status_code == 404:
                    self.get_logger().debug(
                        f'AMR {self.amr_id} not found or scan endpoint unavailable: HTTP 404'
                    )
                elif response.status_code == 503:
                    self.get_logger().debug(
                        f'AMR {self.amr_id} not ready for scan request: HTTP 503 (will retry)'
                    )
                else:
                    self.get_logger().warn(
                        f'Failed to get scan: HTTP {response.status_code} (URL: {url})'
                    )
                return
            
            # Validate content type
            content_type = response.headers.get('Content-Type', '')
            if 'image/png' not in content_type and 'image' not in content_type.lower():
                self.get_logger().warn(
                    f'Unexpected Content-Type: {content_type}, expected image/png. '
                    f'Proceeding anyway, but data may be invalid.'
                )
            
            # Validate response size (prevent DoS)
            max_response_size = 10 * 1024 * 1024  # 10 MB limit
            if len(response.content) > max_response_size:
                self.get_logger().error(
                    f'Response too large: {len(response.content)} bytes (max: {max_response_size})'
                )
                return
            
            if len(response.content) == 0:
                self.get_logger().warn('Empty response from scan endpoint')
                return
            
            # Decode PNG image
            # Reference: Symovo endpoint /v0/agv/{id}/scan.png
            # The scan is returned as PNG image (see openapi spec line ~2133)
            try:
                # Validate PNG signature (first 8 bytes should be PNG magic number)
                png_signature = b'\x89PNG\r\n\x1a\n'
                if not response.content.startswith(png_signature):
                    self.get_logger().warn(
                        f'Response does not appear to be a valid PNG file. '
                        f'First bytes: {response.content[:8].hex()}'
                    )
                    # Continue anyway - some servers may not include signature
                
                nparr = np.frombuffer(response.content, np.uint8)
                
                # Validate array size
                if len(nparr) == 0:
                    self.get_logger().warn('Empty image data')
                    return
                
                img = cv2.imdecode(nparr, cv2.IMREAD_GRAYSCALE)
                
                if img is None:
                    self.get_logger().warn(
                        f'Failed to decode PNG scan image from {url}. '
                        f'Response size: {len(response.content)} bytes, '
                        f'Content-Type: {response.headers.get("Content-Type", "unknown")}'
                    )
                    return
                
                # Validate image dimensions
                if img.size == 0:
                    self.get_logger().warn('Decoded PNG image is empty')
                    return
                
                # Validate reasonable image dimensions (prevent memory exhaustion)
                height, width = img.shape[:2]
                max_dimension = 4096  # Reasonable maximum
                if width > max_dimension or height > max_dimension:
                    self.get_logger().error(
                        f'Image dimensions too large: {width}x{height} '
                        f'(max: {max_dimension}x{max_dimension})'
                    )
                    return
                
                if width < 10 or height < 10:
                    self.get_logger().warn(
                        f'Image dimensions suspiciously small: {width}x{height}'
                    )
                    # Continue anyway - may be valid
                    
            except ValueError as e:
                self.get_logger().error(f'Invalid PNG data format: {e}')
                return
            except MemoryError as e:
                self.get_logger().error(f'Out of memory decoding PNG: {e}')
                return
            except Exception as e:
                self.get_logger().error(f'Unexpected error decoding PNG: {e}')
                return
            
            # Convert PNG to LaserScan with validation
            try:
                scan_msg = self.png_to_laserscan(img)
                
                # Validate LaserScan message before publishing
                if not self._validate_laserscan(scan_msg):
                    self.get_logger().warn('Generated LaserScan message failed validation, skipping publish')
                    return
                
                # Publish
                self.scan_pub.publish(scan_msg)
                
            except Exception as e:
                self.get_logger().error(f'Error converting/publishing scan: {e}')
            
        except Exception as e:
            self.get_logger().error(f'Error updating scan: {e}')
    
    def _validate_laserscan(self, scan_msg):
        """Validate LaserScan message before publishing"""
        try:
            # Check required fields
            if not hasattr(scan_msg, 'ranges') or scan_msg.ranges is None:
                self.get_logger().error('LaserScan missing ranges')
                return False
            
            # Validate ranges array
            if len(scan_msg.ranges) == 0:
                self.get_logger().warn('LaserScan has empty ranges array')
                return False
            
            # Validate range values
            valid_ranges = 0
            for r in scan_msg.ranges:
                if isinstance(r, (int, float)):
                    if r < 0 and not math.isinf(r):
                        self.get_logger().warn(f'Invalid range value: {r} (negative and not inf)')
                    elif math.isfinite(r) and (r < self.range_min or r > self.range_max):
                        # Out of bounds but finite - may be valid depending on sensor
                        pass
                    if math.isfinite(r):
                        valid_ranges += 1
                else:
                    self.get_logger().warn(f'Non-numeric range value: {type(r)}')
            
            if valid_ranges == 0:
                self.get_logger().warn('No valid finite ranges in LaserScan')
                # Don't fail - may be intentional (all obstacles)
            
            # Validate angles
            if scan_msg.angle_min >= scan_msg.angle_max:
                self.get_logger().error(f'Invalid angle range: min={scan_msg.angle_min}, max={scan_msg.angle_max}')
                return False
            
            # Validate expected number of rays matches ranges length
            expected_rays = int((scan_msg.angle_max - scan_msg.angle_min) / scan_msg.angle_increment)
            if abs(len(scan_msg.ranges) - expected_rays) > 1:  # Allow 1 ray difference for rounding
                self.get_logger().warn(
                    f'Range count mismatch: expected ~{expected_rays}, got {len(scan_msg.ranges)}'
                )
                # Don't fail - may be due to rounding
            
            return True
            
        except Exception as e:
            self.get_logger().error(f'Error validating LaserScan: {e}')
            return False
    
    def png_to_laserscan(self, img):
        """
        Convert PNG scan image to LaserScan message.
        
        ⚠️  PLACEHOLDER IMPLEMENTATION (AE.HUB MVP Requirement):
        This is a simplified conversion. The actual implementation depends on
        how Symovo encodes the scan data in the PNG image.
        
        Reference: Symovo endpoint /v0/agv/{id}/scan.png
        The scan is returned as PNG image, but the exact encoding format is not
        documented in the OpenAPI specification.
        
        Assumptions (may not be accurate):
        - Image is a polar plot of scan data
        - Each row/column represents an angle
        - Pixel intensity represents distance
        
        WARNING: This conversion may not produce accurate LaserScan data.
        For production use, verify the PNG format with Symovo documentation
        or contact Symovo support for format specification.
        """
        scan_msg = LaserScan()
        scan_msg.header.stamp = self.get_clock().now().to_msg()
        scan_msg.header.frame_id = self.frame_id
        
        scan_msg.angle_min = self.angle_min
        scan_msg.angle_max = self.angle_max
        scan_msg.angle_increment = self.angle_increment
        scan_msg.time_increment = self.time_increment
        scan_msg.scan_time = self.scan_time
        scan_msg.range_min = self.range_min
        scan_msg.range_max = self.range_max
        
        # Calculate number of rays
        num_rays = int((self.angle_max - self.angle_min) / self.angle_increment)
        
        # Convert image to ranges
        # This is a placeholder - actual conversion depends on Symovo's PNG format
        # Option 1: If image is a polar plot (angle vs distance)
        # Option 2: If image is a top-down view (need to extract radial distances)
        # Option 3: If image contains encoded distance data
        
        # For now, use a simple conversion: extract radial distances from center
        height, width = img.shape
        center_x, center_y = width // 2, height // 2
        
        ranges = []
        for i in range(num_rays):
            angle = self.angle_min + i * self.angle_increment
            
            # Calculate pixel coordinates for this angle
            # Assuming image center is robot position
            max_radius = min(width, height) // 2
            
            # Sample along the ray
            distances = []
            for r in range(1, max_radius):
                x = int(center_x + r * math.cos(angle))
                y = int(center_y + r * math.sin(angle))
                
                if 0 <= x < width and 0 <= y < height:
                    pixel_value = img[y, x]
                    # Convert pixel value to distance
                    # This depends on how Symovo encodes distance in PNG
                    # Assuming: darker = closer, lighter = farther
                    # Or: specific encoding scheme
                    distance = self.pixel_to_distance(pixel_value, r, max_radius)
                    if self.range_min <= distance <= self.range_max:
                        distances.append(distance)
            
            # Use closest valid distance (or implement more sophisticated logic)
            if distances:
                ranges.append(min(distances))
            else:
                ranges.append(float('inf'))  # No obstacle detected
        
        scan_msg.ranges = ranges
        
        # Calculate intensities (optional)
        scan_msg.intensities = [0.0] * len(ranges)
        
        return scan_msg
    
    def pixel_to_distance(self, pixel_value, radius, max_radius):
        """
        Convert pixel value to distance in meters.
        
        This is a placeholder - actual conversion depends on Symovo's encoding.
        Common approaches:
        1. Pixel value directly represents distance
        2. Pixel intensity represents reflectivity, distance encoded differently
        3. Color channels encode different information
        """
        # Placeholder: simple linear mapping
        # Adjust based on actual Symovo PNG format
        normalized = pixel_value / 255.0
        distance = self.range_min + normalized * (self.range_max - self.range_min)
        return distance


def main(args=None):
    rclpy.init(args=args)
    node = SymovoScanConverter()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

