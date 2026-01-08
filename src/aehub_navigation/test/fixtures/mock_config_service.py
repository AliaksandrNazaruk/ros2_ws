#!/usr/bin/env python3

"""
Mock Config Service for testing

Provides a simple HTTP server that simulates the Broker Config Service API.
"""

from http.server import HTTPServer, BaseHTTPRequestHandler
import json
import threading


class MockConfigServiceHandler(BaseHTTPRequestHandler):
    """HTTP request handler for mock config service"""
    
    config_data = None
    api_key = None
    
    def do_GET(self):
        """Handle GET requests"""
        if self.path.startswith('/config/broker'):
            # Check API key
            req_api_key = self.headers.get('X-API-Key', '')
            if self.api_key and req_api_key != self.api_key:
                self.send_response(401)
                self.send_header('Content-type', 'application/json')
                self.end_headers()
                self.wfile.write(json.dumps({'error': 'Unauthorized'}).encode())
                return
            
            # Return config
            self.send_response(200)
            self.send_header('Content-type', 'application/json')
            self.end_headers()
            self.wfile.write(json.dumps(self.config_data).encode())
        elif self.path == '/health':
            self.send_response(200)
            self.send_header('Content-type', 'application/json')
            self.end_headers()
            self.wfile.write(json.dumps({'status': 'healthy'}).encode())
        else:
            self.send_response(404)
            self.end_headers()
    
    def log_message(self, format, *args):
        """Suppress log messages"""
        pass


class MockConfigService:
    """Mock Config Service HTTP server"""
    
    def __init__(self, port=7900, config_data=None, api_key='test_api_key'):
        self.port = port
        self.config_data = config_data or {
            'broker': 'localhost',
            'broker_port': 1883,
            'mqtt_user': 'testuser',
            'mqtt_password': 'testpass',
            'mqtt_use_tls': False,
            'mqtt_tls_insecure': False
        }
        self.api_key = api_key
        self.server = None
        self.thread = None
    
    def start(self):
        """Start the mock server"""
        # Set class variables for handler
        MockConfigServiceHandler.config_data = self.config_data
        MockConfigServiceHandler.api_key = self.api_key
        
        self.server = HTTPServer(('localhost', self.port), MockConfigServiceHandler)
        
        def run_server():
            self.server.serve_forever()
        
        self.thread = threading.Thread(target=run_server, daemon=True)
        self.thread.start()
    
    def stop(self):
        """Stop the mock server"""
        if self.server:
            self.server.shutdown()
            self.server.server_close()
            if self.thread:
                self.thread.join(timeout=1.0)
    
    def update_config(self, new_config):
        """Update the configuration data"""
        self.config_data = new_config
        # Update handler's config_data
        MockConfigServiceHandler.config_data = new_config


if __name__ == '__main__':
    # Test the mock server
    import time
    
    service = MockConfigService()
    service.start()
    
    print(f"Mock Config Service started on http://localhost:{service.port}")
    print("Press Ctrl+C to stop")
    
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        pass
    finally:
        service.stop()
        print("Mock Config Service stopped")
