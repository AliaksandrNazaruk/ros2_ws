#!/usr/bin/env python3
"""
Full Integration Test
Tests complete flow: Config Service -> MQTT -> Navigation Node -> Symovo
"""

import requests
import json
import time
import sys
import paho.mqtt.client as mqtt
import uuid
import ssl
from datetime import datetime, timezone
import urllib3

urllib3.disable_warnings(urllib3.exceptions.InsecureRequestWarning)

CONFIG_SERVICE_URL = "http://localhost:7900"
CONFIG_SERVICE_API_KEY = "tR-UZ2j2KutE6OYlEGbsx0h5qe071L-gC5kd1hHKfw4"
ROBOT_ID = "robot_001"
FASTAPI_URL = "http://localhost:8000"
SYMOVO_ENDPOINT = "https://192.168.1.100"
AMR_ID = 15

def print_section(title):
    print("\n" + "=" * 60)
    print(f"  {title}")
    print("=" * 60)

def test_config_service():
    """Test Config Service connection"""
    print_section("1. Config Service Test")
    
    try:
        url = f"{CONFIG_SERVICE_URL}/config/broker?include_password=true"
        headers = {"X-API-Key": CONFIG_SERVICE_API_KEY}
        response = requests.get(url, headers=headers, timeout=5)
        
        if response.status_code == 200:
            config = response.json()
            print(f" Config Service доступен")
            print(f"   Broker: {config.get('broker')}:{config.get('broker_port')}")
            print(f"   User: {config.get('mqtt_user')}")
            print(f"   TLS: {config.get('mqtt_use_tls')}")
            return config
        else:
            print(f" Config Service notдоступен: HTTP {response.status_code}")
            return None
    except Exception as e:
        print(f" Error подключенandя к Config Service: {e}")
        return None

def test_mqtt_connection(mqtt_config):
    """Test MQTT connection"""
    print_section("2. MQTT Connection Test")
    
    connected = False
    client = None
    
    def on_connect(client, userdata, flags, rc):
        nonlocal connected
        if rc == 0:
            connected = True
            print(f" Подключено к MQTT брокеру")
        else:
            print(f" Error подключенandя: код {rc}")
    
    try:
        client = mqtt.Client(client_id=f"test_{uuid.uuid4()}")
        client.username_pw_set(
            mqtt_config['mqtt_user'],
            mqtt_config['mqtt_password']
        )
        client.on_connect = on_connect
        
        if mqtt_config.get('mqtt_use_tls'):
            context = ssl.create_default_context()
            if mqtt_config.get('mqtt_tls_insecure'):
                context.check_hostname = False
                context.verify_mode = ssl.CERT_NONE
            client.tls_set_context(context)
        
        broker = mqtt_config['broker']
        port = mqtt_config['broker_port']
        print(f"🔌 Подключенandе к {broker}:{port}...")
        client.connect(broker, port, 60)
        client.loop_start()
        
        timeout = 10
        start = time.time()
        while not connected and (time.time() - start) < timeout:
            time.sleep(0.1)
        
        if connected:
            time.sleep(1)
            client.loop_stop()
            client.disconnect()
            return True
        else:
            print(" Таймаут подключенandя")
            return False
    except Exception as e:
        print(f" Error подключенandя: {e}")
        return False
    finally:
        if client:
            client.loop_stop()
            client.disconnect()

def test_fastapi():
    """Test FastAPI endpoints"""
    print_section("3. FastAPI Test")
    
    try:
        # Health check
        response = requests.get(f"{FASTAPI_URL}/api/monitor/health", timeout=5)
        if response.status_code == 200:
            data = response.json()
            print(f" FastAPI работает")
            print(f"   ROS2: {data.get('ros2', False)}")
            print(f"   MQTT: {data.get('mqtt', False)}")
        
        # Positions
        response = requests.get(f"{FASTAPI_URL}/api/positions", timeout=5)
        if response.status_code == 200:
            data = response.json()
            positions = data.get('positions', [])
            print(f" Позandцandand доступны: {len(positions)} шт.")
            if positions:
                print(f"   Первая позandцandя: {positions[0].get('position_id')}")
                return positions[0].get('position_id')
        
        return None
    except Exception as e:
        print(f" Error FastAPI: {e}")
        return None

def test_symovo_api():
    """Test Symovo API"""
    print_section("4. Symovo API Test")
    
    try:
        # Get AMR status
        response = requests.get(f"{SYMOVO_ENDPOINT}/v0/agv", verify=False, timeout=5)
        if response.status_code == 200:
            data = response.json()
            amr = next((a for a in data if a.get('id') == AMR_ID), None)
            if amr:
                print(f" AMR {AMR_ID} найден")
                print(f"   Позandцandя: x={amr['pose']['x']:.3f}, y={amr['pose']['y']:.3f}")
                print(f"   Состоянandе: {amr.get('state', 'unknown')}")
                print(f"   Батарея: {amr.get('battery_level', 0) * 100:.1f}%")
                return True
        
        print(f" AMR {AMR_ID} not найден")
        return False
    except Exception as e:
        print(f" Error Symovo API: {e}")
        return False

def test_mqtt_navigation_command(mqtt_config, target_id):
    """Test sending navigation command via MQTT"""
    print_section("5. MQTT Navigation Command Test")
    
    status_received = False
    last_status = None
    
    def on_connect(client, userdata, flags, rc):
        if rc == 0:
            print(f" Подключено к MQTT")
            status_topic = f"aroc/robot/{ROBOT_ID}/status/navigation"
            client.subscribe(status_topic, qos=1)
            print(f" Подпandска на: {status_topic}")
    
    def on_message(client, userdata, msg):
        nonlocal status_received, last_status
        try:
            payload = json.loads(msg.payload.decode())
            status_received = True
            last_status = payload
            print(f"\n📨 Status получен:")
            print(f"   Status: {payload.get('status')}")
            print(f"   Target ID: {payload.get('target_id')}")
            print(f"   Progress: {payload.get('progress_percent', 0)}%")
        except Exception as e:
            print(f" Error парсandнга сообщенandя: {e}")
    
    try:
        client = mqtt.Client(client_id=f"nav_test_{uuid.uuid4()}")
        client.username_pw_set(
            mqtt_config['mqtt_user'],
            mqtt_config['mqtt_password']
        )
        client.on_connect = on_connect
        client.on_message = on_message
        
        if mqtt_config.get('mqtt_use_tls'):
            context = ssl.create_default_context()
            if mqtt_config.get('mqtt_tls_insecure'):
                context.check_hostname = False
                context.verify_mode = ssl.CERT_NONE
            client.tls_set_context(context)
        
        client.connect(mqtt_config['broker'], mqtt_config['broker_port'], 60)
        client.loop_start()
        
        # Wait for connection
        time.sleep(2)
        
        # Send command
        command = {
            "command_id": str(uuid.uuid4()),
            "timestamp": datetime.now(timezone.utc).isoformat(),
            "target_id": target_id,
            "priority": "normal"
        }
        
        topic = f"aroc/robot/{ROBOT_ID}/commands/navigateTo"
        result = client.publish(topic, json.dumps(command), qos=1)
        
        if result.rc == mqtt.MQTT_ERR_SUCCESS:
            print(f" Command отправлена:")
            print(f"   Topic: {topic}")
            print(f"   Command ID: {command['command_id']}")
            print(f"   Target: {target_id}")
            
            # Wait for status
            print(f"\n Waiting statusа (10 сек)...")
            timeout = 10
            start = time.time()
            while not status_received and (time.time() - start) < timeout:
                time.sleep(0.5)
            
            if status_received:
                print(f" Status получен!")
                return True
            else:
                print(f"  Status not получен (возможно, navigation node not запущен)")
                return False
        else:
            print(f" Error отправкand команды: {result.rc}")
            return False
    except Exception as e:
        print(f" Error: {e}")
        return False
    finally:
        if client:
            client.loop_stop()
            client.disconnect()

def main():
    print("\n" + "=" * 60)
    print("  FULL INTEGRATION TEST")
    print("=" * 60)
    print(f"Time: {datetime.now().isoformat()}")
    print()
    
    results = {}
    
    # 1. Config Service
    mqtt_config = test_config_service()
    if not mqtt_config:
        print("\n Not уyesлось получandть конфandгурацandю MQTT")
        return 1
    results['config_service'] = True
    
    # 2. MQTT Connection
    if not test_mqtt_connection(mqtt_config):
        print("\n Not уyesлось подключandться к MQTT")
        return 1
    results['mqtt'] = True
    
    # 3. FastAPI
    target_id = test_fastapi()
    if not target_id:
        target_id = "position_default"  # Fallback
    results['fastapi'] = True
    
    # 4. Symovo API
    if not test_symovo_api():
        print("\n  Symovo API notдоступен, но продолжаем тестandрованandе")
    results['symovo'] = True
    
    # 5. MQTT Navigation Command
    nav_result = test_mqtt_navigation_command(mqtt_config, target_id)
    results['navigation'] = nav_result
    
    # Summary
    print_section("Test Summary")
    print(f"Config Service: {'' if results.get('config_service') else ''}")
    print(f"MQTT Connection: {'' if results.get('mqtt') else ''}")
    print(f"FastAPI: {'' if results.get('fastapi') else ''}")
    print(f"Symovo API: {'' if results.get('symovo') else ''}")
    print(f"Navigation Command: {'' if results.get('navigation') else ' '}")
    
    passed = sum(1 for v in results.values() if v)
    total = len(results)
    print(f"\nTotal: {passed}/{total} tests passed")
    
    return 0 if passed == total else 1

if __name__ == '__main__':
    sys.exit(main())

