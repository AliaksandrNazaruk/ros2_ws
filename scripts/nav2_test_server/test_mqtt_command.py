#!/usr/bin/env python3
"""Простой скрипт для отправки команды навигации через MQTT"""

import sys
import os
import importlib.util

script_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, script_dir)

# Загружаем модули напрямую
spec_config = importlib.util.spec_from_file_location("nav2_test_server.config", os.path.join(script_dir, "config.py"))
config_module = importlib.util.module_from_spec(spec_config)
sys.modules['nav2_test_server'] = type(sys)('nav2_test_server')
sys.modules['nav2_test_server.config'] = config_module
spec_config.loader.exec_module(config_module)

spec_mqtt = importlib.util.spec_from_file_location("nav2_test_server.mqtt_client", os.path.join(script_dir, "mqtt_client.py"))
mqtt_client_module = importlib.util.module_from_spec(spec_mqtt)
sys.modules['nav2_test_server.mqtt_client'] = mqtt_client_module
spec_mqtt.loader.exec_module(mqtt_client_module)

MQTTTestClient = mqtt_client_module.MQTTTestClient
Config = config_module.Config

import time

def main():
    config = Config()
    client = MQTTTestClient(config)
    
    print("Подключение к MQTT...")
    if not client.connect():
        print("Ошибка: не удалось подключиться к MQTT")
        return 1
    
    print("Отправка команды навигации к position_B...")
    command_id = client.send_navigate_command('position_B', 'normal')
    print(f"Команда отправлена: command_id={command_id}")
    print("Целевая позиция: position_B (из YAML файла)")
    print("Ожидание 20 секунд для проверки обработки команды...")
    
    time.sleep(20)
    
    client.disconnect()
    print("Отключено от MQTT")
    return 0

if __name__ == "__main__":
    sys.exit(main())

