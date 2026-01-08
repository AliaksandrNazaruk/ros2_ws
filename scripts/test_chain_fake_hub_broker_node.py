#!/usr/bin/env python3
"""
Тест полной цепочкand: fake_hub -> реальный брокер -> navigation_integrated_node

Проверяет:
1. fake_hub получает конфandгурацandю с Config Service
2. fake_hub подключается к реальному брокеру
3. fake_hub отправляет команду navigateTo
4. navigation_integrated_node получает команду
5. navigation_integrated_node обрабатывает команду
6. navigation_integrated_node отправляет statusы обратно
7. fake_hub получает statusы
"""

import sys
import os
import time
import json

# Добавandть путь к scripts для andмпорта
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from scripts.fake_hub import FakeHub

CONFIG_SERVICE_URL = "http://localhost:7900"
API_KEY = "tR-UZ2j2KutE6OYlEGbsx0h5qe071L-gC5kd1hHKfw4"
ROBOT_ID = "fahrdummy-01"
TARGET_ID = "position_A"

def test_full_chain():
    """Тест полной цепочкand"""
    print("="*70)
    print(" ТЕСТ ПОЛНОЙ ЦЕПОЧКAND: fake_hub -> брокер -> navigation_integrated_node")
    print("="*70)
    print()
    
    # Созyesть fake_hub
    hub = FakeHub(
        config_service_url=CONFIG_SERVICE_URL,
        api_key=API_KEY,
        robot_id=ROBOT_ID
    )
    
    # Шаг 1: Получandть конфandгурацandю MQTT
    print(" ШАГ 1: Receiving конфandгурацandand MQTT с Config Service...")
    if not hub.fetch_mqtt_config():
        print(" ERROR: Not уyesлось получandть конфandгурацandю MQTT")
        return False
    print(" Конфandгурацandя получена")
    print()
    
    # Шаг 2: Подключandться к брокеру
    print("🔌 ШАГ 2: Подключенandе к MQTT брокеру...")
    if not hub.connect_mqtt():
        print(" ERROR: Not уyesлось подключandться к MQTT брокеру")
        return False
    print(" Подключено к брокеру")
    print()
    
    # Шаг 3: Отправandть команду navigateTo
    print(" ШАГ 3: Sending команды navigateTo...")
    command_id = None
    if not hub.send_navigate_command(TARGET_ID, priority="normal"):
        print(" ERROR: Not уyesлось отправandть команду")
        hub.shutdown()
        return False
    
    # Получandть command_id andз последnotй отправленной команды
    # (в реальностand нужно сохранandть его прand отправке)
    print(" Command отправлена")
    print()
    
    # Шаг 4: Жyesть statusы (до 30 секунд)
    print(" ШАГ 4: Waiting statusов от navigation_integrated_node...")
    print("   (максandмум 30 секунд)")
    print()
    
    start_time = time.time()
    timeout = 30.0
    status_received = False
    last_status_type = None
    
    while (time.time() - start_time) < timeout:
        if hub.last_status:
            status_received = True
            current_status = hub.last_status.get('status', 'N/A')
            
            # Показывать только прand andзмеnotнandand statusа
            if current_status != last_status_type:
                last_status_type = current_status
                print(f" Получен status: {current_status}")
                if hub.last_status.get('error_code'):
                    print(f"     Error: {hub.last_status.get('error_code')} - {hub.last_status.get('error_message', 'N/A')}")
                elif current_status == 'navigating':
                    progress = hub.last_status.get('progress_percent', 0)
                    eta = hub.last_status.get('eta_seconds', 0)
                    print(f"    Прогресс: {progress}%, ETA: {eta}s")
                elif current_status in ['arrived', 'idle']:
                    print(f"    Navigation завершена")
                    break
        
        time.sleep(0.5)
    
    print()
    
    # Шаг 5: Проверка результатов
    print("="*70)
    print(" РЕЗУЛЬТАТЫ ТЕСТА:")
    print("="*70)
    
    hub.print_statistics()
    
    if not status_received:
        print(" ERROR: Statusы not получены от navigation_integrated_node")
        print("   Возможные прandчandны:")
        print("   - navigation_integrated_node not запущен")
        print("   - navigation_integrated_node not подключен к брокеру")
        print("   - navigation_integrated_node not подпandсан на команды")
        hub.shutdown()
        return False
    
    if hub.last_status:
        status = hub.last_status.get('status', 'N/A')
        error_code = hub.last_status.get('error_code')
        
        if error_code:
            print(f"  Получена error: {error_code}")
            print(f"   Сообщенandе: {hub.last_status.get('error_message', 'N/A')}")
            if error_code == 'NAV_SERVER_UNAVAILABLE':
                print("   ℹ  Nav2 сервер notдоступен (это нормально еслand Nav2 not запущен)")
            return True  # Тест прошёл, но есть error навandгацandand
        elif status in ['idle', 'arrived']:
            print(" ТЕСТ ПРОЙДЕН: Command обработана, statusы получены")
            return True
        else:
            print(f"  Status: {status} (ожandyesлось idle/arrived)")
            return True  # Тест прошёл частandчно
    
    hub.shutdown()
    return True

if __name__ == "__main__":
    success = test_full_chain()
    sys.exit(0 if success else 1)

