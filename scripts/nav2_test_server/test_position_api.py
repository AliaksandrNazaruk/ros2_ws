#!/usr/bin/env python3
"""
Тестовый скрипт для проверки API управления позициями
"""

import requests
import json
import sys
from typing import Dict, Any

BASE_URL = "http://localhost:8000"

def print_response(response: requests.Response, title: str = ""):
    """Печатает ответ сервера"""
    print(f"\n{'='*60}")
    if title:
        print(f"{title}")
    print(f"{'='*60}")
    print(f"Status: {response.status_code}")
    try:
        data = response.json()
        print(f"Response: {json.dumps(data, indent=2, ensure_ascii=False)}")
    except:
        print(f"Response: {response.text}")
    print(f"{'='*60}\n")

def test_get_all_positions():
    """Тест получения всех позиций"""
    print("🔍 Тест: Получить все позиции")
    response = requests.get(f"{BASE_URL}/api/positions")
    print_response(response, "GET /api/positions")
    return response.status_code == 200

def test_get_position(position_id: str):
    """Тест получения конкретной позиции"""
    print(f"🔍 Тест: Получить позицию '{position_id}'")
    response = requests.get(f"{BASE_URL}/api/positions/{position_id}")
    print_response(response, f"GET /api/positions/{position_id}")
    return response.status_code == 200

def test_create_position(position_id: str, x: float, y: float, theta: float, description: str = ""):
    """Тест создания позиции"""
    print(f"➕ Тест: Создать позицию '{position_id}'")
    data = {
        "position_id": position_id,
        "x": x,
        "y": y,
        "theta": theta,
        "description": description
    }
    response = requests.post(
        f"{BASE_URL}/api/positions",
        json=data,
        headers={"Content-Type": "application/json"}
    )
    print_response(response, f"POST /api/positions")
    return response.status_code == 201

def test_update_position(position_id: str, x: float, y: float, theta: float, description: str = ""):
    """Тест обновления позиции"""
    print(f"🔄 Тест: Обновить позицию '{position_id}'")
    data = {
        "position_id": position_id,
        "x": x,
        "y": y,
        "theta": theta,
        "description": description
    }
    response = requests.post(
        f"{BASE_URL}/api/positions",
        json=data,
        headers={"Content-Type": "application/json"}
    )
    print_response(response, f"POST /api/positions (update)")
    return response.status_code == 201

def test_delete_position(position_id: str):
    """Тест удаления позиции"""
    print(f"🗑️  Тест: Удалить позицию '{position_id}'")
    response = requests.delete(f"{BASE_URL}/api/positions/{position_id}")
    print_response(response, f"DELETE /api/positions/{position_id}")
    return response.status_code == 200

def test_invalid_position():
    """Тест с невалидными данными"""
    print("❌ Тест: Создать позицию с невалидными данными")
    data = {
        "position_id": "",  # Пустой ID
        "x": 10000.0,  # Превышает лимит
        "y": 20.0,
        "theta": 0.0
    }
    response = requests.post(
        f"{BASE_URL}/api/positions",
        json=data,
        headers={"Content-Type": "application/json"}
    )
    print_response(response, "POST /api/positions (invalid data)")
    return response.status_code == 422  # Validation error

def test_nonexistent_position():
    """Тест получения несуществующей позиции"""
    print("❌ Тест: Получить несуществующую позицию")
    response = requests.get(f"{BASE_URL}/api/positions/nonexistent_position_12345")
    print_response(response, "GET /api/positions/nonexistent_position_12345")
    return response.status_code == 404

def main():
    """Основная функция тестирования"""
    print("🚀 Начало тестирования API управления позициями\n")
    
    # Проверка доступности сервера
    try:
        response = requests.get(f"{BASE_URL}/api/monitor/health", timeout=2)
        if response.status_code != 200:
            print(f"❌ Сервер недоступен. Status: {response.status_code}")
            sys.exit(1)
    except requests.exceptions.RequestException as e:
        print(f"❌ Не удалось подключиться к серверу: {e}")
        print(f"   Убедитесь, что сервер запущен на {BASE_URL}")
        sys.exit(1)
    
    print("✅ Сервер доступен\n")
    
    results = []
    
    # 1. Получить все позиции (начальное состояние)
    results.append(("GET all positions (initial)", test_get_all_positions()))
    
    # 2. Создать новую позицию
    results.append(("CREATE position_F", test_create_position(
        "position_F", 10.0, 20.0, 1.57, "Новая позиция F"
    )))
    
    # 3. Получить созданную позицию
    results.append(("GET position_F", test_get_position("position_F")))
    
    # 4. Создать еще одну позицию
    results.append(("CREATE position_G", test_create_position(
        "position_G", 15.0, 25.0, 0.0, "Позиция G"
    )))
    
    # 5. Получить все позиции (после добавления)
    results.append(("GET all positions (after add)", test_get_all_positions()))
    
    # 6. Обновить существующую позицию
    results.append(("UPDATE position_F", test_update_position(
        "position_F", 12.0, 22.0, 3.14, "Обновленная позиция F"
    )))
    
    # 7. Проверить обновление
    results.append(("GET position_F (after update)", test_get_position("position_F")))
    
    # 8. Удалить позицию
    results.append(("DELETE position_G", test_delete_position("position_G")))
    
    # 9. Проверить удаление (должна быть 404)
    results.append(("GET position_G (after delete)", test_get_position("position_G")))
    
    # 10. Получить все позиции (после удаления)
    results.append(("GET all positions (after delete)", test_get_all_positions()))
    
    # 11. Тест невалидных данных
    results.append(("INVALID data test", test_invalid_position()))
    
    # 12. Тест несуществующей позиции
    results.append(("NONEXISTENT position test", test_nonexistent_position()))
    
    # Итоги
    print("\n" + "="*60)
    print("📊 РЕЗУЛЬТАТЫ ТЕСТИРОВАНИЯ")
    print("="*60)
    
    passed = sum(1 for _, result in results if result)
    total = len(results)
    
    for test_name, result in results:
        status = "✅ PASS" if result else "❌ FAIL"
        print(f"{status}: {test_name}")
    
    print("="*60)
    print(f"Всего тестов: {total}")
    print(f"Успешно: {passed}")
    print(f"Провалено: {total - passed}")
    print("="*60)
    
    if passed == total:
        print("\n🎉 Все тесты пройдены успешно!")
        return 0
    else:
        print(f"\n⚠️  {total - passed} тест(ов) провалено")
        return 1

if __name__ == "__main__":
    sys.exit(main())

