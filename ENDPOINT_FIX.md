# Исправление API Endpoint для движения робота

## Проблема
Робот не двигался, потому что base_controller использовал неправильный API endpoint.

## Решение
Исправлен endpoint с `/v0/amr/{id}/move/speed` на `/v0/agv/{id}/move/speed`

## Изменения

### 1. `src/base_controller/src/async_driver_client.cpp`
- Строка 154: `/v0/amr/` → `/v0/agv/`
- Строка 316: `/v0/amr/` → `/v0/agv/` (SSL версия)

### 2. `src/base_controller/include/base_controller/async_driver_client.hpp`
- Обновлен комментарий с правильным endpoint

### 3. `scripts/test_symovo_api_direct.sh`
- Обновлен для использования правильного endpoint

## Проверка

Правильный endpoint возвращает **202 Accepted**:
```bash
curl -k -X PUT "https://192.168.1.100/v0/agv/15/move/speed" \
     -H 'Content-Type: application/json' \
     -d '{"speed": 0.2, "angular_speed": 0.0, "duration": 0.05}'
```

## Пересборка и перезапуск

```bash
# 1. Пересобрать base_controller
cd /home/boris/ros2_ws
colcon build --packages-select base_controller
source install/setup.bash

# 2. Перезапустить base_controller
bash scripts/restart_base_controller.sh

# 3. Протестировать движение
python3 scripts/test_robot_command.py
```

## Результат
После исправления робот должен успешно получать команды движения через Symovo API.
