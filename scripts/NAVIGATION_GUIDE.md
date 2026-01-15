# Руководство по навигации робота

## ✅ Теперь можно отправлять робота в любую точку на карте!

После исправления endpoint для движения робота, система навигации полностью функциональна.

## Способы отправки робота

### 1. Использование предопределенных позиций

В файле `src/aehub_navigation/config/positions.yaml` есть предопределенные позиции:

- `position_A`: x=0.5, y=0.5
- `position_B`: x=1.0, y=1.0
- `position_C`: x=1.5, y=1.5
- `position_D`: x=2.0, y=0.5
- `position_E`: x=0.5, y=2.0
- `test_position`: x=0.51, y=-0.24

**Использование:**
```bash
python3 scripts/send_robot_to_position.py position_A
python3 scripts/send_robot_to_position.py position_B
```

### 2. Использование произвольных координат

Можно отправить робота в любую точку на карте, указав координаты:

```bash
# С указанием x, y, yaw (в радианах)
python3 scripts/send_robot_to_position.py 1.5 1.5 0.0

# Только x, y (yaw = 0)
python3 scripts/send_robot_to_position.py 2.0 0.5

# Пример: поворот на 90 градусов (1.57 радиан)
python3 scripts/send_robot_to_position.py 1.0 1.0 1.57
```

### 3. Интерактивный режим

Запустите скрипт без аргументов для просмотра доступных опций:

```bash
python3 scripts/send_robot_to_position.py
```

## Проверка текущей позиции

Перед отправкой робота можно проверить его текущую позицию:

```bash
# Через odometry
ros2 topic echo /odom --once

# Через AMCL (более точная локализация)
ros2 topic echo /amcl_pose --once
```

## Мониторинг навигации

Во время навигации скрипт показывает:
- Текущую позицию робота
- Оставшееся расстояние до цели
- Время навигации

## Отмена навигации

Для отмены текущей навигации нажмите `Ctrl+C` или используйте:

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: 'map'}}}" --cancel
```

## Требования

Для работы навигации должны быть запущены:

1. ✅ **base_controller** - для движения робота
2. ✅ **Nav2 stack** - для планирования пути
3. ✅ **map_server** - для загрузки карты
4. ✅ **AMCL** - для локализации
5. ✅ **navigation_integrated_node** - для интеграции с MQTT (опционально)

## Запуск как systemd service (рекомендуется)

Это исключает случайный запуск нескольких `ros2 launch ...` из разных терминалов и появление дублей в ROS графе.

### Установка

```bash
./scripts/install_aehub_nav2_service.sh
sudo nano /etc/aehub_nav2.env
```

### Управление (start/stop/status/logs)

```bash
./scripts/aehub_nav2_ctl.sh start
./scripts/aehub_nav2_ctl.sh status
./scripts/aehub_nav2_ctl.sh logs
./scripts/aehub_nav2_ctl.sh stop
```

## Примеры использования

### Пример 1: Отправка в предопределенную позицию
```bash
python3 scripts/send_robot_to_position.py position_C
```

### Пример 2: Отправка в произвольную точку
```bash
python3 scripts/send_robot_to_position.py 1.8 1.2 0.785  # 45 градусов
```

### Пример 3: Возврат в исходную позицию
```bash
# Сначала узнайте текущую позицию
ros2 topic echo /amcl_pose --once

# Затем отправьте робота обратно
python3 scripts/send_robot_to_position.py 0.5 0.5 0.0
```

## Добавление новых позиций

Чтобы добавить новую предопределенную позицию, отредактируйте файл:

`src/aehub_navigation/config/positions.yaml`

Добавьте новую позицию в формате:
```yaml
positions:
  my_new_position:
    x: 1.75
    y: 0.75
    theta: 0.0
    description: "My custom position"
```

После этого можно использовать:
```bash
python3 scripts/send_robot_to_position.py my_new_position
```

## Устранение проблем

### Робот не двигается
1. Проверьте, что base_controller запущен и использует исправленный endpoint
2. Проверьте статус: `python3 scripts/check_base_controller_status.py`

### Nav2 не принимает цели
1. Проверьте, что Nav2 запущен: `ros2 node list | grep nav`
2. Проверьте карту: `ros2 topic echo /map --once`
3. Проверьте локализацию: `ros2 topic echo /amcl_pose --once`

### Робот не может достичь цели
1. Убедитесь, что цель находится в свободном пространстве на карте
2. Проверьте, что путь не заблокирован препятствиями
3. Попробуйте другую позицию рядом

## Дополнительные инструменты

- `scripts/test_robot_navigation.py` - базовый тест навигации
- `scripts/diagnose_robot_movement.py` - диагностика движения
- `scripts/comprehensive_movement_debug.sh` - комплексная диагностика
