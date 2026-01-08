# Роль navigation_integrated_node в системе навигации

## Краткий ответ

**Да, `navigation_integrated_node` должен быть запущен, если вы используете MQTT для отправки команд навигации.**

**Нет, без него можно работать, но только через альтернативные способы отправки команд.**

---

## Что делает navigation_integrated_node?

`navigation_integrated_node` — это **мост между MQTT и Nav2**, который выполняет следующие функции:

### 1. Обработка MQTT команд
- **Подписывается** на топик: `aroc/robot/{robot_id}/commands/navigateTo`
- **Получает** команды навигации в формате:
  ```json
  {
    "command_id": "uuid-v4",
    "timestamp": "ISO-8601",
    "target_id": "position_A",
    "priority": "normal"
  }
  ```

### 2. Преобразование target_id → координаты
- Получает координаты позиции из **position_registry** (локальный YAML файл)
- **Примечание:** В новой архитектуре должен получать из robot_service API

### 3. Отправка целей в Nav2
- Создает `NavigateToPose.Goal` с координатами
- Отправляет цель в Nav2 Action Server через ROS2 Action Client
- Обрабатывает feedback и результаты от Nav2

### 4. Публикация статуса в MQTT
- Публикует статус навигации в топик: `aroc/robot/{robot_id}/status/navigation`
- Формат:
  ```json
  {
    "command_id": "uuid-v4",
    "status": "navigating",  // или "arrived", "error", "idle"
    "target_id": "position_A",
    "error_code": null,
    "error_message": null
  }
  ```

---

## Когда navigation_integrated_node НЕ нужен?

### Вариант 1: Использование robot_service API напрямую

Если вы используете **robot_service API** для запуска навигации:

```bash
curl -X POST "http://localhost:8110/robot_positions/run?position_id=Shampoo"
```

В этом случае `robot_service` сам обрабатывает команду и отправляет цель в Nav2. `navigation_integrated_node` не нужен.

**Однако:** `robot_service` может использовать `navigation_integrated_node` внутри себя, или иметь свой собственный механизм отправки команд в Nav2.

### Вариант 2: Прямая работа с Nav2 через ROS2

Можно отправлять цели напрямую в Nav2 через ROS2 Action Client:

```python
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient

# Создать Action Client
action_client = ActionClient(node, NavigateToPose, 'navigate_to_pose')

# Создать цель
goal = NavigateToPose.Goal()
goal.pose.header.frame_id = "map"
goal.pose.pose.position.x = 2.56
goal.pose.pose.position.y = -3.25
goal.pose.pose.orientation.w = 1.0

# Отправить цель
action_client.send_goal_async(goal)
```

В этом случае `navigation_integrated_node` не нужен, но вы теряете:
- Обработку MQTT команд
- Публикацию статуса в MQTT
- Управление состоянием навигации

---

## Когда navigation_integrated_node ОБЯЗАТЕЛЕН?

### Сценарий 1: Использование MQTT для команд навигации

Если вы отправляете команды навигации через **MQTT** (как в вашем тестовом скрипте):

```python
mqtt_client.send_navigate_command(target_id="Shampoo", priority="normal")
```

**Без `navigation_integrated_node`:**
- ❌ Команды будут отправлены в MQTT топик
- ❌ Но никто не будет их обрабатывать
- ❌ Nav2 не получит цели
- ❌ Статус не будет публиковаться обратно в MQTT

**С `navigation_integrated_node`:**
- ✅ Команды обрабатываются из MQTT
- ✅ Цели отправляются в Nav2
- ✅ Статус публикуется обратно в MQTT

### Сценарий 2: Интеграция с внешними системами через MQTT

Если внешние системы (AE.HUB, другие сервисы) отправляют команды через MQTT, то `navigation_integrated_node` обязателен.

---

## Архитектура системы

```
┌─────────────┐
│  MQTT       │
│  Commands   │
└──────┬──────┘
       │
       │ aroc/robot/{id}/commands/navigateTo
       ▼
┌─────────────────────────┐
│ navigation_integrated_  │  ← ОБЯЗАТЕЛЕН для MQTT
│ node                     │
│ - Получает MQTT команды  │
│ - Запрашивает координаты │
│ - Отправляет в Nav2      │
└──────┬──────────────────┘
       │
       │ NavigateToPose.Goal
       ▼
┌─────────────┐
│    Nav2     │
│ Action      │
│ Server      │
└──────┬──────┘
       │
       │ /cmd_vel
       ▼
┌─────────────┐
│ base_       │
│ controller  │
└─────────────┘

АЛЬТЕРНАТИВНЫЙ ПУТЬ (без navigation_integrated_node):

┌─────────────┐
│ robot_      │
│ service     │
│ API         │
└──────┬──────┘
       │
       │ POST /robot_positions/run
       │ (может использовать navigation_integrated_node внутри,
       │  или иметь свой механизм)
       ▼
┌─────────────┐
│    Nav2     │
└─────────────┘
```

---

## Рекомендации

### Для вашей системы (AE.HUB MVP):

1. **Если используете MQTT команды** → `navigation_integrated_node` **ОБЯЗАТЕЛЕН**
2. **Если используете только robot_service API** → проверьте, использует ли `robot_service` `navigation_integrated_node` внутри
3. **Для тестирования** → можно работать без него, отправляя цели напрямую в Nav2

### Текущая ситуация:

В вашем тестовом скрипте команды отправляются через MQTT:
```python
mqtt_client.send_navigate_command(target_id="Shampoo", priority="normal")
```

**Поэтому `navigation_integrated_node` должен быть запущен**, иначе команды не будут обработаны.

---

## Как запустить navigation_integrated_node?

### Способ 1: Через скрипт (рекомендуется)

```bash
cd /home/boris/ros2_ws
./scripts/start_navigation_node.sh
```

### Способ 2: Вручную

```bash
cd /home/boris/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

python3 -m aehub_navigation.navigation_integrated_node \
  --ros-args \
  -p robot_id:=robot_001 \
  -p config_service_url:=http://localhost:7900 \
  -p config_service_api_key:=tR-UZ2j2KutE6OYlEGbsx0h5qe071L-gC5kd1hHKfw4
```

### Способ 3: Через launch файл

```bash
ros2 launch aehub_navigation aehub_navigation.launch.py \
  robot_id:=robot_001 \
  config_service_url:=http://localhost:7900 \
  config_service_api_key:=tR-UZ2j2KutE6OYlEGbsx0h5qe071L-gC5kd1hHKfw4
```

---

## Итог

- **Для MQTT команд:** `navigation_integrated_node` **ОБЯЗАТЕЛЕН**
- **Для robot_service API:** зависит от реализации (скорее всего тоже нужен)
- **Для прямого управления Nav2:** не нужен, но теряете MQTT интеграцию

**Рекомендация:** Запускайте `navigation_integrated_node` всегда, если планируете использовать MQTT для команд навигации.

