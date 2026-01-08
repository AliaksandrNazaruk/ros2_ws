# Алгоритм перемещения робота из точки A в точку B через Nav2 Testing Server

## Обзор

Данный документ описывает последовательность шагов для перемещения робота из точки A в точку B с использованием `nav2_test_server` и связанных компонентов системы.

---

## Предварительные условия

1. **Nav2 процессы запущены** (через `ProcessManager` или вручную)
2. **robot_service API доступен** (порт 8110)
3. **MQTT брокер подключен** (если используется MQTT для навигации)
4. **ROS2 демон работает**
5. **Позиции созданы** в robot_service API

---

## Последовательность шагов

### Шаг 1: Подготовка системы

#### 1.1. Запуск Nav2 и base_controller

```python
# Через ProcessManager (программно)
from nav2_test_server.process_manager import ProcessManager

manager = ProcessManager()
manager.start_nav2(
    map_file="/path/to/map.yaml",
    symovo_endpoint="https://192.168.1.100",
    amr_id=15,
    use_scan_converter=True
)
```

**Или через API:**
```bash
curl -X POST http://localhost:8000/api/process/nav2/start \
  -H "Content-Type: application/json" \
  -d '{
    "map_file": "/path/to/map.yaml",
    "symovo_endpoint": "https://192.168.1.100",
    "amr_id": 15,
    "use_scan_converter": true
  }'
```

**Что происходит:**
- Запускается launch-файл `symovo_nav2.launch.py`
- Инициализируются узлы Nav2 (planner, controller, recovery)
- Запускается `base_controller_node` для управления базой робота
- При необходимости запускается `symovo_scan_converter` для преобразования данных лидара

#### 1.2. Проверка статуса процессов

```bash
curl http://localhost:8000/api/process/nav2/status
```

**Ожидаемый результат:**
```json
{
  "name": "nav2",
  "status": "running",
  "running": true,
  "pid": 12345
}
```

---

### Шаг 2: Получение координат точек A и B

#### 2.1. Получение списка позиций из robot_service API

```python
from nav2_test_server.robot_position_client import RobotPositionClient

client = RobotPositionClient(base_url="http://localhost:8110")
positions = await client.list_positions()
```

**Или через API напрямую:**
```bash
curl http://localhost:8110/robot_positions/list
```

**Ответ:**
```json
[
  {
    "id": "4440d3b6",
    "name": "position_A",
    "params": {
      "location": {
        "x_m": 1.2,
        "y_m": 3.4,
        "theta_deg": 0.0,
        "map_id": 0
      }
    }
  },
  {
    "id": "1f56e527",
    "name": "position_B",
    "params": {
      "location": {
        "x_m": 5.6,
        "y_m": 7.8,
        "theta_deg": 90.0,
        "map_id": 0
      }
    }
  }
]
```

#### 2.2. Извлечение координат точек A и B

Клиент автоматически нормализует данные:
- `x_m`, `y_m` → `x`, `y` (метры)
- `theta_deg` → `theta` (радианы)

---

### Шаг 3: Выбор метода навигации

Существует два основных способа запуска навигации:

#### Вариант A: Через robot_service API (рекомендуется)

#### 3A.1. Запуск навигации к позиции

```python
# Используя RobotPositionClient
result = await client.run_position("position_B")
```

**Или через API напрямую:**
```bash
curl -X POST "http://localhost:8110/robot_positions/run?position_id=position_B"
```

**Ответ:**
```json
{
  "success": true,
  "task_id": "task_12345",
  "result": "navigating",
  "detail": "Navigation started",
  "error": null
}
```

**Что происходит:**
- robot_service получает координаты позиции из базы данных
- Формируется команда навигации
- Команда отправляется в систему навигации (через ROS2 или MQTT)
- Возвращается `task_id` для отслеживания статуса

#### 3A.2. Мониторинг статуса навигации

```bash
# Получить статус задачи
curl "http://localhost:8110/api/v1/robot/tasks/status/{task_id}"

# Или получить текущую задачу
curl "http://localhost:8110/api/v1/robot/tasks/current"
```

---

#### Вариант B: Через MQTT (альтернативный способ)

#### 3B.1. Отправка команды навигации через MQTT

```python
from nav2_test_server.mqtt_client import MQTTTestClient
from nav2_test_server.config import Config

config = Config()
mqtt_client = MQTTTestClient(config)
mqtt_client.connect()

command_id = mqtt_client.send_navigate_command(
    target_id="position_B",
    priority="normal"
)
```

**Что происходит:**
- Генерируется уникальный `command_id` (UUID)
- Формируется JSON-команда:
  ```json
  {
    "command_id": "uuid-here",
    "timestamp": "2024-01-01T12:00:00Z",
    "target_id": "position_B",
    "priority": "normal"
  }
  ```
- Команда публикуется в MQTT топик: `aroc/robot/{robot_id}/commands/navigateTo`
- QoS=1 (гарантированная доставка)

#### 3B.2. Получение статуса через MQTT

**Подписка на топик статуса:**
```
aroc/robot/{robot_id}/status/navigation
```

**Формат сообщения:**
```json
{
  "command_id": "uuid-here",
  "status": "navigating",  // или "arrived", "error", "idle"
  "error_code": null,
  "error_message": null,
  "timestamp": "2024-01-01T12:00:00Z"
}
```

**Статусы:**
- `idle` - Робот не выполняет навигацию
- `navigating` - Робот движется к цели
- `arrived` - Робот достиг цели
- `error` - Произошла ошибка

---

### Шаг 4: Обработка навигации в системе робота

#### 4.1. Получение команды навигации

**Если через robot_service API:**
- robot_service получает запрос на запуск навигации
- Извлекает координаты позиции из базы данных
- Формирует цель для Nav2

**Если через MQTT:**
- MQTT брокер доставляет команду в топик `commands/navigateTo`
- Navigation node подписывается на этот топик
- Получает команду и извлекает `target_id`

#### 4.2. Преобразование target_id в координаты

```python
# Navigation node получает target_id="position_B"
# Запрашивает координаты из robot_service API
position = await robot_client.get_position("position_B")

# Извлекает координаты
x = position['x']  # 5.6
y = position['y']  # 7.8
theta = position['theta']  # 1.57 (90 градусов в радианах)
```

#### 4.3. Формирование цели для Nav2

```python
# Создание PoseStamped для Nav2
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose

goal = NavigateToPose.Goal()
goal.pose.header.frame_id = "map"
goal.pose.header.stamp = rclpy.time.Time().to_msg()
goal.pose.pose.position.x = x
goal.pose.pose.position.y = y
goal.pose.pose.orientation = quaternion_from_euler(0, 0, theta)
```

#### 4.4. Отправка цели в Nav2 Action Server

```python
# Navigation node отправляет цель в Nav2
nav2_action_client.send_goal_async(goal)
```

**Что происходит в Nav2:**
1. **Planner** вычисляет путь от текущей позиции до цели
2. **Controller** генерирует команды управления (velocity commands)
3. **Recovery behaviors** обрабатывают ситуации, когда робот застрял
4. Команды отправляются в `base_controller` через топик `/cmd_vel`

---

### Шаг 5: Выполнение движения

#### 5.1. base_controller получает команды

**Топик:** `/cmd_vel` (geometry_msgs/msg/Twist)

**Формат:**
```python
{
  "linear": {"x": 0.5, "y": 0.0, "z": 0.0},
  "angular": {"x": 0.0, "y": 0.0, "z": 0.2}
}
```

#### 5.2. Преобразование команд в управление базой

- `base_controller` получает команды от Nav2
- Преобразует их в команды для драйвера робота (через Symovo API)
- Отправляет команды на физическую базу робота

#### 5.3. Обратная связь (Odometry)

**Топик:** `/odom` (nav_msgs/msg/Odometry)

- Робот отправляет данные о своем текущем положении
- Nav2 использует эти данные для коррекции пути
- Обновляется оценка позиции робота

---

### Шаг 6: Мониторинг навигации

#### 6.1. Мониторинг через nav2_test_server

```bash
# Проверка статуса Nav2
curl http://localhost:8000/api/monitor/nav2

# Проверка топиков
curl http://localhost:8000/api/monitor/topics

# Проверка узлов ROS2
curl http://localhost:8000/api/monitor/nodes
```

#### 6.2. Отслеживание прогресса

**Топики для мониторинга:**
- `/odom` - текущая позиция робота
- `/navigate_to_pose/_action/status` - статус Nav2 action
- `/navigate_to_pose/_action/feedback` - обратная связь от Nav2
- `/navigate_to_pose/_action/result` - результат навигации

#### 6.3. Проверка достижения цели

Nav2 отправляет результат, когда:
- ✅ **SUCCEEDED** - робот достиг цели
- ❌ **ABORTED** - навигация прервана (препятствие, ошибка)
- ⏸️ **CANCELED** - навигация отменена пользователем

---

### Шаг 7: Завершение навигации

#### 7.1. Получение результата

**Через robot_service API:**
```bash
curl "http://localhost:8110/api/v1/robot/tasks/status/{task_id}"
```

**Через MQTT:**
```json
{
  "command_id": "uuid-here",
  "status": "arrived",
  "error_code": null,
  "error_message": null
}
```

#### 7.2. Обновление истории навигации

`nav2_test_server` автоматически:
- Записывает команду в историю (`DataCollector`)
- Обновляет статус при получении уведомлений
- Вычисляет метрики (время выполнения, успешность)

#### 7.3. Логирование результата

```python
# В DataCollector
{
  "command_id": "uuid-here",
  "target_id": "position_B",
  "status": "arrived",
  "duration_seconds": 45.2,
  "timestamp": "2024-01-01T12:00:00Z"
}
```

---

## Диаграмма последовательности

```
┌─────────────┐    ┌──────────────┐    ┌─────────────┐    ┌─────────────┐    ┌──────────┐
│   Client    │    │nav2_test_    │    │robot_service│    │Navigation   │    │   Nav2   │
│             │    │server        │    │API          │    │Node         │    │          │
└──────┬──────┘    └──────┬───────┘    └──────┬──────┘    └──────┬──────┘    └────┬─────┘
       │                  │                    │                   │               │
       │ 1. Запрос       │                    │                   │               │
       │ навигации       │                    │                   │               │
       ├─────────────────>│                    │                   │               │
       │                  │                    │                   │               │
       │                  │ 2. Получить        │                   │               │
       │                  │ координаты         │                   │               │
       │                  ├───────────────────>│                   │               │
       │                  │                    │                   │               │
       │                  │ 3. Координаты      │                   │               │
       │                  │<───────────────────┤                   │               │
       │                  │                    │                   │               │
       │                  │ 4. Отправить       │                   │               │
       │                  │ команду            │                   │               │
       │                  ├───────────────────────────────────────>│               │
       │                  │                    │                   │               │
       │                  │                    │ 5. Получить       │               │
       │                  │                    │ координаты        │               │
       │                  │                    ├──────────────────>│               │
       │                  │                    │                   │               │
       │                  │                    │ 6. Координаты    │               │
       │                  │                    │<──────────────────┤               │
       │                  │                    │                   │               │
       │                  │                    │ 7. Отправить цель │               │
       │                  │                    ├───────────────────────────────────>│
       │                  │                    │                   │               │
       │                  │                    │                   │ 8. Планирование│
       │                  │                    │                   │<──────────────┤
       │                  │                    │                   │               │
       │                  │                    │                   │ 9. Команды     │
       │                  │                    │                   │<──────────────┤
       │                  │                    │                   │               │
       │                  │                    │ 10. Статус       │               │
       │                  │                    │<──────────────────┤               │
       │                  │                    │                   │               │
       │                  │ 11. Результат     │                   │               │
       │                  │<───────────────────┤                   │               │
       │                  │                    │                   │               │
       │ 12. Результат    │                    │                   │               │
       │<─────────────────┤                    │                   │               │
       │                  │                    │                   │               │
```

---

## Обработка ошибок

### Ошибка: Позиция не найдена

**Причина:** `target_id` не существует в robot_service API

**Решение:**
1. Проверить список позиций: `GET /robot_positions/list`
2. Убедиться, что позиция создана
3. Использовать правильный `target_id`

### Ошибка: Nav2 не запущен

**Причина:** Процессы Nav2 не запущены

**Решение:**
```bash
# Проверить статус
curl http://localhost:8000/api/process/nav2/status

# Запустить Nav2
curl -X POST http://localhost:8000/api/process/nav2/start
```

### Ошибка: MQTT не подключен

**Причина:** MQTT клиент не подключен к брокеру

**Решение:**
1. Проверить настройки MQTT в `.env`
2. Проверить доступность брокера
3. Проверить логи: `GET /api/monitor/health`

### Ошибка: Навигация прервана

**Причины:**
- Препятствие на пути
- Недостаточно места для маневра
- Ошибка планировщика

**Решение:**
1. Проверить карту и препятствия
2. Проверить параметры Nav2 (costmaps, planner)
3. Использовать recovery behaviors

---

## Пример полного цикла

```python
import asyncio
from nav2_test_server.robot_position_client import RobotPositionClient
from nav2_test_server.process_manager import ProcessManager

async def navigate_from_a_to_b():
    # 1. Запуск Nav2
    manager = ProcessManager()
    if not manager.start_nav2():
        print("Failed to start Nav2")
        return
    
    # 2. Получение позиций
    client = RobotPositionClient()
    position_a = await client.get_position("position_A")
    position_b = await client.get_position("position_B")
    
    print(f"Point A: ({position_a['x']}, {position_a['y']})")
    print(f"Point B: ({position_b['x']}, {position_b['y']})")
    
    # 3. Запуск навигации
    result = await client.run_position("position_B")
    task_id = result.get('task_id')
    
    print(f"Navigation started, task_id: {task_id}")
    
    # 4. Мониторинг (упрощенный)
    # В реальности нужно опрашивать статус через API
    
    await client.close()

# Запуск
asyncio.run(navigate_from_a_to_b())
```

---

## Заключение

Алгоритм перемещения робота включает:
1. ✅ Подготовку системы (запуск Nav2)
2. ✅ Получение координат точек A и B
3. ✅ Выбор метода навигации (robot_service API или MQTT)
4. ✅ Обработку команды в системе робота
5. ✅ Выполнение движения через Nav2 и base_controller
6. ✅ Мониторинг прогресса
7. ✅ Завершение и логирование результата

Все компоненты работают вместе для обеспечения надежной навигации робота между точками.

