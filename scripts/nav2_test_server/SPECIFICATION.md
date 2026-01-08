# Полная спецификация и инструкция по использованию Nav2 Testing Server

## Содержание

1. [Обзор модуля](#обзор-модуля)
2. [Архитектура системы](#архитектура-системы)
3. [Установка и настройка](#установка-и-настройка)
4. [Конфигурация](#конфигурация)
5. [API Endpoints](#api-endpoints)
6. [ProcessManager - Управление процессами](#processmanager---управление-процессами)
7. [Примеры использования](#примеры-использования)
8. [Устранение неполадок](#устранение-неполадок)

---

## Обзор модуля

**Nav2 Testing Server** — это FastAPI-сервер для мониторинга, тестирования и анализа системы навигации Nav2 в ROS2. Модуль предоставляет REST API для управления процессами Nav2 и base_controller, мониторинга ROS2-топиков и узлов, отправки команд навигации через MQTT, управления позициями и анализа производительности системы.

### Основные возможности

- **Мониторинг ROS2**: Мониторинг топиков, узлов, Nav2 action server и TF-трансформаций в реальном времени
- **Управление процессами**: Запуск, остановка и перезапуск процессов Nav2 и base_controller через API
- **MQTT интеграция**: Отправка команд навигации и получение статусов через MQTT
- **Управление позициями**: Создание, чтение, обновление и удаление позиций для навигации
- **Сбор данных**: Автоматический сбор истории навигации и метрик
- **Анализ**: Анализ производительности навигации и системных метрик
- **Web Dashboard**: Интерактивный веб-интерфейс для визуализации

---

## Архитектура системы

### Компоненты модуля

#### 1. **ProcessManager** (`process_manager.py`)
Класс для управления жизненным циклом ROS2-процессов (Nav2 и base_controller).

**Основные функции:**
- Запуск/остановка процессов Nav2 и base_controller
- Мониторинг статуса процессов
- Управление логами процессов
- Очистка зависших процессов

**Класс ProcessStatus:**
```python
class ProcessStatus(str, Enum):
    STOPPED = "stopped"      # Процесс остановлен
    STARTING = "starting"    # Процесс запускается
    RUNNING = "running"      # Процесс работает
    STOPPING = "stopping"    # Процесс останавливается
    ERROR = "error"          # Ошибка в процессе
```

#### 2. **ROS2Monitor** (`ros2_monitor.py`)
Мониторинг ROS2-топиков, узлов, Nav2 action server и TF-трансформаций.

#### 3. **MQTTTestClient** (`mqtt_client.py`)
Клиент для отправки команд навигации и получения статусов через MQTT.

#### 4. **DataCollector** (`data_collector.py`)
Сбор и хранение данных о навигации.

#### 5. **NavigationAnalyzer** (`analysis.py`)
Анализ данных навигации и генерация статистики.

#### 6. **RobotPositionClient** (`robot_position_client.py`)
Клиент для работы с API локального сервера позиций (robot_service). 

**Примечание:** Этот клиент доступен для использования в других модулях, но endpoints для управления позициями были удалены из nav2_test_server, так как они избыточны. Используйте robot_service API напрямую.

**Основные функции (для использования в других модулях):**
- Получение списка позиций
- Получение конкретной позиции
- Создание/обновление позиции
- Удаление позиции
- Запуск навигации к позиции

### Структура сервисов

```
AppContext
├── ROS2Service (ROS2Monitor)
├── MQTTService (MQTTTestClient)
├── ProcessService (ProcessManager)
├── DataCollector
└── NavigationAnalyzer

Note: Position management is handled directly by robot_service API, not through this server.
```

---

## Установка и настройка

### Требования

- **ROS2 Jazzy** (установлен в `/opt/ros/jazzy/`)
- **Python 3.12+**
- **ROS2 workspace** собран и установлен (`/home/boris/ros2_ws/install/`)

### Установка зависимостей

```bash
cd /home/boris/ros2_ws/scripts
pip install -r requirements_test_server.txt
```

### Запуск сервера

#### Способ 1: Использование скрипта (рекомендуется)

```bash
cd /home/boris/ros2_ws
./scripts/start_test_server.sh
```

#### Способ 2: Ручной запуск

```bash
# 1. Подготовка окружения ROS2
source /opt/ros/jazzy/setup.bash
source /home/boris/ros2_ws/install/setup.bash

# 2. Переход в директорию сервера
cd /home/boris/ros2_ws/scripts/nav2_test_server

# 3. Запуск сервера
python3 -m uvicorn main:app \
  --host 0.0.0.0 \
  --port 8000 \
  --proxy-headers \
  --forwarded-allow-ips="*"
```

### Доступ к сервисам

После запуска сервера доступны:

- **Web Dashboard**: http://localhost:8000/ или http://localhost:8000/dashboard
- **API Documentation (Swagger)**: http://localhost:8000/docs
- **API Documentation (ReDoc)**: http://localhost:8000/redoc
- **Monitoring Page**: http://localhost:8000/monitoring

---

## Конфигурация

### Файл `.env`

Создайте файл `.env` в директории `nav2_test_server/` для настройки параметров:

```bash
# MQTT настройки
MQTT_BROKER=82.165.177.194
MQTT_PORT=8883
MQTT_USERNAME=bridge_user
MQTT_PASSWORD=your_password
MQTT_USE_TLS=true
MQTT_TLS_INSECURE=true

# Идентификация робота
ROBOT_ID=robot_001

# Config Service (опционально)
CONFIG_SERVICE_URL=http://localhost:7900
CONFIG_SERVICE_API_KEY=your_api_key

# Настройки сервера
SERVER_HOST=0.0.0.0
SERVER_PORT=8000

# Robot Service API (for position management)
ROBOT_SERVICE_URL=http://localhost:8110
ROBOT_SERVICE_TIMEOUT=10.0

# Настройки данных
HISTORY_SIZE=1000
TOPIC_BUFFER_SIZE=100
```

### Robot Service API

Позиции для навигации управляются через API локального сервера `robot_service` (порт 8110). Это единый источник истины для всех позиций.

**URL сервера:** Настраивается через переменную окружения `ROBOT_SERVICE_URL` (по умолчанию: `http://localhost:8110`)

**API Endpoints robot_service:**
- `GET /api/v1/robot/robot_positions/list` - Получить список всех позиций
- `POST /api/v1/robot/robot_positions/save` - Сохранить/обновить позицию
- `POST /api/v1/robot/robot_positions/delete?position_id=...` - Удалить позицию
- `POST /api/v1/robot/robot_positions/run?position_id=...` - Запустить навигацию к позиции

**Формат позиции:**
- `position_id`: идентификатор позиции (строка)
- `x`, `y`: координаты в метрах (диапазон: -1000.0 до 1000.0)
- `theta`: ориентация в радианах (диапазон: -π до π)
- `description`: описание позиции (максимум 256 символов)

**Примечание:** Локальный файл `config/positions.yaml` больше не используется. Все операции с позициями выполняются через API.

---

## API Endpoints

### Мониторинг

**Примечание:** В текущей версии реализованы базовые endpoints мониторинга. Дополнительные endpoints (навигация, анализ, MQTT) упомянуты в README.md, но еще не реализованы в коде.

#### `GET /api/monitor/health`
Проверка состояния сервера и подключений.

**Ответ:**
```json
{
  "status": "ok",
  "ros2": true,
  "mqtt": true,
  "timestamp": "2024-01-01T12:00:00.000000"
}
```

#### `GET /api/monitor/status`
Статус системы (legacy endpoint).

**Ответ:**
```json
{
  "ros2": true,
  "mqtt": true,
  "timestamp": "2024-01-01T12:00:00.000000"
}
```

#### `GET /api/monitor/nodes`
Получить список всех ROS2-узлов с их publishers, subscribers, services и actions.

**Ответ:**
```json
[
  {
    "name": "nav2_test_server_monitor",
    "namespace": "",
    "publishers": ["/topic1", "/topic2"],
    "subscribers": ["/topic3"],
    "services": ["/service1"],
    "actions": ["/navigate_to_pose"]
  }
]
```

#### `GET /api/monitor/topics`
Получить информацию о всех мониторируемых топиках.

**Ответ:**
```json
{
  "/odom": {
    "name": "/odom",
    "type": "nav_msgs/msg/Odometry",
    "last_message_time": "2024-01-01T12:00:00.000000",
    "message_count": 1234,
    "frequency_hz": 20.5,
    "data": {...}
  }
}
```

**Планируемые endpoints мониторинга (еще не реализованы):**
- `GET /api/monitor/topics/{topic_name}` - Данные конкретного топика
- `GET /api/monitor/nav2` - Статус Nav2 action server
- `GET /api/monitor/tf` - TF-трансформации

### Управление процессами

**Примечание:** В текущей версии реализован только endpoint для получения статуса. Управление процессами (запуск/остановка) доступно через класс `ProcessManager` программно (см. раздел [ProcessManager - Управление процессами](#processmanager---управление-процессами)). API endpoints для управления процессами планируются к реализации.

#### `GET /api/process/nav2/status`
Получить статус процесса Nav2.

**Ответ:**
```json
{
  "name": "nav2",
  "status": "running",
  "running": true,
  "pid": 12345,
  "logs": [
    "[INFO] Nav2 started",
    "[INFO] Waiting for map..."
  ]
}
```

**Возможные значения статуса:**
- `stopped` - Процесс остановлен
- `starting` - Процесс запускается
- `running` - Процесс работает
- `stopping` - Процесс останавливается
- `error` - Ошибка в процессе

**Планируемые endpoints (еще не реализованы):**
- `POST /api/process/nav2/start` - Запустить Nav2 и base_controller
- `POST /api/process/nav2/stop` - Остановить Nav2 и base_controller
- `POST /api/process/nav2/restart` - Перезапустить Nav2 и base_controller
- `POST /api/process/base_controller/start` - Запустить только base_controller
- `POST /api/process/base_controller/stop` - Остановить base_controller
- `POST /api/process/base_controller/restart` - Перезапустить base_controller
- `GET /api/process/base_controller/status` - Получить статус процесса base_controller
- `GET /api/process/status` - Получить статус всех управляемых процессов

**Временное решение:** Используйте класс `ProcessManager` напрямую в Python-скриптах для управления процессами (см. примеры в разделе [Примеры использования](#примеры-использования)).

### Управление позициями

**Примечание:** Endpoints для управления позициями были удалены из nav2_test_server, так как они избыточны. Позиции управляются напрямую через API локального сервера `robot_service` (порт 8110), который является единым источником истины.

**Используйте robot_service API напрямую:**

- `GET /api/v1/robot/robot_positions/list` - Получить список всех позиций
- `POST /api/v1/robot/robot_positions/save` - Сохранить/обновить позицию
- `POST /api/v1/robot/robot_positions/delete?position_id=...` - Удалить позицию
- `POST /api/v1/robot/robot_positions/run?position_id=...` - Запустить навигацию к позиции

**Пример использования:**
```bash
# Получить список позиций
curl http://localhost:8110/api/v1/robot/robot_positions/list

# Сохранить позицию
curl -X POST http://localhost:8110/api/v1/robot/robot_positions/save \
  -H "Content-Type: application/json" \
  -d '{"position_id": "position_A", "x": 1.2, "y": 3.4, "theta": 0.0, "description": "Position A"}'

# Удалить позицию
curl -X POST "http://localhost:8110/api/v1/robot/robot_positions/delete?position_id=position_A"

# Запустить навигацию к позиции
curl -X POST "http://localhost:8110/api/v1/robot/robot_positions/run?position_id=position_A"
```

### Дополнительные endpoints (планируются)

Следующие endpoints упомянуты в README.md, но еще не реализованы в текущей версии:

**Тестирование навигации:**
- `POST /api/test/navigate` - Отправить команду навигации
- `POST /api/test/cancel` - Отменить навигацию
- `GET /api/test/history` - История команд
- `POST /api/test/sequence` - Последовательность команд

**Анализ:**
- `GET /api/analysis/stats` - Статистика навигации
- `GET /api/analysis/topics` - Анализ топиков
- `GET /api/analysis/nav2` - Анализ Nav2
- `GET /api/analysis/metrics` - Системные метрики
- `GET /api/analysis/report` - Полный отчет анализа

**MQTT:**
- `GET /api/mqtt/status` - Статус подключения MQTT
- `GET /api/mqtt/last_status` - Последний статус навигации из MQTT

---

## ProcessManager - Управление процессами

### Описание класса

`ProcessManager` — основной класс для управления жизненным циклом ROS2-процессов. Он обеспечивает запуск, остановку, мониторинг и очистку процессов Nav2 и base_controller.

### Инициализация

```python
from nav2_test_server.process_manager import ProcessManager

manager = ProcessManager(workspace_path="/home/boris/ros2_ws")
```

**Параметры:**
- `workspace_path` (по умолчанию: "/home/boris/ros2_ws"): Путь к ROS2 workspace

### Методы

#### `start_nav2()`

Запускает Nav2 и base_controller через launch-файл `symovo_nav2.launch.py`.

```python
success = manager.start_nav2(
    map_file="/path/to/map.yaml",          # Опционально
    symovo_endpoint="https://192.168.1.100",
    amr_id=15,
    use_scan_converter=True
)
```

**Параметры:**
- `map_file` (опционально): Путь к YAML-файлу карты
- `symovo_endpoint` (по умолчанию: "https://192.168.1.100"): Endpoint API Symovo
- `amr_id` (по умолчанию: 15): ID AMR
- `use_scan_converter` (по умолчанию: True): Включить scan converter

**Возвращает:** `bool` — True если запуск успешен, False в противном случае

**Примечания:**
- Если процесс уже запущен, метод вернет False
- Процесс запускается в отдельной группе процессов для лучшего контроля
- Логи процесса сохраняются и доступны через `get_status()`

#### `start_base_controller()`

Запускает только узел base_controller.

```python
success = manager.start_base_controller(
    driver_endpoint="https://192.168.1.100",
    amr_id=15,
    tls_verify=False
)
```

**Параметры:**
- `driver_endpoint` (по умолчанию: "https://192.168.1.100"): Endpoint API Symovo
- `amr_id` (по умолчанию: 15): ID AMR
- `tls_verify` (по умолчанию: False): Включить проверку TLS

**Возвращает:** `bool` — True если запуск успешен

#### `stop_nav2()`

Останавливает процесс Nav2 и все связанные процессы.

```python
success = manager.stop_nav2()
```

**Возвращает:** `bool` — True если остановка успешна

**Примечания:**
- Метод сначала пытается корректно завершить процесс (SIGTERM)
- Если процесс не останавливается в течение 5 секунд, выполняется принудительное завершение (SIGKILL)
- Также завершаются все дочерние процессы (ros2 launch, base_controller_node, symovo_scan_converter)

#### `stop_base_controller()`

Останавливает процесс base_controller.

```python
success = manager.stop_base_controller()
```

#### `restart_nav2()`

Перезапускает Nav2 (останавливает и запускает заново).

```python
success = manager.restart_nav2(
    map_file="/path/to/map.yaml",
    symovo_endpoint="https://192.168.1.100",
    amr_id=15,
    use_scan_converter=True
)
```

**Параметры:** Аналогично `start_nav2()`

#### `restart_base_controller()`

Перезапускает base_controller.

```python
success = manager.restart_base_controller(
    driver_endpoint="https://192.168.1.100",
    amr_id=15,
    tls_verify=False
)
```

#### `get_status()`

Получить статус конкретного процесса.

```python
status = manager.get_status("nav2")
```

**Параметры:**
- `process_name`: Имя процесса ("nav2" или "base_controller")

**Возвращает:** `Dict[str, Any]`
```python
{
    "name": "nav2",
    "status": "running",  # stopped, starting, running, stopping, error
    "running": True,
    "pid": 12345,
    "logs": ["[INFO] Nav2 started", ...]  # Последние 20 строк логов
}
```

#### `get_all_status()`

Получить статус всех процессов.

```python
all_status = manager.get_all_status()
```

**Возвращает:** `Dict[str, Dict[str, Any]]`
```python
{
    "nav2": {...},
    "base_controller": {...}
}
```

#### `cleanup_orphaned_processes()`

Очистить зависшие процессы base_controller и symovo_scan_converter, которые не отслеживаются менеджером.

```python
cleaned = manager.cleanup_orphaned_processes()
```

**Возвращает:** `Dict[str, int]`
```python
{
    "base_controller": 2,      # Количество завершенных процессов
    "symovo_scan_converter": 1
}
```

**Примечания:**
- Метод безопасен для вызова в любое время
- Не завершает процессы, отслеживаемые менеджером
- Использует `psutil` для корректного завершения процессов

### Внутренние механизмы

#### Управление окружением ROS2

Метод `_get_ros2_env()` автоматически настраивает окружение ROS2:
1. Загружает переменные окружения из `/opt/ros/jazzy/setup.bash`
2. Загружает переменные окружения из `{workspace_path}/install/setup.bash`
3. Возвращает словарь с переменными окружения для subprocess

#### Чтение логов

Логи процессов читаются в фоновом потоке (`_read_process_logs()`):
- Логи сохраняются в `process_logs[process_name]`
- Хранятся последние 100 строк логов
- Обновление происходит в реальном времени

#### Безопасное завершение процессов

При остановке процесса:
1. Сначала отправляется SIGTERM процессу и его группе
2. Ожидание до 5 секунд для корректного завершения
3. Если процесс не завершился, отправляется SIGKILL
4. Дополнительно завершаются все дочерние процессы через `psutil`
5. Используется `pkill` для завершения процессов по паттерну имени

---

## RobotPositionClient - Управление позициями через API

### Описание класса

`RobotPositionClient` — клиент для работы с API локального сервера `robot_service` для управления позициями. Заменяет локальный файл `positions.yaml` на API-based хранилище.

### Инициализация

```python
from nav2_test_server.robot_position_client import RobotPositionClient

client = RobotPositionClient(
    base_url="http://localhost:8110",
    timeout=10.0
)
```

**Параметры:**
- `base_url` (по умолчанию: "http://localhost:8110"): Базовый URL API robot_service
- `timeout` (по умолчанию: 10.0): Таймаут запросов в секундах

### Методы

#### `list_positions()`

Получить список всех позиций из API.

```python
positions = await client.list_positions()
```

**Возвращает:** `List[Dict[str, Any]]` — список словарей с позициями

**Формат позиции:**
```python
{
    "position_id": "position_A",
    "x": 1.2,
    "y": 3.4,
    "theta": 0.0,
    "description": "Position A"
}
```

#### `get_position()`

Получить конкретную позицию по ID.

```python
position = await client.get_position("position_A")
```

**Параметры:**
- `position_id`: Идентификатор позиции

**Возвращает:** `Optional[Dict[str, Any]]` — словарь с позицией или None, если не найдена

#### `save_position()`

Сохранить или обновить позицию.

```python
success = await client.save_position(
    position_id="position_F",
    x=10.0,
    y=20.0,
    theta=1.57,
    description="Новая позиция F"
)
```

**Параметры:**
- `position_id`: Идентификатор позиции
- `x`: X координата в метрах
- `y`: Y координата в метрах
- `theta`: Ориентация в радианах
- `description`: Описание позиции

**Возвращает:** `bool` — True если успешно

**Исключения:**
- `ValueError`: Ошибка валидации данных
- `Exception`: Ошибка HTTP запроса

#### `delete_position()`

Удалить позицию.

```python
success = await client.delete_position("position_F")
```

**Параметры:**
- `position_id`: Идентификатор позиции

**Возвращает:** `bool` — True если успешно

**Исключения:**
- `ValueError`: Позиция не найдена
- `Exception`: Ошибка HTTP запроса

#### `run_position()`

Запустить навигацию к позиции.

```python
result = await client.run_position("position_A")
```

**Параметры:**
- `position_id`: Идентификатор позиции

**Возвращает:** `Dict[str, Any]` — результат с полями:
```python
{
    "success": True,
    "task_id": "task_123",
    "result": "navigating",
    "detail": "Navigation started",
    "error": None
}
```

#### `close()`

Закрыть HTTP клиент.

```python
await client.close()
```

**Примечание:** Вызывается автоматически при shutdown приложения.

---

## Примеры использования

### Пример 1: Запуск Nav2 через API

```bash
curl -X POST http://localhost:8000/api/process/nav2/start \
  -H "Content-Type: application/json" \
  -d '{
    "map_file": "/home/boris/ros2_ws/maps/warehouse.yaml",
    "symovo_endpoint": "https://192.168.1.100",
    "amr_id": 15,
    "use_scan_converter": true
  }'
```

### Пример 2: Проверка статуса процессов

```bash
# Статус Nav2
curl http://localhost:8000/api/process/nav2/status

# Статус всех процессов
curl http://localhost:8000/api/process/status
```

### Пример 3: Создание позиции

```bash
curl -X POST http://localhost:8000/api/positions \
  -H "Content-Type: application/json" \
  -d '{
    "position_id": "dock_1",
    "x": 5.0,
    "y": 10.0,
    "theta": 1.57,
    "description": "Docking station 1"
  }'
```

### Пример 4: Использование ProcessManager в Python

```python
from nav2_test_server.process_manager import ProcessManager

# Инициализация
manager = ProcessManager(workspace_path="/home/boris/ros2_ws")

# Запуск Nav2
if manager.start_nav2(
    map_file="/path/to/map.yaml",
    symovo_endpoint="https://192.168.1.100",
    amr_id=15
):
    print("Nav2 started successfully")
    
    # Проверка статуса
    status = manager.get_status("nav2")
    print(f"Status: {status['status']}")
    print(f"PID: {status['pid']}")
    print(f"Logs: {status['logs']}")
    
    # Остановка
    if manager.stop_nav2():
        print("Nav2 stopped successfully")
else:
    print("Failed to start Nav2")
```

### Пример 5: Мониторинг узлов ROS2

```bash
curl http://localhost:8000/api/monitor/nodes | jq
```

### Пример 6: Работа с позициями через robot_service API

```bash
# Получить список позиций
curl http://localhost:8110/api/v1/robot/robot_positions/list | jq

# Создать позицию
curl -X POST http://localhost:8110/api/v1/robot/robot_positions/save \
  -H "Content-Type: application/json" \
  -d '{
    "position_id": "dock_1",
    "x": 5.0,
    "y": 10.0,
    "theta": 1.57,
    "description": "Docking station 1"
  }'

# Удалить позицию
curl -X POST "http://localhost:8110/api/v1/robot/robot_positions/delete?position_id=dock_1"

# Запустить навигацию к позиции
curl -X POST "http://localhost:8110/api/v1/robot/robot_positions/run?position_id=dock_1"
```

### Пример 7: Использование RobotPositionClient в Python (для других модулей)

```python
import asyncio
from nav2_test_server.robot_position_client import RobotPositionClient

async def main():
    # Инициализация клиента
    client = RobotPositionClient(base_url="http://localhost:8110")
    
    try:
        # Получить все позиции
        positions = await client.list_positions()
        print(f"Found {len(positions)} positions")
        
        # Создать новую позицию
        await client.save_position(
            position_id="dock_1",
            x=5.0,
            y=10.0,
            theta=1.57,
            description="Docking station 1"
        )
        print("Position created")
        
        # Получить конкретную позицию
        position = await client.get_position("dock_1")
        print(f"Position: {position}")
        
        # Запустить навигацию к позиции
        result = await client.run_position("dock_1")
        print(f"Navigation started: {result}")
        
    finally:
        await client.close()

# Запуск
asyncio.run(main())
```

---

## Устранение неполадок

### Проблема: Процесс Nav2 не запускается

**Возможные причины:**
1. Launch-файл не найден
2. ROS2 окружение не настроено
3. Порт уже занят

**Решение:**
```bash
# Проверка наличия launch-файла
ls -la /home/boris/ros2_ws/launch/symovo_nav2.launch.py

# Проверка ROS2 окружения
source /opt/ros/jazzy/setup.bash
source /home/boris/ros2_ws/install/setup.bash
ros2 pkg list | grep symovo_nav2

# Проверка логов процесса
curl http://localhost:8000/api/process/nav2/status | jq .logs
```

### Проблема: Процесс не останавливается

**Решение:**
```python
# Использование cleanup для принудительной очистки
manager = ProcessManager()
cleaned = manager.cleanup_orphaned_processes()
print(f"Cleaned processes: {cleaned}")
```

Или через командную строку:
```bash
# Принудительное завершение всех процессов Nav2
pkill -f symovo_nav2.launch.py
pkill -f base_controller_node
pkill -f symovo_scan_converter
```

### Проблема: Позиции не сохраняются

**Проверка:**
1. Убедитесь, что robot_service запущен и доступен: `curl http://localhost:8110/api/v1/robot/robot_positions/list`
2. Проверьте настройку `ROBOT_SERVICE_URL` в `.env` файле
3. Проверьте логи сервера на наличие ошибок подключения к API
4. Убедитесь, что robot_service контейнер работает: `docker ps | grep robot_service`

### Проблема: MQTT не подключается

**Проверка:**
1. Убедитесь, что настройки MQTT указаны в `.env`
2. Проверьте доступность брокера: `ping 82.165.177.194`
3. Проверьте статус MQTT: `curl http://localhost:8000/api/monitor/health | jq .mqtt`

### Проблема: ROS2 узлы не видны

**Решение:**
1. Убедитесь, что ROS2 демон запущен: `ros2 daemon status`
2. Если демон не запущен: `ros2 daemon start`
3. Проверьте, что ROS2 инициализирован в сервере: `curl http://localhost:8000/api/monitor/health | jq .ros2`

### Проблема: Зависшие процессы

**Автоматическая очистка:**
```python
manager = ProcessManager()
cleaned = manager.cleanup_orphaned_processes()
```

**Ручная очистка:**
```bash
# Найти процессы
ps aux | grep base_controller_node
ps aux | grep symovo_scan_converter

# Завершить процессы
kill -TERM <PID>
# Если не помогает:
kill -KILL <PID>
```

---

## Дополнительная информация

### Структура файлов модуля

```
nav2_test_server/
├── __init__.py
├── main.py                    # FastAPI приложение
├── process_manager.py         # Управление процессами
├── ros2_monitor.py            # Мониторинг ROS2
├── mqtt_client.py             # MQTT клиент
├── data_collector.py          # Сбор данных
├── analysis.py                # Анализ данных
├── models.py                  # Pydantic модели
├── config.py                  # Конфигурация
├── static/                    # Веб-интерфейс
│   ├── index.html
│   └── monitoring.html
├── config/
│   └── positions.yaml         # Файл позиций
└── SPECIFICATION.md           # Этот документ
```

### Логирование

Логи процессов сохраняются в памяти и доступны через API:
- Последние 100 строк логов для каждого процесса
- Обновление в реальном времени
- Доступ через `GET /api/process/{name}/status`

### Безопасность

- CORS настроен для разрешения всех источников (для разработки)
- В продакшене рекомендуется настроить конкретные источники
- TLS для MQTT можно включить через `MQTT_USE_TLS=true`

### Производительность

- Кэширование данных ROS2 обновляется каждые 2 секунды
- Логи процессов ограничены 100 строками
- История навигации ограничена 1000 записями (настраивается)

---

## Заключение

Nav2 Testing Server предоставляет полный набор инструментов для управления, мониторинга и тестирования системы навигации Nav2. Модуль `ProcessManager` обеспечивает надежное управление жизненным циклом процессов, а REST API позволяет интегрировать функциональность в другие системы.

Для дополнительной информации см.:
- Swagger документация: http://localhost:8000/docs
- Исходный код: `/home/boris/ros2_ws/scripts/nav2_test_server/`

