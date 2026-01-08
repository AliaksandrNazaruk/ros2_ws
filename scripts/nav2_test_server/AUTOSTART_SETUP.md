# Автозапуск navigation_integrated_node

## Текущая ситуация

**✅ navigation_integrated_node теперь запускается автоматически вместе с Nav2 через launch файл `symovo_nav2.launch.py`.**

**⚠️ НЕТ автоматического запуска при перезагрузке системы** (нужен systemd сервис или другой механизм автозапуска).

Сейчас все ROS2 ноды запускаются через:
- Launch файл `symovo_nav2.launch.py` (Nav2, base_controller, **navigation_integrated_node**)
- Скрипты (`start_navigation_node.sh`) - для ручного запуска только navigation_integrated_node
- API endpoints (`nav2_test_server`) - для программного запуска через ProcessManager

---

## Варианты настройки автозапуска

### Вариант 1: Добавить в launch файл (рекомендуется)

Добавить `navigation_integrated_node` в основной launch файл `symovo_nav2.launch.py`, чтобы он запускался вместе с Nav2 и base_controller.

**Преимущества:**
- Все компоненты запускаются вместе
- Единая точка управления
- Проще поддерживать

**Недостатки:**
- Нужно модифицировать launch файл
- Зависит от того, как запускается Nav2

---

### Вариант 2: Systemd сервис (для автозапуска при загрузке)

Создать systemd unit файл для автоматического запуска при загрузке системы.

**Преимущества:**
- Автоматический запуск при перезагрузке
- Управление через systemctl (start/stop/restart/status)
- Автоматический перезапуск при сбое
- Логирование в journald

**Недостатки:**
- Требует root прав для установки
- Нужно правильно настроить зависимости (ROS2, сеть, Config Service)

---

### Вариант 3: Добавить в существующий launch файл через IncludeLaunchDescription

Создать отдельный launch файл для `navigation_integrated_node` и включить его в основной launch файл.

**Преимущества:**
- Модульность
- Можно запускать отдельно или вместе
- Легко отключить при необходимости

---

## Рекомендуемое решение

### ✅ Шаг 1: Добавить navigation_integrated_node в symovo_nav2.launch.py (ВЫПОЛНЕНО)

`navigation_integrated_node` уже добавлен в `/home/boris/ros2_ws/launch/symovo_nav2.launch.py`.

**Что было добавлено:**
- Аргументы launch файла для navigation_integrated_node:
  - `robot_id` (default: 'robot_001')
  - `config_service_url` (default: 'http://localhost:7900')
  - `config_service_api_key` (default: '', **REQUIRED**)
  - `config_poll_interval` (default: '5.0')
  - `positions_file` (optional)
- Node для navigation_integrated_node, который запускается после Nav2

**Запуск:**
```bash
ros2 launch symovo_nav2 symovo_nav2.launch.py \
  map_file:=/path/to/map.yaml \
  config_service_api_key:=your_api_key_here
```

**Или через API:**
```bash
curl -X POST http://localhost:8000/api/process/nav2/start \
  -H "Content-Type: application/json" \
  -d '{
    "map_file": "/path/to/map.yaml",
    "config_service_api_key": "your_api_key_here",
    "robot_id": "robot_001",
    "config_service_url": "http://localhost:7900"
  }'
```

**Примечание:** Если `config_service_api_key` не указан, `navigation_integrated_node` не сможет подключиться к MQTT брокеру и будет выдавать ошибки, но Nav2 все равно запустится.

---

### Шаг 1 (старая версия - для справки):

Модифицировать `/home/boris/ros2_ws/launch/symovo_nav2.launch.py`:

```python
# Добавить в конец launch description, перед return ld

# Navigation integrated node (MQTT bridge)
navigation_integrated_node = Node(
    package='aehub_navigation',
    executable='navigation_integrated_node',
    name='navigation_integrated_node',
    output='screen',
    parameters=[{
        'robot_id': 'robot_001',
        'config_service_url': 'http://localhost:7900',
        'config_service_api_key': LaunchConfiguration('config_service_api_key'),  # Из аргументов
        'config_poll_interval': 5.0,
        'positions_file': LaunchConfiguration('positions_file'),  # Опционально
    }]
)

# Добавить аргумент для API key
declare_config_service_api_key = DeclareLaunchArgument(
    'config_service_api_key',
    default_value='',
    description='API key for Config Service (REQUIRED for navigation_integrated_node)'
)

# Добавить в ld
ld.add_action(declare_config_service_api_key)
ld.add_action(navigation_integrated_node)
```

**Запуск:**
```bash
ros2 launch symovo_nav2 symovo_nav2.launch.py \
  map_file:=/path/to/map.yaml \
  config_service_api_key:=your_api_key_here
```

---

### Шаг 2: Создать systemd сервис для автозапуска (опционально)

Если нужен автозапуск при загрузке системы, создать systemd unit файл.

**Файл:** `/etc/systemd/system/navigation-integrated-node.service`

```ini
[Unit]
Description=AE.HUB Navigation Integrated Node (MQTT Bridge)
After=network-online.target ros2-daemon.service
Wants=network-online.target

[Service]
Type=simple
User=boris
Group=boris
WorkingDirectory=/home/boris/ros2_ws
Environment="PATH=/opt/ros/jazzy/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin"
ExecStart=/bin/bash -c 'source /opt/ros/jazzy/setup.bash && source /home/boris/ros2_ws/install/setup.bash && python3 -m aehub_navigation.navigation_integrated_node --ros-args -p robot_id:=robot_001 -p config_service_url:=http://localhost:7900 -p config_service_api_key:=YOUR_API_KEY_HERE'
Restart=always
RestartSec=10
StandardOutput=journal
StandardError=journal

[Install]
WantedBy=multi-user.target
```

**Установка:**
```bash
# Скопировать файл
sudo cp navigation-integrated-node.service /etc/systemd/system/

# Загрузить API key из .env файла (опционально, можно вручную)
API_KEY=$(grep "CONFIG_SERVICE_API_KEY" /home/boris/ros2_ws/scripts/nav2_test_server/.env | cut -d '=' -f2 | tr -d ' ')
sudo sed -i "s/YOUR_API_KEY_HERE/$API_KEY/" /etc/systemd/system/navigation-integrated-node.service

# Перезагрузить systemd
sudo systemctl daemon-reload

# Включить автозапуск
sudo systemctl enable navigation-integrated-node.service

# Запустить сервис
sudo systemctl start navigation-integrated-node.service

# Проверить статус
sudo systemctl status navigation-integrated-node.service
```

**Управление:**
```bash
# Запустить
sudo systemctl start navigation-integrated-node

# Остановить
sudo systemctl stop navigation-integrated-node

# Перезапустить
sudo systemctl restart navigation-integrated-node

# Статус
sudo systemctl status navigation-integrated-node

# Логи
sudo journalctl -u navigation-integrated-node -f
```

---

## Текущий способ запуска

Сейчас `navigation_integrated_node` запускается вручную через:

```bash
./scripts/start_navigation_node.sh
```

Или напрямую:
```bash
python3 -m aehub_navigation.navigation_integrated_node \
  --ros-args \
  -p robot_id:=robot_001 \
  -p config_service_url:=http://localhost:7900 \
  -p config_service_api_key:=your_api_key
```

**Проблема:** При перезагрузке системы узел не запустится автоматически.

---

## Рекомендации

1. **Краткосрочно:** Продолжать запускать вручную через скрипт
2. **Среднесрочно:** Добавить в launch файл `symovo_nav2.launch.py` для запуска вместе с Nav2
3. **Долгосрочно:** Создать systemd сервис для автозапуска при загрузке системы

---

## Проверка автозапуска

После настройки автозапуска проверить:

```bash
# Проверить, запущен ли узел
ros2 node list | grep navigation_integrated_node

# Проверить статус systemd сервиса (если используется)
sudo systemctl status navigation-integrated-node

# Проверить логи
sudo journalctl -u navigation-integrated-node -n 50
```

---

## Зависимости для автозапуска

`navigation_integrated_node` требует:
1. **ROS2 демон** должен быть запущен
2. **Config Service** должен быть доступен (http://localhost:7900)
3. **MQTT брокер** должен быть доступен (конфигурация получается из Config Service)
4. **Nav2** должен быть запущен (для отправки целей)

**Порядок запуска:**
1. ROS2 демон
2. Config Service
3. Nav2 (через launch файл)
4. navigation_integrated_node (после Nav2)

---

## Итог

**Сейчас:** НЕТ автозапуска. Нужно запускать вручную.

**Рекомендация:** Добавить в launch файл `symovo_nav2.launch.py` для запуска вместе с Nav2, или создать systemd сервис для автозапуска при загрузке системы.

