# Статус ROS2 нод

## Проверка дубликатов

### Команды для проверки:

```bash
# Список всех нод
ros2 node list

# Подсчет дубликатов
ros2 node list | sort | uniq -c | sort -rn

# Проверка конкретной ноды
ros2 node list | grep -c base_controller
```

### Очистка дубликатов:

```bash
# Использовать скрипт очистки
bash scripts/cleanup_duplicate_nodes.sh

# Или вручную
pkill -f <node_name>
```

## Критические ноды

### Обязательные ноды:

1. **base_controller** - должен быть 1 экземпляр
   - Публикует `/odom` и TF
   - Подписывается на `/cmd_vel`

2. **navigation_integrated_node** - должен быть 1 экземпляр
   - Интегрирует MQTT и Nav2
   - Обрабатывает команды навигации

3. **Nav2 компоненты:**
   - `/planner_server` - должен быть active
   - `/controller_server` - должен быть active
   - `/bt_navigator` - должен быть active
   - `/lifecycle_manager_navigation` - должен быть 1 экземпляр

## Типичные дубликаты

### Проблемные дубликаты:

- `/symovo_scan_converter` - может быть несколько экземпляров (нормально, если запущено несколько launch файлов)
- `/recoveries_server` - должен быть 1 экземпляр
- `/local_costmap` - должен быть 1 экземпляр
- `/global_costmap` - должен быть 1 экземпляр
- `/lifecycle_manager_navigation` - должен быть 1 экземпляр

### Тестовые ноды (можно игнорировать):

- `/mock_odom_publisher` - используется для тестирования

## Активация Nav2 компонентов

```bash
ros2 lifecycle set /planner_server activate
ros2 lifecycle set /controller_server activate
ros2 lifecycle set /bt_navigator activate
```

Или использовать скрипт:
```bash
bash scripts/setup_nav2_for_navigation.sh
```
