# Nav2 Fixes Summary

## Дата: 2026-01-15

## Исправления

### 1. ✅ Увеличен transform_tolerance

**Файл**: `config/nav2_symovo_params.yaml`

**Изменения**:
- AMCL: `transform_tolerance: 1.0` → `2.0` секунды
- Controller (MPPI): `transform_tolerance: 0.1` → `1.0` секунда
- Recoveries: `transform_tolerance: 0.1` → `1.0` секунда

**Причина**: Nav2 controller получал ошибку "extrapolation into the future" при трансформации goal pose в costmap frame. Увеличение tolerance позволяет обрабатывать небольшие задержки в публикации TF.

### 2. ✅ Проверен QoS для /initialpose

**Файл**: `src/aehub_navigation/src/aehub_navigation/initial_pose_from_symovo.py`

**Статус**: ✅ Уже правильный
- Используется `QoSReliabilityPolicy.BEST_EFFORT`
- Совместимо с AMCL

### 3. ✅ Создан скрипт для исправления дублирования узлов

**Файл**: `scripts/fix_nav2_issues.sh`

**Функция**: Останавливает дублирующиеся launch процессы и очищает lock файлы.

## Обнаруженные проблемы

### ❌ Критическая проблема: TF map->odom не публикуется

**Симптомы**:
- AMCL активен (lifecycle: active)
- `/scan` публикуется (52 сообщения за 10 сек)
- `/odom` публикуется (153 сообщения за 10 сек)
- `/map` публикуется
- Но TF `map->odom` НЕ существует

**Ошибка в логах**:
```
[ERROR] Exception in transformPose: Lookup would require extrapolation into the future.
Requested time 1768503307.884811 but the latest data is at time 1768503307.877845,
when looking up transform from frame [map] to frame [odom]
[ERROR] Unable to transform goal pose into costmap frame
```

**Возможные причины**:
1. AMCL не получает `/initialpose` правильно (хотя QoS правильный)
2. AMCL не может локализоваться из-за проблем с `/scan` данными
3. AMCL не публикует TF из-за проблем с синхронизацией времени

**Следующие шаги**:
1. Проверить, получает ли AMCL `/initialpose` (проверить логи AMCL)
2. Проверить формат `/scan` данных (frame_id, timestamp)
3. Убедиться, что AMCL публикует `/amcl_pose`
4. Проверить синхронизацию времени между узлами

### ❌ Проблема: Дублирование узлов

**Симптомы**:
- 6 экземпляров `symovo_scan_converter` (должен быть 1)
- Дублируются: `initial_pose_from_symovo`, `lifecycle_manager_*`, `static_tf_base_laser`

**Решение**: Использовать `scripts/fix_nav2_issues.sh` перед запуском launch файла.

### ❌ Проблема: /cmd_vel не публикуется

**Симптомы**:
- Nav2 controller получает goal
- Но не может трансформировать его в costmap frame
- Goal прерывается до того, как controller может отправить команды

**Причина**: Следствие проблемы с TF map->odom.

## Диагностические инструменты

### 1. `scripts/diagnose_nav2_issues.py`

Проверяет:
- `/scan` публикацию
- `/initialpose` публикацию
- `/cmd_vel` публикацию
- `/odom` публикацию

### 2. `scripts/fix_nav2_issues.sh`

Исправляет:
- Останавливает дублирующиеся процессы
- Очищает lock файлы

## Команды для проверки

```bash
# Проверить TF
ros2 run tf2_ros tf2_echo map odom

# Проверить /amcl_pose
ros2 topic echo /amcl_pose

# Проверить AMCL lifecycle
ros2 lifecycle get /amcl

# Проверить узлы
ros2 node list | grep -E "(scan_converter|initial_pose)"

# Проверить /scan
ros2 topic hz /scan

# Запустить диагностику
python3 scripts/diagnose_nav2_issues.py

# Исправить дублирование
./scripts/fix_nav2_issues.sh
```

## Рекомендации

1. **Перед запуском launch файла**: Всегда запускать `scripts/fix_nav2_issues.sh`
2. **Проверить TF**: Убедиться, что TF `map->odom` существует перед отправкой команд
3. **Мониторинг**: Использовать `scripts/diagnose_nav2_issues.py` для проверки системы
4. **Логи**: Проверять логи AMCL на ошибки локализации
