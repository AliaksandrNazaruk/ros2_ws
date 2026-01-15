# Nav2 Diagnostics Report

## Дата: 2026-01-15

## Результаты диагностики

### ✅ Что работает:

1. **/scan topic**: ✅ Публикуется (52 сообщения за 10 сек)
   - symovo_scan_converter работает
   - Формат LaserScan корректный
   - 6 публикаторов, 3 подписчика

2. **/odom topic**: ✅ Публикуется (153 сообщения за 10 сек)
   - base_controller публикует odometry
   - Данные от Symovo API получаются

3. **AMCL lifecycle**: ✅ Активен (state: active [3])
   - Параметры корректные:
     - scan_topic: scan
     - global_frame_id: map
     - odom_frame_id: odom

4. **MQTT коммуникация**: ✅ Работает
   - Команды принимаются
   - События публикуются

### ❌ Проблемы:

1. **TF map->odom**: ❌ КРИТИЧЕСКАЯ ПРОБЛЕМА
   - TF существует, но данные не синхронизированы
   - Ошибка: "Lookup would require extrapolation into the future"
   - Controller не может трансформировать goal pose в costmap frame
   - **Причина**: AMCL не публикует TF map->odom синхронно или данные устарели

2. **/cmd_vel**: ❌ Не публикуется (0 сообщений)
   - **Причина**: Nav2 прерывает цель ДО того, как controller может отправить команды
   - Controller получает goal, но не может его обработать из-за TF проблемы

3. **/initialpose**: ⚠️ QoS несовместимость
   - initial_pose_from_symovo публикует с RELIABILITY QoS
   - AMCL ожидает BEST_EFFORT QoS
   - Сообщения не доставляются

4. **Дублирование узлов**: ❌ КРИТИЧЕСКАЯ ПРОБЛЕМА
   - 6 экземпляров symovo_scan_converter (должен быть 1!)
   - Дублируются: initial_pose_from_symovo, lifecycle_manager_*, static_tf_base_laser
   - **Причина**: Множественные запуски launch файла

5. **Nav2 Controller ошибка**: ❌
   ```
   [ERROR] Exception in transformPose: Lookup would require extrapolation into the future.
   Requested time 1768503307.884811 but the latest data is at time 1768503307.877845,
   when looking up transform from frame [map] to frame [odom]
   [ERROR] Unable to transform goal pose into costmap frame
   ```

## Детальный анализ

### Проблема 1: TF map->odom не синхронизирован

**Симптомы:**
- Controller получает goal, но не может трансформировать его в costmap frame
- Ошибка extrapolation указывает на задержку в публикации TF

**Возможные причины:**
1. AMCL не публикует TF map->odom регулярно
2. TF публикуется с задержкой
3. AMCL не получает /scan или /initialpose правильно

**Решение:**
- Проверить, публикует ли AMCL /amcl_pose
- Проверить transform_tolerance в AMCL параметрах
- Увеличить transform_tolerance или исправить синхронизацию

### Проблема 2: /initialpose QoS несовместимость

**Симптомы:**
- initial_pose_from_symovo публикует с RELIABILITY
- AMCL ожидает BEST_EFFORT
- Сообщения не доставляются

**Решение:**
- Изменить QoS в initial_pose_from_symovo на BEST_EFFORT
- Или изменить QoS в AMCL на RELIABILITY

### Проблема 3: Дублирование узлов

**Симптомы:**
- 6 экземпляров symovo_scan_converter
- Множественные экземпляры других узлов

**Решение:**
- Остановить все запущенные launch файлы
- Очистить lock файлы
- Запустить один экземпляр launch файла

## Рекомендации

### Немедленные действия:

1. **Остановить дублирующиеся узлы:**
   ```bash
   pkill -9 -f "symovo_nav2"
   pkill -9 -f "ros2 launch"
   rm -f /tmp/*.lock
   ```

2. **Исправить QoS для /initialpose:**
   - Изменить initial_pose_from_symovo.py для использования BEST_EFFORT QoS

3. **Проверить AMCL публикацию TF:**
   - Убедиться, что AMCL публикует /amcl_pose
   - Проверить transform_tolerance параметр

4. **Увеличить transform_tolerance:**
   - В nav2_symovo_params.yaml увеличить transform_tolerance для AMCL и costmaps

### Долгосрочные исправления:

1. Добавить проверку дублирования узлов в launch файл
2. Исправить QoS несовместимость
3. Добавить мониторинг TF синхронизации
4. Улучшить обработку ошибок TF в controller

## Команды для проверки

```bash
# Проверить TF
ros2 run tf2_ros tf2_echo map odom

# Проверить /amcl_pose
ros2 topic echo /amcl_pose

# Проверить узлы
ros2 node list | grep -E "(scan_converter|initial_pose)"

# Проверить AMCL lifecycle
ros2 lifecycle get /amcl

# Проверить /scan
ros2 topic hz /scan
```
