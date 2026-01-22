## Цель

Быстрый e2e smoke-test цепочки:

- MQTT `navigateTo/cancel` → `aehub_mqtt_transport` → `aehub_mqtt_protocol_adapter`
- → `aehub_navigation_executor` → ROS action `capabilities/navigation/execute`
- → `aehub_nav2_capability_server` → Nav2 → `cmd_vel` → `base_controller` → Symovo

## Предусловия

- `base_controller` должен успешно читать Symovo и публиковать `robot/status`.
- В `robot_interface_readiness_symovo.yaml` должны быть корректные пути:
  - `symovo_motors_enabled_path: "state_flags.drive_ready"`
  - `symovo_estop_active_path: "state_flags.emergency_stop_reset_request"`
  - `symovo_status_json_required: true`

## Запуск e2e стека

1) Сборка:

```bash
cd ~/ros2_ws
colcon build --packages-select aehub_navigation_executor aehub_robot_readiness base_controller aehub_navigation
source install/setup.bash
```

2) Запуск:

```bash
ros2 launch aehub_navigation_executor e2e_symovo_mqtt_to_nav2.launch.py \
  robot_id:=robot_001 \
  robot_params_file:=/home/boris/ros2_ws/src/aehub_robot_readiness/config/robot_interface_readiness_symovo.yaml \
  config_service_url:=http://localhost:7900 \
  api_key:=YOUR_API_KEY
```

## Preflight (ROS)

Проверить, что readiness не блокирует выполнение:

```bash
ros2 topic echo /robot/robot_001/robot/status
ros2 topic echo /robot/robot_001/health/robot_readiness
ros2 topic echo /robot/robot_001/health/navigation_capability
```

Ожидания:
- `robot/status`: `driver_connected=true`, `motors_enabled_valid=true`, `estop_active_valid=true`
- `health/robot_readiness`: `OK` когда моторы разрешены и e-stop не активен
- `health/navigation_capability`: `OK` только если composite readiness == READY

## Smoke (MQTT → navigateTo)

Отправить команду (пример — `mosquitto_pub`):

```bash
mosquitto_pub -h <BROKER_HOST> -p <BROKER_PORT> -u <USER> -P <PASS> \
  -t "aroc/robot/robot_001/commands/navigateTo" \
  -m '{"command_id":"11111111-1111-1111-1111-111111111111","x":0.1,"y":0.0,"theta":0.0}'
```

Подписаться на события:

```bash
mosquitto_sub -h <BROKER_HOST> -p <BROKER_PORT> -u <USER> -P <PASS> \
  -t "aroc/robot/robot_001/events"
```

Ожидания:
- сначала `ack` (`accepted` или `rejected`)\n+  - если `rejected`: причина должна указывать на readiness / Nav2 недоступность\n+- затем `result` (`succeeded|canceled|aborted|error`)

## Smoke (MQTT → cancel)

```bash
mosquitto_pub -h <BROKER_HOST> -p <BROKER_PORT> -u <USER> -P <PASS> \
  -t "aroc/robot/robot_001/commands/cancel" \
  -m '{"command_id":"22222222-2222-2222-2222-222222222222"}'
```

Ожидание: `ack accepted` + `result succeeded` (best-effort cancel).

