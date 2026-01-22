## Nav2 TF/odom/map smoke checklist (single robot)

Assume you launch everything in namespace:

- `NS=/robot/<robot_id>`

### Core topics

- **Odometry**

```bash
ros2 topic hz "$NS/odom"
ros2 topic echo -n 1 "$NS/odom/header"
ros2 topic echo -n 1 "$NS/odom/child_frame_id"
```

- **TF (odom->base_link)**

```bash
ros2 run tf2_ros tf2_echo odom base_link
```

- **Laser scan**

```bash
ros2 topic hz "$NS/scan"
ros2 topic echo --once "$NS/scan" | grep "header:"
```

### Localization (map->odom)

- **Check map sync nodes are running** (NEW flow)

```bash
ros2 node list | grep -E "symovo_map_mirror|nav2_localization_orchestrator"
ros2 lifecycle get "$NS/symovo_map_mirror" 2>/dev/null || echo "Node not found (may use legacy fetcher)"
ros2 lifecycle get "$NS/nav2_localization_orchestrator" 2>/dev/null || echo "Node not found (may use legacy fetcher)"
```

- **Map status (from Symovo map mirror)** (NEW flow)

```bash
ros2 topic echo --once "$NS/infra/map/status"
# Expected: ready=true, revision>0, yaml_path non-empty
```

- **AMCL lifecycle state**

```bash
ros2 lifecycle get "$NS/amcl"
```

- **Map server lifecycle state**

```bash
ros2 lifecycle get "$NS/map_server"
```

- **Map metadata** (Nav2 map_server output)

```bash
ros2 topic echo --once "$NS/map_metadata"
# If empty, check map_server lifecycle state and yaml_filename parameter
```

- **TF (map->odom)** (appears only after AMCL is ACTIVE and initial pose is set)

```bash
ros2 run tf2_ros tf2_echo map odom
```

- **AMCL pose**

```bash
ros2 topic hz "$NS/amcl_pose"
ros2 topic echo --once "$NS/amcl_pose"
```

### Navigation execution (capability action)

Send a goal through capability server (replace x/y/theta):

```bash
ros2 action send_goal "$NS/capabilities/navigation/execute" aehub_msgs/action/NavigationExecute \
  "{command_id: test_1, x: 1.0, y: 2.0, theta: 0.0}"
```

If the goal is rejected, check readiness diagnostics:

```bash
ros2 topic echo --once "$NS/diagnostics"
```

