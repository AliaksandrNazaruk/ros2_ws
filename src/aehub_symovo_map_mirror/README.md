# `aehub_symovo_map_mirror`

ROS Interface package for **Symovo → Nav2 map synchronization**.

## Contract (runtime facts we must observe)

From `hardware_api.json` (OpenAPI):
- **Map list**: `GET /v0/map` returns objects with:
  - `id` (may be string UUID-like), `last_changed` (int, caching/revision),
  - `offsetX`, `offsetY` (meters), `resolution` (m/pixel),
  - `size` (pixels), `enabled`.
- **Map image**: `GET /v0/map/{id}/full.png`.
- **AMR pose**: `GET /v0/amr/{id}/pose` returns `{x,y,theta,map_id}` where `map_id` is an integer reference.
- **Map change notification** (`wait_for_changes`) is **not supported in production**: observed to hang (no usable long-poll).
  - Production mode is **poll-only** (see `update_mode=poll`).

Important: the API shows **mixed identifier types** (`map_id` in pose is integer; map `id` may be string). Code must be defensive and support real deployments.

## Manual probe (collect_runtime_contract)

Use this tool to print current runtime values from your robot:

```bash
ros2 run aehub_symovo_map_mirror symovo_map_contract_probe -- \
  --endpoint https://192.168.1.100 --amr-id 15 --tls-verify false
```

Optionally test `wait_for_changes`:

```bash
ros2 run aehub_symovo_map_mirror symovo_map_contract_probe -- \
  --endpoint https://192.168.1.100 --amr-id 15 --tls-verify false \
  --try-wait-for-changes --wait-timeout-sec 5
```

Note: this probe is best-effort only; `wait_for_changes` MUST NOT be used for correctness or liveness.

## Pose conversion policy (geometry module)

Because your runtime indicates **pose is not directly in Nav2 `map` frame**, conversion must be explicit.
This package provides `symovo_map_geometry.py` with parameterized modes:
- **Origin**: `origin_offsets` or `origin_neg_offsets`
- **Pose transform**: `pose_subtract_offsets[_flip_y]` or `pose_add_offsets[_flip_y]` (or `pose_identity`)

We will lock the correct combination once the first end-to-end run confirms geometry.

