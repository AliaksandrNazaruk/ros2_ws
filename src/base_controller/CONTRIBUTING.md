## Contributing

- Copyright 2026 Boris

- Follow the repository architectural contract in `docs/SRS_GLOBAL_STACK.md`.
- Keep `base_controller` as a thin ROS interface adapter (no FSM, no readiness decisions).
- Use relative topic names and rely on namespacing via launch (`/robot/<id>/...`).

