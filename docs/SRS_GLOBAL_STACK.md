# SRS: Global Requirements for AE.HUB ROS 2 Stack (Architectural Contract)
Версия: 0.1  
Статус: **Нормативный документ (MUST/SHALL)**  
Цель: чтобы агенты/разработчики писали код «как топовые разработчики роботов на ROS 2» и **не нарушали зоны ответственности**, обеспечивая **лёгкую заменяемость модулей**.

---

## 1. Термины и принципиальные определения

- **ROS 2**: middleware (транспорт/синхронизация/инструменты), **не архитектура**.
- **Архитектура**: слойная модель ответственности, контракты и инварианты владения ресурсами.
- **Application Layer / Orchestrator**: бизнес‑логика сценариев, FSM, идемпотентность.
- **Capability Layer (Service)**: функциональная возможность робота как сервис с **одним API**.
- **ROS Interface Layer**: тонкие адаптеры/драйверы; минимум логики.
- **Hardware Implementation**: реализация “железа” (real/sim) как взаимозаменяемая часть **в одном графе**.
- **Ready/Health**: готовность/здоровье узла; **ACTIVE == готов к эксплуатации**.

---

## 2. Цели (Goals)

- **G1**: обеспечить строгие зоны ответственности и предотвращение “Swiss Army knife” нод.
- **G2**: обеспечить заменяемость Nav2/симуляции/железа **без переписывания Application logic**.
- **G3**: обеспечить production‑готовность: Lifecycle + Health + Watchdog + деградация.
- **G4**: обеспечить масштабирование на multi‑robot: всё под `/robot/<id>/...`.

## 3. Не‑цели (Non‑Goals)

- **NG1**: “подписаться на топики Nav2 и писать логику по сообщениям” — запрещено.
- **NG2**: “отдельная система симулятора с другим графом” — запрещено.
- **NG3**: “сделать один монолитный ROS‑нод, который всё умеет” — запрещено.

---

## 4. Референс‑модель слоёв (обязательная)

### 4.0. Directionality contract: “слои не управляют соседним”

- **INV-DIR-01 (MUST)**: ни один слой **не управляет** соседним слоем (no “layer manager”).
- **INV-DIR-02 (MUST)**: направление взаимодействия строго:
  - **наблюдение вверх** (events/telemetry/state)
  - **команды вниз** (intents/requests/goals)
- **INV-DIR-03 (MUST)**: readiness = **gate/validator**, не controller:
  - readiness делает **снимок‑проверку** и возвращает результат (READY/NOT_READY + summary)
  - readiness не публикует команд, не меняет lifecycle других узлов, не “чинит” систему
- **INV-DIR-04 (MUST)**: “Nav2 = чёрный ящик” означает:
  - Nav2 рассматривается как **внешняя система**, а не “dependency, на которую можно опираться топиками”
  - бизнес‑решения не принимаются на основании топиков Nav2

### 4.1. Слои

1) **Application Layer (Orchestrator / FSM)**
- Содержит: FSM, идемпотентность, политики (rate‑limit, дедуп), сценарии.
- Не содержит: Nav2‑топики, `/cmd_vel`, hw‑топики.

2) **Capability Layer (Services)**
- Содержит: единый API capability (Actions/Services), readiness/STOP политики, внутренние таймауты.
- Внутри может использовать Nav2/другие зависимости, но наружу отдаёт **только свой контракт**.

3) **ROS Interface Layer**
- Содержит: transport/edge‑protocol адаптеры, драйверы.
- Разрешено: минимальная трансформация формата, валидация схемы на границе.
- Запрещено: FSM, дедуп, бизнес‑политики.

4) **Hardware / Simulation**
- Реализации одинакового интерфейса (real vs sim), выбираются аргументами launch.

### 4.2. Инвариант “тонкий ROS слой”
- **INV-THIN-01 (MUST)**: каждый ROS‑нод должен быть **либо** адаптером/драйвером, **либо** сервисом‑capability, **либо** orchestrator. Смешивание запрещено.

### 4.3. Навигация (доменная 6‑слойная модель)

Эта модель является обязательной для навигационного домена и уточняет зоны ответственности.

#### L1 — Command / Intent Layer (внешний мир)
- **Ответственность**: *что хотим сделать*, но не “как”.
- **Компоненты**: MQTT/HTTP/REST/UI/Mission Planner.
- **Команды**: `navigate_to(pose)`, `cancel`, `pause`, `resume`.
- **Запрещено (MUST NOT)**: проверять TF/odom, знать про Nav2 lifecycle, зависеть от Nav2 топиков.

#### L2 — Nav2Adapter (Application / Orchestration Layer)
- **Ответственность**: единая точка управления навигацией; FSM — единственный источник истины.
- **Содержит**: FSM навигации, контракт с L1, публикацию событий, политику идемпотентности/дедупа.
- **Зависимости (строго вниз)**:
  - readiness gates (L3) **только через итог READY/NOT_READY**
  - action client к capability/Nav2 (через контракт)
- **Инварианты**:
  - **INV-NAVAD-01 (MUST)**: никогда не отправлять goal, если любой gate != READY
  - **INV-NAVAD-02 (MUST)**: никогда не ждать readiness в lifecycle callbacks (только snapshot checks в рабочем контуре)
  - **INV-NAVAD-03 (MUST)**: FSM публикует единый event stream наверх; consumers не читают “детали внутренностей”

#### L3 — Readiness Gates (Domain Validation Layer)
- **Ответственность**: детерминированная валидация готовности (snapshot‑проверки), без управления.
- **Nav2ReadinessGate** проверяет (примерно): lifecycle Nav2 нод, наличие action `/navigate_to_pose`, TF `map→odom→base_link`, необходимые источники локализации/карты.
  - **Запрещено**: знать про robot hardware, владеть `/cmd_vel`.
- **RobotReadinessGate** проверяет физическую готовность (примерно): odom, наличие подписчиков cmd_vel интерфейса, motors enabled, e‑stop.
  - **Запрещено**: знать про Nav2, lifecycle Nav2.
- **Композиция**:
  - **INV-GATE-01 (MUST)**: `ready = nav2_gate && robot_gate`
  - **INV-GATE-02 (MUST)**: Nav2Adapter видит **только итог** (READY/NOT_READY + краткий summary), детали доступны как telemetry/debug отдельно.

#### L4 — Nav2 Stack (External System)
- **Ответственность**: внешняя навигационная система (не наш код).
- **Контракт**: Actions + Lifecycle + TF + необходимые топики.
- **Запрещено**: добавлять туда логику адаптера или readiness.

#### L5 — Motion Execution Layer (Robot Control)
- **Компоненты**: base_controller, cmd_vel_mux, motor drivers, safety controller.
- **Контракт**: принимает `/cmd_vel`, публикует `/odom`, обеспечивает safety stop.
- **Инвариант**: RobotReadinessGate — наблюдатель, а не управляющий контроллер.

#### L6 — Hardware / Drivers
- **Компоненты**: motor drivers, encoders, safety relays, e‑stop.
- **Запрещено**: ROS‑логика и Nav2‑логика.

#### Trace (нормативный поток)

`[Mission/Intent] → [Nav2Adapter FSM] → (if READY) [Action Client] → [Nav2 Stack] → [/cmd_vel] → [Base Controller] → [Robot]`

---

## 5. Nav2: правила интеграции (строго)

- **INV-NAV2-01 (MUST)**: Nav2 используется **только** через Actions + Lifecycle.
- **INV-NAV2-02 (MUST NOT)**: никакой бизнес‑логики на топиках Nav2 (`/odom`, `/amcl_pose`, `/cmd_vel`, etc.).
- **INV-NAV2-03 (MUST)**: наружу Nav2 рассматривается как **чёрный ящик** за capability‑API.

---

## 6. Контракты и “ад из топиков”: жёсткие правила

### 6.1. Правило 1‑входного топика
- **INV-IO-01 (MUST)**: компонент (узел) должен иметь **не более 1 входного топика** для своего домена.
- **INV-IO-02 (SHOULD)**: один выходной поток событий на домен (единый event stream).

### 6.2. Никаких “сырых” топиков наверх
- **INV-RAW-01 (MUST NOT)**: нельзя поднимать “сырые” транспортные/внутренние топики как интерфейс capability.
- **INV-RAW-02 (MUST)**: наверх идут только агрегированные/строго типизированные статусы/события.

---

## 7. Namespace и именование (обязательное)

- **INV-NS-01 (MUST)**: всё должно жить под namespace: `/robot/<robot_id>/...`.
- **INV-NS-02 (MUST)**: в коде использовать **относительные** имена топиков (без ведущего `/`).
- **INV-NS-03 (MUST NOT)**: запрет на глобальные `/odom`, `/scan`, `/cmd_vel` в интерфейсах верхних слоёв (драйверы/локальные реализации могут иметь свои имена, но запуск должен быть namespaced).

---

## 8. Lifecycle / Health / Watchdog (production минимум)

### 8.1. Lifecycle обязателен
- **INV-LC-01 (MUST)**: все ключевые узлы должны быть LifecycleNodes.
- **INV-LC-02 (MUST)**: ACTIVE означает “готов обслуживать запросы”.
- **INV-LC-03 (MUST)**: узел обязан корректно освобождать ресурсы в deactivate/cleanup.

### 8.2. Health
- **INV-HLTH-01 (MUST)**: каждый ключевой узел публикует health (`diagnostic_msgs/DiagnosticArray` или эквивалент).
- **INV-HLTH-02 (MUST)**: health отражает readiness/ошибки зависимостей/статус жизненного цикла.

### 8.3. Watchdog + деградация
- **INV-WD-01 (MUST)**: должны быть таймауты на внешние зависимости (MQTT/HTTP/Nav2).
- **INV-WD-02 (MUST)**: деградация должна быть логической (не “молчание”): события/health должны показывать проблему.

---

## 9. Sim/Real: один launch, один graph

- **INV-SIM-01 (MUST)**: Simulation = hardware implementation, а не отдельная система.
- **INV-SIM-02 (MUST)**: **один bringup**, **один граф**, отличия только через параметры:
  - `use_sim_time:=true|false`
  - `hardware:=sim|real`
- **INV-SIM-03 (MUST NOT)**: разные графы для sim/real запрещены.

---

## 10. Navigation как сервис (NavigationService API)

### 10.1. Модель capability
Capability Navigation предоставляет наружу **один API**:
- Action: `capabilities/navigation/execute` (`aehub_msgs/action/NavigationExecute`)
- (опционально) Service: `capabilities/navigation/get_state`
- (опционально) Service: `capabilities/navigation/reset`

### 10.2. Владение критическими ресурсами
- **INV-OWN-01 (MUST)**: `/cmd_vel` публикует **только** Navigation Capability (в рамках namespace).
- **INV-OWN-02 (MUST)**: STOP‑semantics (burst/сafety policy) — **только** Capability.
- **INV-OWN-03 (MUST)**: readiness gates являются **наблюдателями** и проверяются *по запросу* (gate snapshot), но не управляют системой.
- **INV-OWN-04 (MUST)**: orchestrator никогда не публикует `/cmd_vel` и не “помогает” Nav2.

---

## 11. Transport / Protocol boundary (MQTT)

### 11.1. Transport‑узел (чистый транспорт)
- **INV-TR-01 (MUST)**: transport не парсит JSON и не знает команд/событий.
- **INV-TR-02 (MUST)**: transport мостит только типизированный конверт:
  - `infra/mqtt/in` (`aehub_msgs/MqttEnvelope`)
  - `infra/mqtt/out` (`aehub_msgs/MqttEnvelope`)

### 11.2. Protocol edge adapter (единственное место для JSON)
- **INV-PA-01 (MUST)**: JSON допускается только в protocol adapter.
- **INV-PA-02 (MUST)**: protocol adapter имеет:
  - один вход: `infra/mqtt/in` (`MqttEnvelope`)
  - один выход: `infra/mqtt/out` (`MqttEnvelope`)
  - одну подписку на события домена: `events/navigation` (`NavigationEvent`)
- **INV-PA-03 (MUST)**: protocol adapter не делает FSM/дедуп/политики — только схема/маршрутизация.

---

## 12. Нормативные интерфейсы (целевые имена)

Все имена ниже предполагаются **относительными** и будут жить под `/robot/<id>/...` через launch.

- `infra/mqtt/broker_config` (`aehub_msgs/BrokerConfig`)
- `infra/mqtt/in` (`aehub_msgs/MqttEnvelope`)
- `infra/mqtt/out` (`aehub_msgs/MqttEnvelope`)
- `commands/navigation` (`aehub_msgs/NavigationCommand`)
- `events/navigation` (`aehub_msgs/NavigationEvent`)
- `capabilities/navigation/execute` (`aehub_msgs/action/NavigationExecute`)
- `health/<node>` (`diagnostic_msgs/DiagnosticArray` или эквивалент)

---

## 13. Anti‑patterns (запрещено)

- Подписка на Nav2 топики и принятие решений “по сообщениям”.
- Оркестратор публикует `/cmd_vel` или делает STOP.
- Нода с 5–10 входными топиками “потому что так удобно”.
- Разные launch/разные графы для sim и real.
- Глобальные имена топиков без `/robot/<id>/...`.

---

## 14. Чеклист “проект здоров” (Definition of Done)

- Можно удалить Nav2 и заменить mock‑capability без изменения orchestrator.
- FSM (Application Layer) запускается и тестируется без ROS‑топиков Nav2.
- Sim/Real: один bringup, один graph.
- Нет бизнес‑логики на топиках.
- Явно определён владелец каждого критического топика (особенно `/cmd_vel`).
- Каждый ключевой узел: Lifecycle + Health + Watchdog.

