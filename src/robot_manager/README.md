# robot_manager

Load robot from YAML; expose **control**, **update**, and **home / stop / move / auto**.

```mermaid
flowchart LR
  A[YAML] --> B[RobotManager]
  B --> C[control]
  B --> D[update]
  B --> E[home|stop|move|auto]
  C --> F[JointState]
  D --> G[state]
  E --> H[mode]
```

**RobotManager:** `control(status)` → command or None; `update(status, obstacles)` → internal state; mode methods set flags only.  
**Types:** `JointState`, `RobotConfig`, obstacle/pose/twist dataclasses; FSM: `FsmState`, `FsmAction`.
