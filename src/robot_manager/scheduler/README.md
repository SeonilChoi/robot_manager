# Scheduler

**When / what:** advance time (`step`), apply actions (`tick`), expose state and progress (0–1).

```mermaid
stateDiagram-v2
  [*] --> STOPPED
  STOPPED --> OPERATING: MOVE
  STOPPED --> HOMING: HOME
  OPERATING --> STOPPED: STOP / progress=1
  HOMING --> STOPPED: STOP / progress=1
  HOMING --> HOMING: HOME
```

**FsmScheduler:** `reset()` → STOPPED, time 0; `step()` → advance dt; `tick(FsmAction)` → `(changed, FsmState)`. Invalid transition raises.
