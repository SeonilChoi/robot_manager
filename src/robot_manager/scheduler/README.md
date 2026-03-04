# Scheduler

The **scheduler** decides *when* the robot does *what*: it advances time, accepts commands (actions), and updates an internal **state** (e.g. STOPPED, OPERATING, HOMING). You typically call `step()` to advance time and `tick(action)` to apply a command and get the new state and progress.

This README is a **beginner guide**: it explains the idea, shows a visual state graph, and documents each function in detail.

---

## What does the scheduler do?

- **Time:** Keeps an internal time `_t` and a time step `_dt`. You call `step()` to advance time.
- **State:** Holds the current mode (e.g. STOPPED, OPERATING, HOMING). Actions (STOP, MOVE, HOME) change the state according to a **transition table**.
- **Progress:** For timed actions (e.g. “move for 1 second”), the scheduler reports progress from 0 to 1. When progress reaches 1, the action is “done” (e.g. HOMING → STOPPED).

So: you **step** time, then **tick** with an action; the scheduler returns the new state and progress.

---

## Architecture (visual)

```
┌──────────────────────────────────────────────────────────────────────────────┐
│  Your control loop                                                           │
│    scheduler.reset()           # optional: start from a clean state          │
│    scheduler.step()            # advance time by dt                          │
│    changed, fsm_state = scheduler.tick(FsmAction(MOVE, duration=1.0))        │
│    # use fsm_state.state and fsm_state.progress                              │
└──────────────────────────────────────────────────────────────────────────────┘
                    │
                    ▼
┌──────────────────────────────────────────────────────────────────────────────┐
│  Scheduler (core.scheduler) – abstract base                                  │
│    _dt, _t, _T        # time step, current time, action duration             │
│    reset()            # abstract: reset state and time                       │
│    step()             # abstract: advance _t by _dt                          │
│    tick(action)       # abstract: process action, return (changed?, state)   │
│    _progress_raw(t)   # t / _T (0 if _T==0)                                  │
└──────────────────────────────────────────────────────────────────────────────┘
                    │
                    ▼
┌──────────────────────────────────────────────────────────────────────────────┐
│  FsmScheduler (scheduler.fsm_scheduler) – FSM implementation                 │
│    _state: State      # STOPPED | OPERATING | HOMING | INVALID               │
│    reset()            # _state = STOPPED, _t = 0                             │
│    step()             # _t += _dt                                            │
│    tick(FsmAction)    # lookup next state, update progress, maybe transition │
└──────────────────────────────────────────────────────────────────────────────┘
                    │
                    ▼
┌──────────────────────────────────────────────────────────────────────────────┐
│  FSM logic (same module)                                                     │
│    TRANSITION_TABLE[(State, Action)] → State                                 │
│    get_next_state(current, action) → State                                   │
└──────────────────────────────────────────────────────────────────────────────┘
```

---
►
## State diagram (FSM)

States: **STOPPED**, **OPERATING**, **HOMING**, **INVALID**.  
Actions: **STOP**, **MOVE**, **HOME**.

```
   HOME                                               MOVE
 ┌──────┐    Sensor Home                           ┌────────┐ 
 │      │         or                               │        │
 │  ┌────────┐   STOP  ┌───────────┐  STOP   ┌───────────┐  │
 └─►│ HOMING │◄───────►│  STOPPED  │◄───────►│ OPERATING │◄─┘
    └────────┘  HOME   └───────────┘   MOVE  └───────────┘
        │                                          │
        │                                          │
        │                                          │
        │ MOVE         ┌───────────┐          HOME │
        └─────────────►│  INVALID  │◄──────────────┘
                       └───────────┘
                        (error: invalid transition)
```

**Transition table (summary):**

| Current   | STOP   | MOVE      | HOME    |
|----------|--------|-----------|---------|
| STOPPED  | STOPPED | OPERATING | HOMING  |
| OPERATING| STOPPED | OPERATING | INVALID |
| HOMING   | STOPPED | INVALID   | HOMING  |
| INVALID  | STOPPED | INVALID   | INVALID |

If you use an invalid (current, action) pair (e.g. OPERATING + HOME), the FSM goes to INVALID and `tick()` raises `ValueError`.

**Special rule:** When in HOMING and **progress reaches 1.0**, the state is set to **STOPPED** (homing complete).

---

## Data flow (step and tick)

```
  reset()
    → _state = STOPPED, _t = 0

  step()
    → _t += _dt

  tick(FsmAction(action, duration))
    → _T = duration
    → progress = _progress_raw(_t + _dt)   # 0..1
    → next_state = get_next_state(_state, action)
    → if next_state == INVALID: raise ValueError
    → if next_state != _state:  update _state, return (True, FsmState)
    → if progress == 1.0:       _t = 0; if HOMING then _state = STOPPED; return (True, FsmState)
    → else:                     return (False, FsmState)
```

---

## Module layout

| Module | Contents |
|--------|----------|
| `robot_manager.core.scheduler` | `Scheduler` (ABC): `reset`, `step`, `tick`, `_progress_raw` |
| `robot_manager.scheduler.fsm_scheduler` | `State`, `Action`, `FsmState`, `FsmAction`, `TRANSITION_TABLE`, `get_next_state`, `FsmScheduler` |

---

## Function reference

### Base class: `Scheduler` (core.scheduler)

Abstract base. Subclass and implement `reset()`, `step()`, and `tick(action)`.

---

#### `Scheduler(dt)`

**Purpose:** Create a scheduler with a fixed time step.

**Parameters:**

| Parameter | Type | Description |
|-----------|------|-------------|
| `dt` | `float` | Time step (e.g. in seconds). Each `step()` adds this to internal time `_t`. |

**Internal attributes (for subclasses):** `_dt`, `_t` (current time), `_T` (duration of the current action; 0 if none).

---

#### `reset()` *(abstract)*

**Purpose:** Put the scheduler back to a known initial state (e.g. state = STOPPED, time = 0).

**Parameters:** None.

**Returns:** None.

**Usage:** Call when starting a new run or when you want to “restart” the state machine. In `FsmScheduler`, this sets `_state = STOPPED` and `_t = 0`.

---

#### `step()` *(abstract)*

**Purpose:** Advance internal time by one step (`_dt`).

**Parameters:** None.

**Returns:** None.

**Usage:** Call once per control cycle before or after `tick()`. In `FsmScheduler`, this does `_t += _dt`.

---

#### `tick(action)` *(abstract)*

**Purpose:** Apply an action, update state and progress, and return whether something “meaningful” changed and the new state info.

**Parameters:**

| Parameter | Type | Description |
|-----------|------|-------------|
| `action` | Any | Action to apply (e.g. `FsmAction` with an action type and duration). |

**Returns:** Defined by the subclass. For `FsmScheduler`, returns `(bool, FsmState)`.

**Usage:** Call every cycle (often after `step()`) with the command you want (STOP, MOVE, HOME, etc.). Use the returned state and progress for logic or display.

---

#### `_progress_raw(t) -> float` *(internal)*

**Purpose:** Compute progress for the current action: ratio of time `t` to action duration `_T`.

**Parameters:**

| Parameter | Type | Description |
|-----------|------|-------------|
| `t` | `float` | Time value (e.g. `_t + _dt`). |

**Returns:** `t / _T` if `_T > 0`, else `0.0`. Rounded to 2 decimal places.

**Usage:** Used inside `tick()` to compute progress; you usually don’t call it directly.

---

### FSM types and table (fsm_scheduler)

---

#### `State` (enum)

**Purpose:** Current mode of the machine.

| Member | Value | Meaning |
|--------|--------|--------|
| `STOPPED` | 0 | Robot is stopped. |
| `OPERATING` | 1 | Robot is moving/operating. |
| `HOMING` | 2 | Robot is in a homing motion. |
| `INVALID` | 3 | Invalid transition (error state). |

---

#### `Action` (enum)

**Purpose:** Command you can send to the scheduler.

| Member | Value | Meaning |
|--------|--------|--------|
| `STOP` | 0 | Stop. |
| `MOVE` | 1 | Move / operate. |
| `HOME` | 2 | Run homing. |

---

#### `FsmState` (dataclass)

**Purpose:** Snapshot returned by `tick()`: current state and progress.

| Field | Type | Description |
|-------|------|-------------|
| `state` | `State` | Current state (STOPPED, OPERATING, HOMING, INVALID). |
| `progress` | `float` | Progress of the current action in [0, 1]. 1.0 = action done. |

---

#### `FsmAction` (dataclass)

**Purpose:** Action to pass to `tick()`: what to do and how long it lasts.

| Field | Type | Description |
|-------|------|-------------|
| `action` | `Action` | STOP, MOVE, or HOME. |
| `duration` | `float` | Duration of the action (e.g. in seconds). Used to compute progress. |

**Example:** `FsmAction(Action.MOVE, duration=1.0)` = “move for 1 second”.

---

#### `TRANSITION_TABLE`

**Purpose:** Lookup table: (current state, action) → next state.

**Type:** `dict[(State, Action), State]`.

**Usage:** You usually don’t use it directly; use `get_next_state(current, action)` instead.

---

#### `get_next_state(current, action) -> State`

**Purpose:** Get the next state from the FSM without changing the scheduler.

**Parameters:**

| Parameter | Type | Description |
|-----------|------|-------------|
| `current` | `State` | Current state. |
| `action` | `Action` | Incoming action. |

**Returns:** Next `State`. If the pair is not in the table, returns `State.INVALID`.

**Usage:** Useful for logic or UI that only needs “what would happen,” or for tests.

---

### FsmScheduler (fsm_scheduler)

Concrete scheduler: FSM + time + progress.

---

#### `FsmScheduler(dt)`

**Purpose:** Create an FSM scheduler with time step `dt`.

**Parameters:**

| Parameter | Type | Description |
|-----------|------|-------------|
| `dt` | `float` | Time step (e.g. `0.01` for 10 ms). |

**Initial state:** `_state = STOPPED`, `_t = 0`, `_T = 0`.

---

#### `reset() -> None`

**Purpose:** Reset to STOPPED and clear time.

**Behavior:** Sets `_state = State.STOPPED` and `_t = 0.0`.

**Usage:** Call at start of a run or when you want to “restart” the FSM.

---

#### `step() -> None`

**Purpose:** Advance internal time by `_dt`.

**Behavior:** Does `_t += _dt`.

**Usage:** Call every control cycle (e.g. before or after `tick()`).

---

#### `tick(action: FsmAction) -> (bool, FsmState)`

**Purpose:** Apply the action, update state and progress, and return whether something changed and the new FSM snapshot.

**Parameters:**

| Parameter | Type | Description |
|-----------|------|-------------|
| `action` | `FsmAction` | Must have `.action` (Action) and `.duration` (float). |

**Returns:**

- **bool:** `True` if the state changed or if progress reached 1.0 (including HOMING → STOPPED).
- **FsmState:** New state and progress (0..1).

**Behavior (short):**

1. Set `_T = action.duration`.
2. Compute progress = `_progress_raw(_t + _dt)`.
3. Get next state from `get_next_state(_state, action.action)`.
4. If next state is INVALID → raise `ValueError("Invalid state transition.")`.
5. If next state ≠ current state → update `_state`, return `(True, FsmState(...))`.
6. If progress == 1.0 → set `_t = 0`; if state is HOMING set `_state = STOPPED`; return `(True, FsmState(...))`.
7. Otherwise → return `(False, FsmState(...))`.

**Usage:** Call every cycle with the desired command. Use the boolean to know when to react (e.g. “state changed” or “homing finished”).

---

## Minimal example

```python
from robot_manager.scheduler.fsm_scheduler import (
    FsmScheduler,
    FsmAction,
    FsmState,
    Action,
    State,
)

dt = 0.1
scheduler = FsmScheduler(dt)

# Start from STOPPED, request MOVE for 1 second
changed, fsm = scheduler.tick(FsmAction(Action.MOVE, duration=1.0))
print(fsm.state)   # State.OPERATING
print(fsm.progress)  # e.g. 0.1

# Advance time and tick again (same action)
scheduler.step()
changed, fsm = scheduler.tick(FsmAction(Action.MOVE, duration=1.0))
print(fsm.progress)  # e.g. 0.2

# Later: request STOP
changed, fsm = scheduler.tick(FsmAction(Action.STOP, duration=0.0))
print(fsm.state)   # State.STOPPED
```

**Homing until done:**

```python
scheduler.reset()
scheduler.tick(FsmAction(Action.HOME, duration=1.0))  # enter HOMING
while scheduler._state != State.STOPPED:
    scheduler.step()
    scheduler.tick(FsmAction(Action.HOME, duration=1.0))
# When progress hits 1.0, _state becomes STOPPED
```

---

## Glossary

- **State:** Current “mode” of the scheduler (STOPPED, OPERATING, HOMING, INVALID).
- **Action:** Command you send (STOP, MOVE, HOME).
- **Transition table:** Rules that say: “if in state S and action A, go to state S'.”
- **Progress:** How far the current timed action has gone (0 to 1). At 1.0, the action is considered done (and in HOMING, state goes to STOPPED).
- **tick:** “Process one command and return the new state and progress.”
- **step:** “Advance internal time by one dt.”
