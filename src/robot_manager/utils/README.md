# utils

RRT algorithm, interpolation, distance, steer, and geometry/kinematics helpers. Shared by planner and robot models.

---

## At a glance

| Module | Exports | Use |
|--------|---------|-----|
| **rrt** | `RrtAlgorithm`, `config_distance`, `config_interpolate`, `config_steer`, `copy_metadata`, `interpolate`, `joint_distance`, `quintic_time_scaling`, `steer` | RRT search, trajectory interpolation and time scaling |
| **geometry** | `transformation_matrix` | Transformation matrix (roll/pitch/yaw, etc.) |
| **modern_robotics** | `FKinSpace` | Forward kinematics (space frame) |

---

## RRT-related (rrt)

| Name | Signature / role |
|------|-------------------|
| **RrtAlgorithm** | `run(start, goal, obstacle_state)` → (success, trajectory). No threading. |
| **quintic_time_scaling** | `t in [0,1]` → smooth progress (zero velocity at start/end) |
| **joint_distance** | Euclidean distance between two `JointState` positions |
| **config_distance** | Distance between two `np.ndarray` configs (generic config space) |
| **steer** / **config_steer** | Move one step from start toward target |
| **interpolate** / **config_interpolate** | Linear interpolation between two states/configs |
| **copy_metadata** | Copy `JointState` metadata (id, etc.) |

---

## Where used

- **planner:** `RrtPlanner` uses `RrtAlgorithm` and the helpers above for trajectory generation and eval.
- **robots:** `LittleReader` and others use `transformation_matrix`, `FKinSpace` for FK and kinematics.
