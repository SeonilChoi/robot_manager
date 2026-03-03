# robot_manager package

Main package: `RobotManager` and `JointState`.

- **robot_manager** – Loads config, owns the robot instance, exposes `initialize()`, `control()`, `update()`.
- **core** – Robot base class, scheduler/planner interfaces, joint state, config types.
- **scheduler** – FSM and other scheduler implementations.
- **planner** – Motion planners (e.g. RRT, PRM).
- **robots** – Concrete robots (e.g. LittleReader).
