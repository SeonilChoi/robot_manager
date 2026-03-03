# robot_manager package

- **robot_manager** – `RobotManager`, `JointState`; loads YAML config, owns robot, exposes `initialize()`, `control()`, `update()`.
- **core** – `Robot` (ABC), `RobotConfig`, `JointState`, `Scheduler`, `Planner`; enums and converters for scheduler/planner types.
- **scheduler** – `FsmScheduler`, FSM states/actions and transition table.
- **planner** – Motion planner implementations (e.g. RRT, PRM); implement `core.planner.Planner`.
- **robots** – Concrete robots (e.g. `LittleReader`) subclasses of `Robot`.
