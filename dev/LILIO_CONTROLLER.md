# OpenArm Controller — lili-o move V2

## Context

lili-o move V2 replaces Placo with CuRobo for motion planning. CuRobo handles forward
kinematics, inverse kinematics, and collision-aware trajectory planning on GPU. The
low-level hardware layer must provide these primitives to feed the pipeline:

1. **Get joint state** — read positions, velocities, torques at any step
2. **Gravity compensation** — float the arm so a user can demonstrate a motion
3. **Send joint action** — execute a single joint-space command

All loop logic (recording, trajectory execution, timing) belongs in lili-o move, not
openarm_can. lili-o move must remain hardware-agnostic. The only data crossing the
boundary between lilio_move and openarm_can is joint state (read) and joint commands
(write).

---

## Architecture decisions

### Hardware-agnostic lili-o move

lili-o move V2 must not depend on any specific robot SDK. It defines an abstract
`RobotHardware` interface. `openarm_can` is one implementation; Franka, simulation,
etc. are others. The only data crossing the boundary is joint state and joint commands.

```
lili-o move V2  (CuRobo planning, task logic, recording loops, trajectory loops)
      ↕  joint state / joint commands
openarm_can     (C++ hardware control, gravity comp, state reading)
      ↕  CAN-FD frames
OpenArm robot
```

### Everything self-contained in openarm_can

Gravity compensation requires dynamics computation (gravity torques from URDF).
Rather than delegating this to lili-o move (which would break hardware agnosticism),
it lives entirely inside `openarm_can` using the KDL-based `Dynamics` class.
The URDF path is passed once at construction; after that, all gravity computation
is internal and runs in the background IO thread.

### MIT control only

The Damiao motors support MIT (Model-based Impedance Torque) mode, which sends:

```
τ_cmd = kp·(q_target − q) + kd·(dq_target − dq) + τ_ff
```

- **Position control**: high kp/kd, τ_ff = 0
- **Gravity compensation**: kp = 0, kd = light damping, τ_ff = gravity torques
- **PID**: same MIT command, plus a host-side integral term (`ki`, anti-windup
  clamped) added to τ_ff. `OpenArmController`/`DualOpenArmController` expose
  this as `send_joint_action_pid()` / `reset_integral()`, alongside the
  default MIT-only `send_joint_action()`.

### Python bindings simplified

The existing bindings exposed ~30 low-level classes. With `OpenArmController`,
lili-o move only needs two Python objects: `OpenArmController` and `ArmState`.
All other bindings are internal.

---

## What is added to openarm_can

### New C++ files

```
include/openarm/controller/dynamics.hpp           ← KDL wrapper (gravity, jacobian, FK)
src/openarm/controller/dynamics.cpp
include/openarm/controller/openarm_controller.hpp ← NEW
src/openarm/controller/openarm_controller.cpp     ← NEW
```

### Modified files

```
CMakeLists.txt              ← add KDL / urdf / Eigen dependencies + new sources
python/src/openarm_can.cpp  ← replace low-level bindings with OpenArmController
```

---

## Hardware configuration (hardcoded)

| Joint | Motor | Send ID | Recv ID |
|-------|-------|---------|---------|
| 0 – shoulder pitch | DM8009 | 0x01 | 0x11 |
| 1 – shoulder roll  | DM8009 | 0x02 | 0x12 |
| 2 – shoulder yaw   | DM4340 | 0x03 | 0x13 |
| 3 – elbow flex     | DM4340 | 0x04 | 0x14 |
| 4 – wrist 1        | DM4310 | 0x05 | 0x15 |
| 5 – wrist 2        | DM4310 | 0x06 | 0x16 |
| 6 – wrist 3        | DM4310 | 0x07 | 0x17 |
| gripper            | DM4310 (POS_FORCE) | 0x08 | 0x18 |

MIT gains (position control):

```
kp = [300, 300, 150, 150, 40, 40, 30]
kd = [2.5, 2.5, 2.5, 2.5, 0.8, 0.8, 0.8]
```

Gravity compensation gains:

```
kp = [0, 0, 0, 0, 0, 0, 0]
kd = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1]
```

Gripper POS_FORCE limits: `speed = 10.0 rad/s`, `torque_pu = 0.25`

---

## C++ API

```cpp
// ── Data types ────────────────────────────────────────────────────────────

struct ArmState {
    std::array<double, 7> positions;   // rad
    std::array<double, 7> velocities;  // rad/s
    std::array<double, 7> torques;     // Nm
    double gripper_position;           // rad
    double gripper_torque;             // Nm
};

// ── Controller ───────────────────────────────────────────────────────────

class OpenArmController {
public:
    // can_interface : SocketCAN interface name, e.g. "can0"
    // urdf_path     : path to the OpenArm URDF file
    // root_link     : root link name, e.g. "openarm_body_link0"
    // tip_link      : tip link name,  e.g. "openarm_right_hand"
    OpenArmController(const std::string& can_interface,
                      const std::string& urdf_path,
                      const std::string& root_link,
                      const std::string& tip_link);
    ~OpenArmController();

    // Lifecycle
    void enable();
    void disable();

    // Read — thread-safe, updated at ~1 kHz by the IO thread
    ArmState get_joint_state() const;

    // Write — single joint-space command; call in your own loop at your own dt
    void send_joint_action(const std::array<double, 7>& positions,
                           double gripper_position = 0.0);

    // Gravity compensation mode — runs in background: kp=0, kd=light, τ=gravity
    // The arm floats and can be moved by hand.
    // Call get_joint_state() in your own loop to read the demonstrated motion.
    void enable_gravity_compensation(double gripper_position = 0.0);
    void disable_gravity_compensation();
};
```

### Internal design

- **IO thread**: background `std::thread` calling `refresh_all()` + `recv_all(500 µs)`
  at ~1 kHz, writing to a mutex-protected `ArmState latest_state_`.
- **Gravity comp**: when active, the IO thread also calls `Dynamics::GetGravity()`
  and sends a MIT command (kp=0, kd=0.1, τ=gravity torques) on every cycle.
- **Position control**: `send_joint_action()` calls `apply_mit()` with high kp/kd
  and zero τ_ff — one-shot, non-blocking.

---

## Python API

```python
import openarm_can as oa

ctrl = oa.OpenArmController("can0", "/path/to/openarm.urdf",
                             "openarm_body_link0", "openarm_right_hand")
ctrl.enable()

# Read state
state = ctrl.get_joint_state()
# state.positions        → list[float] len 7  (rad)
# state.velocities       → list[float] len 7  (rad/s)
# state.torques          → list[float] len 7  (Nm)
# state.gripper_position → float              (rad)
# state.gripper_torque   → float              (Nm)

# Single joint command (non-blocking)
ctrl.send_joint_action(positions, gripper_position=0.0)

# Gravity compensation mode
ctrl.enable_gravity_compensation(gripper_position=0.0)
ctrl.disable_gravity_compensation()

ctrl.disable()
```

---

## Interface with lili-o move V2

lili-o move owns all loops. openarm_can exposes only primitives.

```
lilio_move/hardware/
├── base.py      ← abstract RobotHardware
├── openarm.py   ← OpenArmHardware wrapping openarm_can (~20 lines)
├── franka.py    ← future
└── sim.py       ← future
```

### base.py

```python
from abc import ABC, abstractmethod

class RobotHardware(ABC):

    @abstractmethod
    def get_joint_state(self) -> dict:
        """Returns positions (7,), velocities (7,), torques (7,), gripper."""

    @abstractmethod
    def send_joint_action(self, positions, gripper_position: float = 0.0) -> None:
        """Send a single joint-space command."""

    @abstractmethod
    def enable_gravity_compensation(self, gripper_position: float = 0.0) -> None:
        """Start gravity compensation mode (arm floats, can be hand-guided)."""

    @abstractmethod
    def disable_gravity_compensation(self) -> None:
        """Stop gravity compensation mode."""
```

### openarm.py

```python
import openarm_can as oa
from .base import RobotHardware

class OpenArmHardware(RobotHardware):

    def __init__(self, can_interface: str, urdf_path: str,
                 root_link: str, tip_link: str):
        self._ctrl = oa.OpenArmController(can_interface, urdf_path,
                                          root_link, tip_link)
        self._ctrl.enable()

    def get_joint_state(self):
        return self._ctrl.get_joint_state()

    def send_joint_action(self, positions, gripper_position=0.0):
        self._ctrl.send_joint_action(positions, gripper_position)

    def enable_gravity_compensation(self, gripper_position=0.0):
        self._ctrl.enable_gravity_compensation(gripper_position)

    def disable_gravity_compensation(self):
        self._ctrl.disable_gravity_compensation()
```

### lili-o move V2 pipeline usage

```python
from lilio_move.hardware.openarm import OpenArmHardware
import time

robot = OpenArmHardware("can0", "/path/to/openarm.urdf",
                        "openarm_body_link0", "openarm_right_hand")

# Hand-guiding: lili-o move owns the recording loop
robot.enable_gravity_compensation()
demo = []
dt = 0.02  # 50 Hz
while recording:
    state = robot.get_joint_state()
    demo.append(state)
    time.sleep(dt)
robot.disable_gravity_compensation()

# CuRobo plans the trajectory (hardware-agnostic)
trajectory = curobo_planner.plan(
    demo_data=demo,
    current_state=robot.get_joint_state(),
)

# Execution: lili-o move owns the trajectory loop
for positions, gripper in zip(trajectory["positions"], trajectory["gripper"]):
    robot.send_joint_action(positions, gripper)
    time.sleep(dt)
```

---

## Dependencies added to openarm_can

| Library | Purpose | Install |
|---------|---------|---------|
| `orocos_kdl` | Rigid-body dynamics (gravity torques) | `apt install liborocos-kdl-dev` |
| `kdl_parser` | Build KDL tree from URDF | `apt install ros-<distro>-kdl-parser` |
| `urdfdom` | Parse URDF files | `apt install liburdfdom-dev` |
| `Eigen3` | Linear algebra (used by KDL) | `apt install libeigen3-dev` |

---

## What is removed / simplified

| Component | Before | After |
|-----------|--------|-------|
| Python bindings | ~460 lines, 30+ classes | ~60 lines, 2 classes |
| `openarm_driver` repo | required | fully replaced |
| PID control | used in lili-o V1 | dropped, MIT only |
| Config YAML system | used in openarm_driver | dropped, constants in C++ |
| `gravity_compensation_record()` | blocking, mixed concerns | removed — lilio_move owns the loop |
| `follow_joint_trajectory()` | blocking, mixed concerns | removed — lilio_move owns the loop |
| `TrajectoryData` struct | returned by recording | removed — plain list of ArmState |
