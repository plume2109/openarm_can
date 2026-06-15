# OpenArm CAN Python bindings

Python bindings for the OpenArm CAN library, providing high-level control of one or two OpenArm robots over SocketCAN.

## Install

Build the C++ library first (see [dev/README.md](../dev/README.md)), then:

```bash
cd python
python -m venv venv
source venv/bin/activate
pip install .
```

## Single-arm

```python
import openarm_can as oa

# Default gains
ctrl = oa.OpenArmController("can0", "/path/to/openarm.urdf",
                             "openarm_body_link0", "openarm_right_hand")

# Custom gains (optional) — tunable without recompiling
ctrl = oa.OpenArmController("can0", "/path/to/openarm.urdf",
                             "openarm_body_link0", "openarm_right_hand",
                             kp               = [300, 300, 150, 150, 40, 40, 30],   # position stiffness
                             kd               = [2.5, 2.5, 2.5, 2.5, 0.8, 0.8, 0.8],  # velocity damping
                             grav_kd          = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1],  # gravity comp damping
                             grav_tau_scale   = 1.0,   # scale computed gravity torques (1.0 = full compensation)
                             gripper_max_speed = 10.0,  # rad/s
                             gripper_torque_pu = 0.25)  # gripper force limit [0–1]
ctrl.enable()

state = ctrl.get_joint_state()
# state.positions        → list[float] × 7   (rad)
# state.velocities       → list[float] × 7   (rad/s)
# state.torques          → list[float] × 7   (Nm)
# state.gripper_position → float              (rad)
# state.gripper_torque   → float              (Nm)

ctrl.send_joint_action(positions, gripper_position=0.0)

ctrl.enable_gravity_compensation(gripper_position=0.0)
ctrl.disable_gravity_compensation()

ctrl.disable()
```

## Dual-arm

```python
import openarm_can as oa

# Default gains
ctrl = oa.DualOpenArmController("can0", "can1", "/path/to/openarm.urdf",
                                 "openarm_body_link0", "openarm_right_hand")

# Custom gains (optional)
ctrl = oa.DualOpenArmController("can0", "can1", "/path/to/openarm.urdf",
                                 "openarm_body_link0", "openarm_right_hand",
                                 kp               = [300, 300, 150, 150, 40, 40, 30],
                                 kd               = [2.5, 2.5, 2.5, 2.5, 0.8, 0.8, 0.8],
                                 grav_kd          = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1],
                                 grav_tau_scale   = 1.0,
                                 gripper_max_speed = 10.0,
                                 gripper_torque_pu = 0.25)
ctrl.enable()

state = ctrl.get_joint_state()
# state.positions              → list[float] × 14  (left[0..6], right[7..13], rad)
# state.velocities             → list[float] × 14  (rad/s)
# state.torques                → list[float] × 14  (Nm)
# state.gripper_left_position  → float              (rad)
# state.gripper_left_torque    → float              (Nm)
# state.gripper_right_position → float              (rad)
# state.gripper_right_torque   → float              (Nm)

ctrl.send_joint_action(positions_14,
                       gripper_left_position=0.0,
                       gripper_right_position=0.0)

ctrl.enable_gravity_compensation(gripper_left_position=0.0,
                                 gripper_right_position=0.0)
ctrl.disable_gravity_compensation()

ctrl.disable()
```

Both arms are driven from a single IO thread (~1 kHz) so CAN frames to `can0` and `can1` are issued in the same loop iteration.
