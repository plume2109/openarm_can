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

ctrl = oa.OpenArmController("can0", "/path/to/openarm.urdf",
                             "openarm_body_link0", "openarm_right_hand")
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

ctrl = oa.DualOpenArmController("can0", "can1", "/path/to/openarm.urdf",
                                 "openarm_body_link0", "openarm_right_hand")
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
