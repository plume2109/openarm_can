# OpenArm CAN Library

A C++ library for CAN communication with OpenArm robotic hardware, supporting Damiao motors over CAN/CAN-FD interfaces.
This library is a part of [OpenArm](https://github.com/enactic/openarm/). See detailed setup guide and docs [here](https://docs.openarm.dev/software/can).


## Quick Start

### Prerequisites

- Linux with SocketCAN support
- CAN interface hardware

### 1. Install

#### Ubuntu

* 22.04 Jammy Jellyfish
* 24.04 Noble Numbat

```bash
sudo apt install -y software-properties-common
sudo add-apt-repository -y ppa:openarm/main
sudo apt update
sudo apt install -y \
  libopenarm-can-dev \
  openarm-can-utils
```

#### AlmaLinux, CentOS, Fedora, RHEL, and Rocky Linux

1. Enable [EPEL](https://docs.fedoraproject.org/en-US/epel/). (Not required for [Fedora](https://fedoraproject.org/))
   * AlmaLinux 8 / Rocky Linux 8
     ```bash
     sudo dnf install -y epel-release
     sudo dnf config-manager --set-enabled powertools
     ```
   * AlmaLinux 9 & 10 / Rocky Linux 9 & 10
     ```bash
     sudo dnf install -y epel-release
     sudo crb enable
     ```
   * CentOS Stream 9
     ```bash
     sudo dnf config-manager --set-enabled crb
     sudo dnf install -y https://dl.fedoraproject.org/pub/epel/epel{,-next}-release-latest-9.noarch.rpm
     ```
   * CentOS Stream 10
     ```bash
     sudo dnf config-manager --set-enabled crb
     sudo dnf install -y https://dl.fedoraproject.org/pub/epel/epel-release-latest-10.noarch.rpm
     ```
   * RHEL 8 & 9 & 10
     ```bash
     releasever="$(. /etc/os-release && echo $VERSION_ID | grep -oE '^[0-9]+')"
     sudo subscription-manager repos --enable codeready-builder-for-rhel-$releasever-$(arch)-rpms
     sudo dnf install -y https://dl.fedoraproject.org/pub/epel/epel-release-latest-$releasever.noarch.rpm
     ```
2. Install the package.
   ```bash
   sudo dnf update
   sudo dnf install -y \
     openarm-can-devel \
     openarm-can-utils
   ```

### 2. Setup CAN Interface

Configure your CAN interface using the provided script:

```bash
# CAN 2.0 (default)
openarm-can-configure-socketcan can0

# CAN-FD with 5Mbps data rate
openarm-can-configure-socketcan can0 -fd
```

### 3. CLI Tool

`openarm-can-cli` provides a command-line interface for motor configuration and diagnostics.

```bash
# Configure CAN interface (default: 1Mbps nominal, 5Mbps data, CAN-FD)
openarm-can-cli -i can0 can_configure

# Configure with 1Mbps data rate (Classic CAN)
openarm-can-cli -i can0 can_configure -d 1000000 --no-fd

# Discover motors on the bus
openarm-can-cli -i can0 discover

# Monitor motor status (arm motors 1-8 by default)
openarm-can-cli -i can0 monitor

# Monitor specific motors
openarm-can-cli -i can0 monitor --id 1,2,3
```

Run `openarm-can-cli -h` for full usage.

### 4. C++ Library

```cpp
#include <openarm/can/socket/openarm.hpp>
#include <openarm/damiao_motor/dm_motor_constants.hpp>

openarm::can::socket::OpenArm arm("can0", true);  // CAN-FD enabled
std::vector<openarm::damiao_motor::MotorType> motor_types = {
    openarm::damiao_motor::MotorType::DM4310, openarm::damiao_motor::MotorType::DM4310};
std::vector<uint32_t> send_can_ids = {0x01, 0x02};
std::vector<uint32_t> recv_can_ids = {0x11, 0x12};

openarm.init_arm_motors(motor_types, send_can_ids, recv_can_ids);
openarm.enable_all();
```

See [dev/README.md](dev/README.md) for how to build.

### 4. Python

**Build & Install:**

Please ensure that you install the C++ library first, as `1. Install` or [dev/README.md](dev/README.md).

```bash
cd python

# Create and activate virtual environment (recommended)
python -m venv venv
source venv/bin/activate

pip install .
```

**Single-arm usage:**

```python
import openarm_can as oa

ctrl = oa.OpenArmController("can0", "/path/to/openarm.urdf",
                             "openarm_body_link0", "openarm_right_hand")
ctrl.enable()

# Read joint state (positions, velocities, torques, gripper)
state = ctrl.get_joint_state()
print(state.positions)        # list[float] × 7, rad
print(state.gripper_position) # float, rad

# Send a joint-space command
ctrl.send_joint_action(positions, gripper_position=0.0)

# Gravity compensation — arm floats, can be hand-guided
ctrl.enable_gravity_compensation()
# ... record state.positions in your own loop ...
ctrl.disable_gravity_compensation()

ctrl.disable()
```

**Dual-arm usage:**

```python
import openarm_can as oa

ctrl = oa.DualOpenArmController("can0", "can1", "/path/to/openarm.urdf",
                                 "openarm_body_link0", "openarm_right_hand")
ctrl.enable()

# Read — positions[0..6] = left arm, positions[7..13] = right arm
state = ctrl.get_joint_state()
print(state.positions)              # list[float] × 14, rad
print(state.gripper_left_position)  # float, rad
print(state.gripper_right_position) # float, rad

# Send a joint-space command to both arms simultaneously
ctrl.send_joint_action(positions_14,
                       gripper_left_position=0.0,
                       gripper_right_position=0.0)

# Gravity compensation on both arms simultaneously
ctrl.enable_gravity_compensation()
ctrl.disable_gravity_compensation()

ctrl.disable()
```

Both arms are driven from a single IO thread (~1 kHz), so CAN frames to `can0` and `can1` are issued in the same loop iteration.

### Examples

- **C++**: `examples/demo.cpp` - Complete arm control demo
- **Python**: `python/examples/example.py` - Basic Python usage

## For developers

See [dev/README.md](dev/README.md).

## Related links

- 📚 Read the [documentation](https://docs.openarm.dev/software/can/)
- 💬 Join the community on [Discord](https://discord.gg/FsZaZ4z3We)
- 📬 Contact us through <openarm@enactic.ai>

## License

Licensed under the Apache License 2.0. See `LICENSE.txt` for details.

Copyright 2025 Enactic, Inc.

## Code of Conduct

All participation in the OpenArm project is governed by our [Code of Conduct](CODE_OF_CONDUCT.md).
