# hand_controller — ROS 2 Package

Controls the robot hand via **Teensy 4.1 + PCA9685** (15× MG90S servos) over USB serial.

---

## Prerequisites

1. **Teensy is flashed** with `servo_controller.ino` (included in the zip).
2. **Teensy is plugged into the Jetson** via USB (usually shows up as `/dev/ttyACM0`).
3. **pyserial** is installed in your ROS environment:
   ```bash
   pip3 install pyserial
   ```

---

## Install into your workspace

```bash
# Copy this package into your workspace src folder
cp -r hand_controller ~/your_ws/src/

cd ~/your_ws
colcon build --packages-select hand_controller
source install/setup.bash
```

---

## Verify the serial port

```bash
ls /dev/ttyACM*
# or
python3 -c "import serial.tools.list_ports; print([p.device for p in serial.tools.list_ports.comports()])"
```

If it shows up as `/dev/ttyACM1` or similar, override the parameter when launching (see below).

### Udev rule (optional but recommended)

Give the Teensy a stable device name so it's always `/dev/teensy`:

```bash
# Find the idVendor / idProduct
udevadm info -a -n /dev/ttyACM0 | grep -E "idVendor|idProduct" | head -2

# Create rule (example values for Teensy 4.1):
echo 'SUBSYSTEM=="tty", ATTRS{idVendor}=="16c0", ATTRS{idProduct}=="0483", SYMLINK+="teensy"' \
  | sudo tee /etc/udev/rules.d/99-teensy.rules
sudo udevadm control --reload-rules && sudo udevadm trigger
```

Then use `serial_port:=/dev/teensy` as your parameter.

---

## Run the node

```bash
# Default port /dev/ttyACM0
ros2 run hand_controller hand_controller_node

# Custom port
ros2 run hand_controller hand_controller_node --ros-args -p serial_port:=/dev/ttyACM1

# Custom port + slower timeout
ros2 run hand_controller hand_controller_node \
  --ros-args -p serial_port:=/dev/ttyACM0 -p ready_timeout:=30.0
```

---

## Topics

### Subscribe (send commands TO the hand)

| Topic | Type | Description |
|-------|------|-------------|
| `/hand/move` | `std_msgs/Float32MultiArray` | Pairs of `[ch, angle, ch, angle …]` |
| `/hand/move_all` | `std_msgs/Float32MultiArray` | 15 floats (one per channel). Use `-1.0` to skip a channel. |
| `/hand/estop` | `std_msgs/Bool` | `True` = ESTOP, `False` = resume all |
| `/hand/speed` | `std_msgs/Float32` | Slew rate in degrees/second (5–720) |

### Publish (read state FROM the hand)

| Topic | Type | Description |
|-------|------|-------------|
| `/hand/angles` | `std_msgs/Float32MultiArray` | 15 current angles at ~10 Hz |
| `/hand/ready` | `std_msgs/Bool` | Latched `True` once Teensy boots and homes |

---

## Quick tests from the terminal

```bash
# Move channel 0 (pinky) to 45°
ros2 topic pub --once /hand/move std_msgs/msg/Float32MultiArray "{data: [0, 45.0]}"

# Move multiple channels at once
ros2 topic pub --once /hand/move std_msgs/msg/Float32MultiArray "{data: [0, 45.0, 3, 90.0, 9, 120.0]}"

# Move ALL channels (use -1.0 to leave a channel unchanged)
ros2 topic pub --once /hand/move_all std_msgs/msg/Float32MultiArray \
  "{data: [45.0, 90.0, 90.0, 90.0, 90.0, 90.0, 90.0, 90.0, 90.0, 90.0, 90.0, 90.0, 45.0, 90.0, 90.0]}"

# Emergency stop
ros2 topic pub --once /hand/estop std_msgs/msg/Bool "{data: true}"

# Resume after ESTOP
ros2 topic pub --once /hand/estop std_msgs/msg/Bool "{data: false}"

# Set speed to 30 degrees/second
ros2 topic pub --once /hand/speed std_msgs/msg/Float32 "{data: 30.0}"

# Watch current angles
ros2 topic echo /hand/angles
```

---

## Channel map (matches servo_controller.ino)

| Channel | Joint     |
|---------|-----------|
| 0       | Pinky     |
| 1–2     | Pinky aux |
| 3–4     | Ring      |
| 5       | Ring aux  |
| 6       | Middle    |
| 7–8     | Middle aux|
| 9       | Index     |
| 10–11   | Index aux |
| 12      | Thumb     |
| 13      | Thumb aux |
| 14      | Unused    |

Edit `INIT_ANGLE`, `JOINT_MIN`, `JOINT_MAX` in `servo_controller.ino` if you need to adjust safe limits.

---

## Package structure

```
hand_controller/
├── hand_controller/
│   ├── __init__.py
│   ├── servo_controller.py       # Serial driver (no ROS dependency)
│   └── hand_controller_node.py   # ROS 2 node
├── resource/hand_controller
├── package.xml
├── setup.py
├── setup.cfg
└── README.md
```
