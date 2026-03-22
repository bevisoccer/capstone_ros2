# leap_hand_bridge — ROS 2 Package

Bridges Leap Motion finger tracking to the robot hand servo controller.

## Architecture

```
Leap Motion sensor
       ↓
leap_publisher (C binary, LeapC API)
       ↓ stdout: "CURLS t i m r p GESTURE"
leap_hand_bridge_node (Python ROS 2)
       ↓ /hand/move_all
hand_controller_node (Python ROS 2)
       ↓ serial
Teensy 4.1 + PCA9685
       ↓ PWM
15x MG90S servos (robot hand)
```

## Step 1 — Build the C publisher

```bash
cd ~/ros2_ws/src/leap_hand_bridge/leap_publisher
mkdir build && cd build
cmake ..
make
```

This produces a `leap_publisher` binary. Note the full path — you'll need it below.

> **Leap SDK paths**: The CMakeLists searches common locations. If it can't find
> `LeapC.h`, install the Ultraleap SDK and set the path manually:
> ```bash
> cmake .. -DLEAPC_INCLUDE_DIR=/path/to/sdk/include \
>           -DLEAPC_LIBRARY=/path/to/sdk/lib/libLeapC.so
> ```

## Step 2 — Build the ROS 2 package

```bash
cd ~/ros2_ws
colcon build --packages-select leap_hand_bridge
source install/setup.bash
```

## Step 3 — Run everything

Terminal 1 — hand controller:
```bash
ros2 run hand_controller hand_controller_node --ros-args -p serial_port:=/dev/teensy
```

Terminal 2 — Leap bridge:
```bash
ros2 run leap_hand_bridge leap_hand_bridge_node \
  --ros-args -p leap_publisher_path:=/full/path/to/leap_publisher
```

## Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/hand/move_all` | `Float32MultiArray` | 15 servo angles (published at 60 Hz) |
| `/leap/curls` | `Float32MultiArray` | Raw curl angles: [thumb, index, middle, ring, pinky] |
| `/leap/gesture` | `String` | Detected gesture name |

## Tuning the mapping

In `leap_hand_bridge_node.py`, adjust these constants to match your hand:

```python
CURL_MIN = 5.0    # curl angle when finger is fully open (degrees)
CURL_MAX = 85.0   # curl angle when finger is fully closed (degrees)

SERVO_OPEN   = 180.0   # servo angle for open finger
SERVO_CLOSED = 0.0     # servo angle for closed finger
```

If fingers move backwards, either flip `SERVO_OPEN`/`SERVO_CLOSED` or launch with:
```bash
--ros-args -p invert_mapping:=true
```

## Channel map

| Channels | Finger |
|----------|--------|
| 0, 1, 2  | Pinky  |
| 3, 4, 5  | Ring   |
| 6, 7, 8  | Middle |
| 9, 10, 11| Index (set to -1.0 in code if not wired) |
| 12, 13   | Thumb  |
| 14       | Unused |
