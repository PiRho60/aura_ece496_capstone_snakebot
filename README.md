# Invasion of the Snake Robot

A low-cost embedded snake robot project that demonstrates lateral-undulation locomotion, wireless control, IMU-based heading control, and off-board AprilTag-based target-direction estimation.

The robot is controlled by an ESP32 that drives an 8-servo segmented body. A separate Python/OpenCV program processes camera input, detects AprilTags, estimates the robot and target positions, displays a control GUI, and sends UDP commands to the ESP32.

## Key Features

- 8-segment servo-driven snake robot
- Lateral-undulation movement on dry, flat surfaces
- ESP32-based embedded firmware
- BNO08x/BNO085 IMU heading sensing using the game rotation vector
- Wi-Fi UDP command interface
- Cardinal direction commands: north, south, east, and west
- Off-board AprilTag detection using OpenCV
- OpenCV GUI with GO, STOP, QUIT, and directional controls
- Target-relative angle and distance estimation from AprilTag positions

## Hardware Requirements

- ESP32 development board
- 8 servo motors
- BNO08x/BNO085 IMU
- 8-segment mechanical snake body with wheels or directional-friction elements
- Battery or external power system for the ESP32, IMU, and servos
- Off-board computer running Python
- Camera connected to the off-board computer
- AprilTag markers:
  - Tag `1`: snake robot
  - Tag `2`: target object
  - Tags `3`, `4`, `5`, `6`: fixed workspace reference tags

The system assumes a dry, flat operating surface with clear camera visibility of the tags.

## Software Requirements

### Embedded

- Arduino IDE or Arduino CLI
- ESP32 Arduino board support
- Arduino libraries:
  - `ESP32Servo`
  - `Adafruit BNO08x`

### Off-board Python

- Python 3
- OpenCV with ArUco/AprilTag support
- NumPy

Install the Python dependencies with:

```bash
pip install opencv-contrib-python numpy
```

## Repository Structure

```text
.
├── README.md
├── camera.py
├── camera/
│   └── AprilTag/
│       └── camera_connect.py
└── snakebot/
    └── snakebot.ino
```

## Main Files

- `snakebot/snakebot.ino`  
  ESP32 firmware for IMU reading, servo control, Wi-Fi connection, UDP command handling, and heading-based locomotion.

- `camera/AprilTag/camera_connect.py`  
  Main off-board controller. Opens the camera feed, detects AprilTags, computes target-relative navigation data, displays the GUI, and sends UDP commands to the ESP32.

- `camera.py`  
  Utility script for checking a camera feed and clicking on the image to print pixel coordinates.

## Configuration

Before running the system, update the hard-coded configuration values.

### ESP32 Firmware

In `snakebot/snakebot.ino`, update the Wi-Fi credentials:

```cpp
const char* SSID = "YOUR_WIFI_SSID";
const char* EAP_IDENTITY = "YOUR_EAP_IDENTITY";
const char* EAP_USERNAME = "YOUR_EAP_USERNAME";
const char* EAP_PASSWORD = "YOUR_EAP_PASSWORD";
```

The current firmware is written for a WPA2-Enterprise-style network. If using a standard WPA/WPA2 personal network, update `connectWiFi()` accordingly.

The UDP port is:

```cpp
constexpr uint16_t UDP_PORT = 6657;
```

The default servo pins are:

```cpp
int servoPins[NUM_SERVOS] = { 33, 32, 22, 21, 17, 16, 14, 13 };
```

The BNO08x IMU pins are:

```cpp
static constexpr int PIN_BNO_CS = 27;
static constexpr int PIN_BNO_INT = 26;
static constexpr int PIN_BNO_RST = 25;
```

### Python Controller

In `camera/AprilTag/camera_connect.py`, update:

```python
CAM_INDEX = 1
ESP_IP = "YOUR_ESP32_IP"
ESP_PORT = 6657
```

The ESP32 IP address is printed in the Arduino Serial Monitor after Wi-Fi connection.

Verify that the reference tag positions match the physical workspace:

```python
WORLD_REF_METERS = {
    3: (0.00, 0.00),
    4: (2.667, 0.00),
    5: (0.00, 3.1),
    6: (2.667, 3.1),
}
```

The camera controller requests a high-resolution camera stream:

```python
REQ_W, REQ_H = 3840, 2160
```

Adjust these values if the camera or computer does not support them.

## Build and Flash

1. Open `snakebot/snakebot.ino` in the Arduino IDE.
2. Select the correct ESP32 board and serial port.
3. Install the required Arduino libraries:
   - `ESP32Servo`
   - `Adafruit BNO08x`
4. Update Wi-Fi credentials and pin assignments if needed.
5. Upload the sketch to the ESP32.
6. Open the Serial Monitor at `115200` baud.

Expected startup output includes BNO08x initialization, Wi-Fi connection status, the ESP32 IP address, and UDP listener status.

## Run the Camera Controller

From the repository root:

```bash
python camera/AprilTag/camera_connect.py
```

The controller opens a full-screen OpenCV window. It detects AprilTags, displays robot/target navigation data, and sends commands to the ESP32 over UDP.

GUI controls:

- `GO`: send a target-relative heading command
- `STOP`: stop motion and center the servos
- `N`, `S`, `E`, `W`: send cardinal-direction commands
- `QUIT`: send stop and exit

Keyboard controls:

- `q` or `Esc`: send stop and exit

To test the camera feed only:

```bash
python camera.py
```

In `camera.py`:

- `+` or `=`: zoom in
- `-`: zoom out
- `c`: clear the click marker
- `Esc`: exit

## Basic Operation

1. Place the robot on a dry, flat surface.
2. Place the four reference AprilTags at the configured workspace corners.
3. Mount or place tag `1` on the robot.
4. Place tag `2` at the target location.
5. Power on the robot.
6. Let the ESP32 complete its startup and warmup period.
7. Start the Python controller.
8. Confirm that the GUI detects the reference tags, snake tag, and target tag.
9. Use the GUI to send direction, stop, or target-heading commands.

For cardinal-direction control, place the robot in the intended reference orientation before operation. The firmware uses the IMU game rotation vector, so north/east/south/west are relative to the startup heading reference rather than true compass north.

## UDP Command Interface

The ESP32 firmware accepts text-based UDP commands on port `6657`:

```text
cmd=STOP
cmd=NORTH
cmd=SOUTH
cmd=EAST
cmd=WEST
cmd=GO,direction=<degrees>,distance_m=<meters>
```

For `GO`, the firmware currently uses `direction` to compute a target heading relative to the current IMU heading. The Python controller also sends `distance_m`, but the current firmware does not use distance for automatic stopping.

## Testing and Verification

Useful checks before full operation:

- Confirm the Serial Monitor shows `BNO08x found!`.
- Confirm the ESP32 connects to Wi-Fi and prints its IP address.
- Confirm UDP starts on port `6657`.
- Run `camera.py` to verify the camera index and video feed.
- Run `camera/AprilTag/camera_connect.py` and confirm tags `1` through `6` are detected.
- Press `STOP` and verify that all servos center.
- Press `N`, `S`, `E`, or `W` and verify that the robot begins moving toward the selected heading.
- Press `GO` with the target tag visible and verify that the GUI reports target angle and distance.
- Check the Python terminal for `[UDP SENT]` messages and the Arduino Serial Monitor for received UDP packets.

## Known Limitations and Assumptions

- The robot is intended for dry, flat indoor surfaces.
- AprilTag-based navigation requires stable lighting and clear visibility of the reference, robot, and target tags.
- Camera angle, tag placement, and calibration affect localization accuracy.
- Target navigation currently sends a heading command; the firmware does not use `distance_m` to stop automatically at the target.
- Cardinal directions are relative to the IMU startup reference, not absolute magnetic north.
- IMU heading drift may accumulate over time.
- Wi-Fi and off-board camera processing introduce latency.
- UDP commands are not authenticated; use the system only on a trusted network.
- Several values are hard-coded, including Wi-Fi settings, ESP32 IP address, camera index, tag IDs, workspace dimensions, servo pins, and IMU pins.
- The OpenCV GUI uses hard-coded button positions intended for the configured camera/window resolution.
- No automated test suite or CI configuration is included.

## Contributors

- Wenhao Qu
- Xiwei Huang
- Mark Parvez
- Nantong Li
