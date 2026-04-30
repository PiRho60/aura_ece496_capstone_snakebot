# Aura - Invasion of the Snake Robot

A low-cost embedded snake robot that demonstrates lateral-undulation locomotion, wireless control, IMU-based heading control, and off-board AprilTag-based target navigation.

The robot uses an ESP32 to drive an 8-servo segmented body. A separate Python/OpenCV program processes camera input, detects AprilTags, computes the target direction, and sends UDP commands to the ESP32.

## Key Features

- 8-segment snake robot driven by servo motors
- Lateral-undulation movement on dry, flat surfaces
- ESP32-based embedded control
- BNO08x/BNO085 IMU heading estimation
- Wi-Fi UDP command interface
- Cardinal direction commands: north, south, east, and west
- Off-board camera processing using OpenCV AprilTag detection
- OpenCV GUI with GO, STOP, QUIT, and direction controls
- Target-relative angle and distance estimation using AprilTags

## Hardware Requirements

- ESP32 development board
- 8 servo motors
- BNO08x/BNO085 IMU
- 8-segment mechanical snake body with wheels/friction elements
- Battery or external power system for the robot electronics and servos
- Camera connected to the off-board computer
- Off-board computer running Python
- AprilTag markers:
  - Tag `1`: snake robot
  - Tag `2`: target object
  - Tags `3`, `4`, `5`, `6`: fixed workspace reference tags

The system assumes operation on a dry, flat surface with clear camera visibility of the tags.

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
│       ├── camera_connect.py
│       └── snakebot/
│           └── snakebot.ino
└── snakebot/
    └── snakebot.ino
```

### Main Files

- `snakebot/snakebot.ino`  
  Main ESP32 firmware for IMU reading, servo control, Wi-Fi UDP communication, and heading-based motion.

- `camera/AprilTag/camera_connect.py`  
  Main off-board control program. Opens the camera feed, detects AprilTags, computes target-relative navigation data, displays the GUI, and sends UDP commands to the ESP32.

- `camera.py`  
  Utility script for opening a camera feed and clicking to inspect pixel coordinates.

- `camera/AprilTag/snakebot/snakebot.ino`  
  A second Arduino sketch variant included under the AprilTag folder. It is very similar to the top-level `snakebot/snakebot.ino`.

## Configuration

Before running the system, update the hard-coded configuration values.

### ESP32 Sketch

In the Arduino sketch, update the Wi-Fi settings:

```cpp
const char* SSID = "...";
const char* EAP_IDENTITY = "...";
const char* EAP_USERNAME = "...";
const char* EAP_PASSWORD = "...";
```

The current sketch is configured for WPA2-Enterprise-style credentials. Adjust the Wi-Fi connection code if using a different network type.

Do not commit private Wi-Fi credentials to a public repository.

The default UDP port is:

```cpp
constexpr uint16_t UDP_PORT = 6657;
```

The default servo pins are:

```cpp
int servoPins[NUM_SERVOS] = { 33, 32, 22, 21, 17, 16, 14, 13 };
```

The BNO08x IMU pins are configured as:

```cpp
static constexpr int PIN_BNO_CS = 27;
static constexpr int PIN_BNO_INT = 26;
static constexpr int PIN_BNO_RST = 25;
```

### Python AprilTag Controller

In `camera/AprilTag/camera_connect.py`, update:

```python
CAM_INDEX = 1
ESP_IP = "..."
ESP_PORT = 6657
```

Also verify that the reference tag positions match the physical workspace:

```python
WORLD_REF_METERS = {
    3: (0.00, 0.00),
    4: (2.667, 0.00),
    5: (0.00, 3.1),
    6: (2.667, 3.1),
}
```

## Build and Flash

1. Open `snakebot/snakebot.ino` in the Arduino IDE.
2. Select the correct ESP32 board and serial port.
3. Install the required libraries:
   - `ESP32Servo`
   - `Adafruit BNO08x`
4. Update the Wi-Fi credentials and pin configuration if needed.
5. Upload the sketch to the ESP32.
6. Open the Serial Monitor at `115200` baud to check startup messages.

Expected startup messages include IMU initialization, Wi-Fi connection status, and UDP listener status.

## Run the Camera Controller

From the repository root:

```bash
python camera/AprilTag/camera_connect.py
```

The program opens a full-screen OpenCV control window. It detects the AprilTags, computes the target direction and distance, and sends commands to the ESP32 over UDP.

Controls:

- `GO`: send a target-relative movement command
- `STOP`: stop the robot and center the servos
- `N`, `S`, `E`, `W`: command cardinal-direction movement
- `QUIT`, `q`, or `Esc`: stop and exit

To test the camera feed only:

```bash
python camera.py
```

In `camera.py`, use `+` or `=` to zoom in, `-` to zoom out, `c` to clear the click marker, and `Esc` to exit.

## Basic Operation

1. Place the robot on a dry, flat surface.
2. Place the four reference AprilTags at the configured workspace corners.
3. Mount or place the snake tag on the robot.
4. Place the target tag at the desired target location.
5. Power on the robot.
6. Let the ESP32 complete its warmup period.
7. Start the Python controller.
8. Confirm that the GUI detects the reference tags, snake tag, and target tag.
9. Use the GUI to send direction, stop, or target-navigation commands.

For heading initialization, the robot should be placed in the intended starting orientation before movement begins.

## Testing and Verification

Useful checks before full operation:

- Confirm the Serial Monitor shows that the BNO08x IMU was detected.
- Confirm the ESP32 connects to Wi-Fi and starts listening on UDP port `6657`.
- Run `camera.py` to verify the camera index and video feed.
- Run `camera/AprilTag/camera_connect.py` and confirm the GUI detects tags `1` through `6`.
- Press `STOP` and verify that all servos center.
- Press `N`, `S`, `E`, or `W` and verify that the robot begins moving toward the selected heading.
- Press `GO` with the target tag visible and verify that the GUI reports a target angle and distance.

The project report verified forward locomotion, 8-segment movement, cardinal-direction movement, target-direction estimation, wireless operation, and GUI-based user input within the intended test environment.

## Known Limitations and Assumptions

- The robot is intended for dry, flat indoor surfaces.
- The AprilTag navigation system requires clear visibility of the reference, snake, and target tags.
- Lighting, camera angle, and tag placement affect localization quality.
- The system relies on an external camera and off-board computer for target navigation.
- Wi-Fi and off-board processing introduce latency.
- The IMU heading approach avoids relying on the magnetometer during operation, so heading drift may accumulate over time.
- Several configuration values are hard-coded, including Wi-Fi settings, ESP32 IP address, camera index, tag IDs, and workspace dimensions.
- No automated test suite, `requirements.txt`, PlatformIO project, or Makefile is included.
- The Python controller sends target distance in the UDP message, but the embedded sketch primarily uses the target direction for motion control.

## Contributors

- Wenhao Qu
- Xiwei Huang
- Mark Parvez
- Nantong Li

Supervisor: Manfredi Maggiore
