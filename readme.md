# AI Driver Car

An autonomous driving robot with a **hybrid architecture**:

- **Raspberry Pi**: Uses a camera and Azure OpenAI Vision LLM to decide the general direction (navigate). It continuously captures images, gets a driving command, and sends it to the Arduino.
- **Arduino**: Handles all real-time driving and obstacle avoidance using a single front-facing ultrasonic sensor. It has its own safety and evasion logic, including vetoing unsafe commands from the Pi.

**LLM = Navigator** (slow, smart) — provides high-level directional goals.
**Arduino = Driver** (fast, reactive) — executes commands, ensures smooth movement, and performs reflex-like obstacle avoidance.

---

## System Architecture

```
                               ┌────────────────────────────┐
                               │     Raspberry Pi           │
                               │                            │
┌───────────┐  rpicam-still    │ 1. Capture image           │
│ Pi Camera │───────────────►  │ 2. Get F/B/L/R from LLM    │
└───────────┘                  │ 3. Send command to Arduino │
                               │    (S on error/exit)       │
                               └───────────┬────────────────┘
                                           │
                                           │ USB Serial (115200)
                                           ▼
                 ┌──────────────────────────────────────────────────┐
┌──────────┐     │                     Arduino                      │
│ HC-SR04  ├───► │ • Executes F, B, L, R, S commands                │
└──────────┘     │ • Vetoes 'F' if path is blocked (<50cm)          │
                 │ • When turning, seeks a clear gap (>90cm)        │
                 │ • Auto-switches to 'F' when gap is found         │
                 │ • Sends sensor telemetry back to Pi (D:<cm>)     │
                 └───────────┬──────────────────────┬───────────────┘
                             │                      │
                             │                      │
                             ▼                      ▼
                        ┌───────────┐          ┌───────────┐
                        │ Left Motor│          │Right Motor│
                        └───────────┘          └───────────┘
```

---

<details>
<summary><strong>How the Arduino Firmware Works (.ino)</strong></summary>

### Arduino Logic Overview

The Arduino acts as a smart motor controller with its own set of reflexes.

-   **Startup:** Initializes motors, ultrasonic sensor, and an I2C LCD.
-   **Main Loop:**
    1.  **Listen for Commands:** Continuously checks for a single-character command (`F`, `B`, `L`, `R`) from the Raspberry Pi over the serial connection.
    2.  **Read Sensor:** Measures the distance to obstacles in front using the ultrasonic sensor every 50ms and sends the telemetry back to the Pi (e.g., `D:120`).
    3.  **Apply Safety Timeouts:** If no command is received from the Pi for over 5 seconds, it assumes the LLM is "thinking" and decelerates. If no command is received for 10 seconds, it performs an emergency stop, assuming the connection is lost.
    4.  **Execute Smart Driving Logic:**
        -   **Hardware Veto:** If the Pi sends `'F'` (Forward) but an obstacle is closer than `VETO_DIST` (50cm), the Arduino ignores the command and instead initiates a turn based on its last successful turn direction (`lastTurnBias`).
        -   **Gap Seeking:** When commanded to turn (`'L'` or `'R'`), it enters a `seeking` mode. It will keep turning in that direction until the sensor reports a clear path of at least `CLEAR_DIST` (90cm).
        -   **Auto-Forward:** Once a clear gap is found while seeking, it automatically cancels the turn and proceeds forward, reporting the status on the LCD.
        -   **Dynamic Braking:** As the robot moves forward, it automatically slows down if an obstacle comes within `SLOW_DIST` (50cm).

### Key Behaviors

-   **`lastTurnBias`:** The Arduino remembers the last direction it turned (`L` or `R`). This is used to make smarter decisions when it needs to override the Pi, preventing it from getting stuck by turning back and forth.
-   **`seeking` Flag:** A state that locks the Arduino into a turn until a clear path is found. This prevents the LLM from interrupting a turn sequence with a different command.
-   **Timeouts:** A critical safety feature that ensures the robot doesn't continue indefinitely if the Pi or the LLM crashes or gets delayed.

</details>

---

## Serial Protocol

The communication between the Pi and Arduino is simple and effective.

**Baud Rate:** 115200

**Pi → Arduino:**
-   `F`, `B`, `L`, `R`: The primary driving commands returned by the LLM.
-   `S`: A command sent by the Raspberry Pi program to stop the motors. The Pi sends this on startup, on clean shutdown (Ctrl-C), or when an error occurs (e.g., the LLM is unavailable or returns an invalid command).

**Arduino → Pi:**
-   `D:<distance_cm>`: Telemetry message sent every 50ms, reporting the distance measured by the front ultrasonic sensor. Example: `D:75`.
-   `A`: Acknowledgement sent back to the Pi after receiving a valid command.

---

## Project Structure

```
AI-Driver-Car/
├── main.py                              # Pi: Main loop for capture → LLM → command
├── .env.example                         # Example for environment variables
├── readme.md
├── modules/
│   ├── cam.py                           # Pi: Image capture via rpicam-still
│   ├── openai_vision.py                 # Pi: Azure OpenAI Vision call logic
│   └── arduino_motorControl_Serial/
│       └── arduino_motorControl_Serial.ino  # Arduino: All motor/sensor/reflex logic
└── images/
    └── image.jpg                        # Overwritten with the latest captured frame
```

---

## Hardware Setup & Wiring

This guide assumes a standard Arduino Uno board and a Raspberry Pi. Pinouts may vary on other boards.

### Core Connections

| From            | To              | Method                                    | Purpose                                     |
| --------------- | --------------- | ----------------------------------------- | ------------------------------------------- |
| **Raspberry Pi**  | **Arduino**     | USB Cable (A to B)                        | Serial communication for sending commands   |
| **Raspberry Pi**  | **Pi Camera**   | CSI Ribbon Cable                          | Video feed for the LLM                      |
| **Motor Driver**  | **Battery**     | DC Barrel Jack or VIN/GND                 | Power for motors                            |
| **Arduino**       | **Motor Driver**| 5V/GND and VIN/GND (see note)             | Power for the Arduino and the driver logic  |

**Note on Power:** You can power the Arduino from the motor driver board, or power the Arduino separately via USB and provide common ground (`GND`) to the motor driver.

### Arduino Pin Connections

#### Motor Controller (L9110S)

The four pins below control the speed and direction of the two motors.

| Motor      | Arduino Pin | Connected To on L9110S |
| ---------- | ----------- | ----------------------- |
| **Motor 1** (e.g., Right) | `PIN 9`     | `A-IA`                  |
|            | `PIN 10`    | `A-IB`                  |
| **Motor 2** (e.g., Left)  | `PIN 5`     | `B-IA`                  |
|            | `PIN 6`     | `B-IB`                  |

#### Ultrasonic Sensor (HC-SR04)

| Sensor Pin | Arduino Pin |
| ---------- | ----------- |
| `VCC`      | `5V`        |
| `GND`      | `GND`       |
| `Trig`     | `PIN 7`     |
| `Echo`     | `PIN 8`     |

#### I2C LCD (Optional, 16x2)

The code supports an I2C LCD for status display.

| LCD Pin | Arduino Pin (Uno) |
| ------- | ------------------- |
| `VCC`   | `5V`                |
| `GND`   | `GND`               |
| `SDA`   | `A4`                |
| `SCL`   | `A5`                |

---

## Quick Start Guide

### 1. Prerequisites

-   Python 3.9+ on the Raspberry Pi.
-   `rpicam-still` command-line tool (standard on Raspberry Pi OS).
-   Arduino IDE to flash the `.ino` firmware.
-   An Azure OpenAI resource with a GPT-4 Vision-capable model deployed.
-   1 x HC-SR04 ultrasonic sensor.

### 2. Flash the Arduino

1.  Open `modules/arduino_motorControl_Serial/arduino_motorControl_Serial.ino` in the Arduino IDE.
2.  Select your board (e.g., Arduino Uno) and the correct serial port.
3.  Upload the sketch.
4.  You can test the sensor by opening the Serial Monitor at **115200 baud**. You should see `D:<cm>` messages printed every 50ms.

### 3. Set Up the Raspberry Pi

Follow these steps on your Raspberry Pi's command line to download the code and prepare the environment.

**Step 1: Download the Code**

First, clone the project repository from GitHub to your home directory. You will need `git` installed for this.

```bash
# Clone the repository (replace with your fork's URL if needed)
git clone https://github.com/HamzahJE/AI-Guided-Robot.git ~/AI-Driver-Car
cd ~/AI-Driver-Car
```

**Step 2: Create a Python Virtual Environment**

It's a best practice to keep project dependencies isolated. A virtual environment creates a self-contained space for the Python packages this project needs, without affecting the system's global packages.

```bash
# Create a virtual environment named 'venv'
python3 -m venv venv
```

**Step 3: Activate the Virtual Environment**

Before you can use it, you must "activate" the environment. Your shell prompt will usually change to show the active environment's name.

```bash
# Activate the 'venv' environment
source venv/bin/activate

# To deactivate later, simply run:
# deactivate
```

**Step 4: Install Dependencies**

With the virtual environment active, you can now install the required Python libraries using `pip`. These libraries are:
- `openai`: To communicate with the Azure OpenAI API.
- `python-dotenv`: To manage your API keys securely in a `.env` file.
- `pyserial`: To handle the serial communication with the Arduino.

```bash
# Install the required Python packages
pip install openai python-dotenv pyserial
```

After these steps, your Raspberry Pi is set up and ready for you to configure your environment variables.

### 4. Configure Environment Variables

Create a file named `.env` in the project root by copying the example:

```bash
cp .env.example .env
```

Now edit the `.env` file with your Azure credentials:

```env
# Azure OpenAI Credentials
OPENAI_API_KEY="your-azure-openai-key"
OPENAI_API_BASE="https://your-base-url/"
OPENAI_ORGANIZATION="your-org-id" # Optional, if applicable
API_VERSION="your-api-version"
MODEL="your-deployment-name" # e.g., gpt-4-vision
```

### 5. Run the Full System

1.  Connect the Pi Camera module to the Raspberry Pi.
2.  Connect the Arduino to the Raspberry Pi via USB.
3.  The `SERIAL_PORT` in `main.py` is defaulted to `/dev/ttyACM0`. If your Arduino is on a different port, find it with `ls /dev/tty*` and update the script.
4.  Launch the main program:

    ```bash
    python main.py
    ```
5.  The car will start its autonomous driving loop. The Pi will continuously capture images, get commands from the LLM, and send them to the Arduino, which executes them while handling real-time obstacle avoidance.
6.  Press **Ctrl-C** in the terminal to stop the program. This will safely send a `S` (Stop) command to the Arduino before exiting.

### Expected Output

```
[serial] Connected to Arduino on /dev/ttyACM0 @ 115200
[cam_thread] Starting async camera capture...
[main] Starting AI driving loop. Arduino handles reflexes, Pi handles strategy.
[arduino] ACK — goal 'S' applied
[llm] Processing frame. Last action was 'S', front dist: -1cm
[llm] New goal: F
[arduino] ACK — goal 'F' applied
[llm] Processing frame. Last action was 'F', front dist: 152cm
...
(Loop continues)
...
[llm] Processing frame. Last action was 'F', front dist: 45cm
[llm] New goal: R
[arduino] ACK — goal 'R' applied
```

---

## Tuning Constants (in .ino file)

You can fine-tune the robot's behavior by adjusting these constants in the `arduino_motorControl_Serial.ino` sketch.

| Constant            | Default | Description                                                                |
| ------------------- | ------- | -------------------------------------------------------------------------- |
| `STOP_DIST`         | 30 cm   | Hard stop distance. The robot will stop if it gets this close.             |
| `SLOW_DIST`         | 50 cm   | Distance at which the robot begins to slow down when moving forward.       |
| `VETO_DIST`         | 50 cm   | The Arduino will reject a forward (`F`) command if an obstacle is closer than this. |
| `CLEAR_DIST`        | 90 cm   | Distance required for the Arduino to consider a path "clear" and exit seeking mode. |
| `BASE_SPEED`        | 200     | Default PWM motor speed (0-255).                                           |
| `MIN_SPEED`         | 90      | The slowest speed the motors will run at during dynamic braking.           |
| `TURN_SPEED`        | 180     | PWM speed used during turns.                                               |
| `SENSOR_INTERVAL`   | 50 ms   | How often the ultrasonic sensor is read.                                   |
| `COMMAND_TIMEOUT`   | 5000 ms | Time without a Pi command before the car slows down.                       |
| `EMERGENCY_TIMEOUT` | 10000 ms| Time without a Pi command before the car performs a full emergency stop.   |

---

## License

MIT
