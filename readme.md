# AI Driver Car

An autonomous driving robot that uses a Raspberry Pi Zero 2 W to capture images, sends them to an LLM API for visual processing, and relays driving commands to an Arduino over serial to control the car's motors.

## How It Works

1. **Raspberry Pi Zero 2 W** captures images using a connected camera.
2. Images are sent to an **LLM API** (vision model) for analysis.
3. The LLM interprets the scene and decides on a movement direction.
4. The Pi sends a single-character serial command to the **Arduino**.
5. The Arduino drives the motors accordingly.

```
Camera → Pi Zero 2 W → LLM API (vision) → Pi Zero 2 W → Serial → Arduino → Motors
```

## Serial Command Protocol

| Command | Action       |
|---------|--------------|
| `F`     | Move forward |
| `B`     | Move backward|
| `L`     | Turn left    |
| `R`     | Turn right   |
| `S`     | Stop         |
| `T`     | Run test sequence |

The Arduino replies with `A` (acknowledged) on success or `E` on an unrecognized command.

**Baud rate:** 115200

## Hardware

- Raspberry Pi Zero 2 W
- Camera module (connected to the Pi)
- Arduino (Uno, Nano, etc.)
- L9110S motor driver
- 2 DC motors
- USB serial connection between Pi and Arduino

## Pin Wiring (Arduino)

| Pin | Function          |
|-----|-------------------|
| 9   | Left Motor Pin A  |
| 10  | Left Motor Pin B  |
| 5   | Right Motor Pin A |
| 6   | Right Motor Pin B |

## Getting Started

### Arduino

1. Open `arduino_motorControl_Serial.ino` in the Arduino IDE.
2. Upload to your Arduino board.
3. You can test standalone by opening the Serial Monitor at **115200 baud** and sending `T` to run the test sequence.

### Raspberry Pi

1. Connect the camera module to the Pi Zero 2 W.
2. Connect the Pi to the Arduino via USB serial.
3. Run the Python script on the Pi that:
   - Captures an image from the camera.
   - Sends the image to the LLM API.
   - Parses the response for a direction command.
   - Sends the command character over serial to the Arduino.

## Project Structure

```
AI-Driver-Car/
├── arduino_motorControl_Serial.ino   # Arduino motor control firmware
└── readme.md                         # This file
```

## License

MIT