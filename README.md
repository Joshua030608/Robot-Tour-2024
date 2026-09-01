# Robot Tour — Science Olympiad

Collaborative Arduino firmware and design references for a Science Olympiad
Robot Tour vehicle. The project uses encoder feedback and a motion queue to
drive a small robot through a programmed course.

## Technical Highlights

- Arduino Uno R3 firmware for dual-motor direction and PWM control.
- Encoder-based position and speed feedback with acceleration/deceleration
  profiles and a proportional-integral control loop.
- Command queue supporting forward and reverse motion, turns, start/abort
  behavior, speed changes, and acceleration changes.
- Calibration logic for motor-encoder differences and repeated-turn accuracy.
- Ultrasonic range-finder support and optional I2C LCD status display.

## Hardware and Setup

The main firmware is [`robot/robot.ino`](robot/robot.ino). To build it:

1. Install the Arduino IDE.
2. Select **Arduino Uno** as the board.
3. Install the `LiquidCrystal_I2C` library.
4. Open `robot/robot.ino`, review the pin assignments and calibration constants,
   then upload to the matching hardware.

The `Robot Design PDFs` directory contains the project wiring and hardware
reference documents. The LCD is disabled by default through `DISPLAY_PRESENT`;
enable it only when the display is connected and configured correctly.

## Repository Layout

```text
robot/              Main firmware plus development sketches
Robot Design PDFs/  Wiring and mechanical hardware references
```

## Credits

This was a collaborative project by **Joshua Ford and Aliza Azhar**. The commit
history documents Joshua's iterations on calibration, movement behavior, sensor
tuning, and competition changes.
