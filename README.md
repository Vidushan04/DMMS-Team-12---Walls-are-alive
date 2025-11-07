# DMMS-Team-12---Walls-are-alive
"The Walls Are Alive" project successfully progressed the development of a human-sized autonomous robotic platform from a preliminary prototype to a fully integrated autonomous system. The core objective was to enhance autonomous navigation, human detection, interactivity and manual control for deployment in a contemporary art gallery installation

System Type: Autonomous Obstacle-Avoidance Platform
Microcontrollers: Keyestudio 2560 WIFI PLUS Main Board (Arduino MEGA) + Raspberry Pi (vision / audio)
Motor Driver: 24V 7A Dual-Channel DC Motor Driver – MakerBotics (12 V DC motors)
Sensors: 5 × HC-SR04 Ultrasonic Sensors
Additional Peripherals: Raspberry Pi Camera + USB Speaker (Human Detection + Voice Interaction)

The DMMS rover combines Arduino-based motor and sensor control with a Raspberry Pi running computer-vision and audio feedback.
The Arduino controls the motors and obstacle-avoidance logic, while the Pi detects nearby humans, and greets them through a speaker.

| Module             | Component / Power Source                                   | Arduino Pin                                                                     | Notes                                                 |
| ------------------ | -----------------------------------------------------------| ------------------------------------------------------------------------------- | ----------------------------------------------------- |
| **Motors**         | Motor power from 15 V battery (via buck converter 12V)     | ENA → D2 (Right PWM), ENB → D3 (Left PWM), IN1–IN4 → D34–D37                    | Motor controller logic (5 V + GND) powered by Arduino |
| **Ultrasonics**    | 5 V from Arduino                                           | L-OUT (22 / 23), L-IN (24 / 25), CTR (26 / 27), R-IN (28 / 29), R-OUT (30 / 31) | Arranged in semicircle: left-to-right field coverage  |
| **Arduino**        | Powered by battery via buck converter (5 V)                | 5 V logic from Arduino to driver                                                | Also supplies 5 V + GND to motor controller logic and |  |                    |                                                            |                                                                                 |                                               sensors |
| **Raspberry Pi**   | Powered by battery via separate buck converter (5 V 2.5 A) |                                                                                 | Runs human detection and audio interaction via Pi     |  |                    |                                                            |                                                                                 |                          Camera +  USB/3.5 mm speaker |

Functional Highlights

Continuous forward navigation with obstacle detection and automatic turning.
Pause when boxed in; backup + rescan if stuck for extended periods.
Caster-wheel filtering to ignore false echoes.
Raspberry Pi subsystem: detects people → plays greetings through speaker.
Manual control (Tele-op) and Emergency Stop (E-STOP) over serial / ESP connection.

Operating Modes

| Mode       | Description                | Trigger    |
| ---------- | -------------------------- | ---------- |
| **AUTO**   | Full autonomous navigation | Default    |
| **TELEOP** | Manual remote control      | Serial ‘M’ |
| **E-STOP** | Emergency halt (latched)   | Serial ‘E’ |


