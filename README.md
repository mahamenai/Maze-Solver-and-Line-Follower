2-in-1 Robot: Line Follower & Maze Solver
A dual-mode autonomous robot built around an ESP32 microcontroller. Same chassis, two completely different behaviors.

Hardware

Brain: ESP32 microcontroller (on a shield)
Motors: 2× N20 gear motors + H-bridge motor driver
Maze sensors: 3× HC-SR04 ultrasonic sensors (left, front, right)
Motion: MPU6050 gyroscope (for precise turns)
Line sensors: 6× IR reflectance sensors
Power: 4× Li-Ion batteries + buck converter (stepped down to 5V)
Interface: Push button (start/stop) + status LED


Mode 1 — Line Follower
The robot uses its IR sensor array to detect and follow a black line on a white surface (or vice versa).
A PID algorithm continuously reads the line's position under the robot and adjusts left/right motor speeds to correct the error — keeping turns smooth and speed high on straights.

Mode 2 — Maze Solver
The robot navigates an unknown maze fully autonomously using the right-hand rule.
Wall following (PID): With two walls, it centers itself by equalizing left and right sensor readings. With one wall, a single-wall PID keeps it at a fixed 10 cm offset from that wall.
Gyro-assisted turns: At intersections, it integrates angular velocity from the gyroscope to execute precise 90° and 180° rotations.
Self-recovery: If any sensor reads under 2 cm, it reverses, identifies which side it clipped, and steers away automatically.

A note on tuning
The maze solver works, but it's not plug-and-play. Physical variables affect performance and will likely need adjustment for your specific build:

PID values (P, D): Tune based on your robot's weight and wheel grip to reduce wobble.
Intersection delays: Adjust the pre/post-turn delays to match your corridor width. If it clips the inner wall on turns, increase the delay.
Base speed: Battery level affects motor speed. Expect a few test runs before things feel dialed in.


Getting started

Clone this repo
Open the project in Arduino IDE or PlatformIO
Install the required libraries:

MPU6050_light
NewPing


Select your desired mode, compile, and upload to the ESP32
Place the robot, press start, step back, and let it go

Contributions and tuning suggestions are welcome!
