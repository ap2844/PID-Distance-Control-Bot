# PID Distance Control Bot

Closed-loop mobile robot built using a Raspberry Pi Pico and L298N motor driver.

The bot uses an ultrasonic sensor to maintain a target distance from an object using a PID controller. Motor output is adjusted in real time based on the distance error, allowing the bot to move forward, stop, or reverse as needed.

Wheel encoder feedback is used to balance motion between the left and right motors, helping maintain straight-line movement.

The system also includes braking during direction changes, reverse kick-start handling, and filtering of sensor readings for stable operation.

## Features

- PID-based distance control using ultrasonic sensing
- Encoder-based motion balancing
- PWM motor control using L298N
- Active braking during direction changes
- Reverse kick-start logic
- Filtered sensor readings

## Hardware

- Raspberry Pi Pico
- L298N motor driver
- Ultrasonic sensor
- DC motors with encoders
- Power supply
- Robot chassis

## Structure
