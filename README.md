# CS 350 – Smart Thermostat Project
Author: Julliane Pamfilo
Course: CS 350 – Emerging Systems Architectures and Technologies

---

## Project Summary
The smart thermostat project was designed to simulate the functionality of an IoT-based thermostat system using a Raspberry Pi 4B. The goal was to create interface software that controls hardware components such as an LCD, LEDs, and a temperature sensor. The thermostat reads environmental data from the AHT20 sensor through the I2C protocol, displays temperature and setpoint values on a 16x2 LCD, and uses PWM-controlled LEDs to visually represent heating or cooling states. This prototype demonstrates how embedded systems can process, display, and control environmental conditions, serving as a foundation for future Wi-Fi-connected smart devices.

---

## What I Did Well
I did particularly well in designing and implementing the thermostat’s state machine, which controls transitions between OFF, HEAT, and COOL modes. I also integrated the AHT20 sensor and LCD display effectively, ensuring that both readings and user adjustments were handled smoothly. The Python code structure is modular, clearly commented, and matches the rubric’s requirements for readability and maintainability.

---

## Where I Could Improve
If I were to enhance this project further, I would add better exception handling for I2C communication errors and sensor disconnections. I would also integrate a Wi-Fi module and RESTful API calls so the thermostat could send temperature data to a remote server or dashboard, aligning more closely with real IoT applications.

---

## Tools and Resources Added to My Support Network
I used the Raspberry Pi 4B, AHT20 temperature sensor, RPLCD, gpiozero, smbus2, and pyserial libraries. Documentation from Adafruit and RPLCD provided guidance on proper I2C configuration. These tools and resources are now part of my long-term reference library for embedded and IoT projects.

---

## Transferable Skills
Through this project, I strengthened my understanding of hardware-software integration, I2C communication, and multithreaded control using interrupts. I also developed practical experience in using UART for serial communication and applying PWM for LED fading effects. These skills are directly transferable to other embedded, robotics, and IoT system designs.

---

## Maintainability, Readability, and Adaptability
I wrote modular, well-commented code that separates responsibilities—sensor reading, display output, and state logic are isolated in clean classes. The variable names are descriptive, the structure is easy to extend, and the system can support additional sensors or wireless connectivity without major rewrites. The included STATEMACHINE.drawio.pdf clearly documents the system flow and supports future maintainability.

---

## Repository Contents
Smart-Thermostat-Project/
│
├── thermostat_final.py
├── STATEMACHINE.drawio.pdf
└── README.md

---

## Repository Link
https://github.com/JullianePamfilo/Smart-Thermostat-Project

---

## Submission Checklist
- [x] Uploaded final project code and state machine diagram
- [x] Added README reflection and summary
- [x] Added instructor as GitHub collaborator
- [x] Verified repository link before submitting to SNHU
