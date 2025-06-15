# P1team
🏎️ Autonomous Racing Car Simulation with ROS 2 & Gazebo
Overview
This project demonstrates a simple autonomous racing system for a small virtual Formula 1-style car. The goal is to simulate self-driving behavior using ROS 2 Humble and the Gazebo simulator, running on a Windows machine via WSL (Windows Subsystem for Linux).

The car navigates by following a sequence of predefined waypoints—without relying on any sensors.

### Features
✅ ROS 2 nodes written in Python

✅ Basic waypoint navigation

✅ Speed adaptation during curves for better stability

✅ Simulation environment built with Gazebo 11

✅ Runs on Windows using WSL + Ubuntu 22.04

### How It Works
The car follows a list of waypoints, adjusting its direction and speed in real time.

At every simulation step, the car calculates its position relative to the next waypoint and modifies its steering angle and velocity accordingly.

Speed is automatically reduced on sharp turns to maintain control and avoid instability.

## Technologies Used
ROS 2 Humble Hawksbill

Gazebo 11

Python 3

Ubuntu 22.04 (via WSL)

Windows 10/11

### Future Improvements
Add LIDAR or camera sensors for perception

Implement PID control for smoother motion

Use path planning algorithms (e.g., A*, RRT) for dynamic navigation

Integrate real-time obstacle avoidance
