# ABRA IDA 2024
 This project implements a ship control system using MAVLink and Pymavlink. It leverages computer vision to detect buoys and control the ship's movements based on the detected objects. The main components of the project include a detector for identifying colored buoys and a control library for managing the ship's operations.

## File Descriptions

- **`main.py`**: This is the main entry point of the application. It initializes the system, starts the detection process, and manages the ship's control commands based on the detections.

- **`detector.py`**: This file contains the logic for detecting buoys of different colors (red, yellow, green) using OpenCV. It defines a class named `Buoy` that stores the coordinates and ID of each detected buoy.

- **`abra_ida.py`**: This library is responsible for interfacing with the MAVLink protocol. It handles reading and writing data from JSON files to keep track of the ship's current mode and status, and provides functions for controlling the ship based on detected objects.

## Dependencies

- OpenCV
- Pymavlink
- NumPy
- JSON (for data management)

