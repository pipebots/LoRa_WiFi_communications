# Overview

This repository contains a ROS2 nodes designed to facilitate communication over LoRa 868 MHz and WiFi. The node subscribes to various pieces of information, including sensor data, and diagnostic information. This information is then transmitted to a ground station. The LoRa communication is established using a Pycom LoPy4 device connected to a Raspberry Pi 4 via a USB port.
This repository contains the following Python scripts for a ROS 2 (Robot Operating System 2) project:

1. `integrated_node.py`: This script defines the `IntegratedNode` class, which is a ROS 2 node responsible for integrating different functionalities. It subscribes to topics for receiving data from other nodes, manages WiFi connection, runs a Flask server for serving web pages, and writes data to a CSV file. It also communicates with a Pycom LoPy4 device over serial. This code integrates ROS 2 with a Flask web server to create a simple web interface for interacting with data received from a ROS topic. The script creates a ROS 2 node that subscribes to a specific ROS topic (folder_path_topic). When a message is received, it logs the folder path, lists the files in that folder, and starts a Flask web server. The web server provides a simple interface to view the list of files and download them.

2. `file_publisher.py`: This script defines the `PublisherNode` class, which is a ROS 2 node responsible for publishing messages to two topics. It publishes the file path to the "folder_path_topic" and a randomly selected location to the "location_topic" at a fixed interval.

3. `diagnostic_subscriber.py`: This script defines the `DiagnosticSubscriber` class, which is a ROS 2 node responsible for subscribing to the "diagnostic_info" topic and receiving diagnostic information. It logs the received information and writes it to a CSV file.

4. `diagnostic_publisher.py`: This script defines the `DiagnosticPublisher` class, which is a ROS 2 node responsible for publishing diagnostic information to the "diagnostic_info" topic. It retrieves system information such as CPU temperature, CPU usage, and RAM usage, and publishes it as a `DiagnosticStatus` message.

5. index.html in templates folder
The HTML code represents a template for rendering a web page that displays a list of files. The template is intended to be used with a Flask web server, and it dynamically generates HTML content based on the files received from the ROS topic. This page allows to view, download and delete the data save at the following location of Rasberry pi. 'home/pi1/Documents/wifi_data'

## Requirements

To set up the communication system, you will need the following hardware components:

* 2 Pycom LoPy4 or FiPy devices
* 2 pycom Expansion Boards 3.0
* Raspberry Pi 4 with ROS2 installed
* 1 PC for base station monitoring

## Prerequisites

- ROS 2: Make sure you have ROS 2 installed on your system. Refer to the ROS 2 documentation for installation instructions.

- Python 3: The scripts are written in Python 3. Make sure you have Python 3 installed on your system.

- Dependencies: Install the necessary dependencies

## Setup Instructions

Connect the Raspberry Pi 4 to the LoPy4 device using a USB cable. Ensure that the correct port (in this code, it is assumed to be ttyACM0) is used.

Connect the LoPy4 device to the PC or laptop located at the ground station.

Save the code in the file mainSender.py to the main.py file of the LoPy4 connected to the Raspberry Pi 4.

Save the code in the file mainBS.py to the main.py file of the LoPy4 connected to the laptop or PC, which functions as the base station.

## How to Use

1. Clone the repository to your local machine.

2. Build the ROS 2 workspace:

   ```bash
   cd <path_to_workspace>
   colcon build
   ```

3. Source the ROS 2 setup file:

   ```bash
   source <path_to_workspace>/install/setup.bash
   ```

4. Open multiple terminal windows to run different scripts.

5. In one terminal, run the integrated node:

   ```bash
   ros2 run <package_name> integrated_node.py
   ```

6. In another terminal, run the file publisher node:

   ```bash
   ros2 run <package_name> file_publisher.py
   ```

7. In another terminal, run the lora node:

   ```bash
   ros2 run <package_name> lora_node.py
   ```

8. In another terminal, run the diagnostic subscriber node:

   ```bash
   ros2 run <package_name> diagnostic_subscriber.py
   ```

9. In another terminal, run the diagnostic publisher node:

   ```bash
   ros2 run <package_name> diagnostic_publisher.py
   ```

Note: Replace `<package_name>` with the actual name of the ROS 2 package containing the scripts.

## Accessing the File List Web Page on Another Device

To access the `index.html` page on another device which is a base station PC/ laptop for monitoring, make sure both Rasberry pi and basetation are on the same local wifi network.

This webpage will diaplay all the files which are stored at the following location of Rasberry pi '/home/pi/Documents/wifi_data'.

Follow these steps:

1. Find the IP address of your Raspberry Pi:
    - You can usually find this information by running `ifconfig` on your Raspberry Pi and looking for the IP address associated with the network interface (e.g., wlan0).
2. Open a web browser on the other device and enter the following URL:

    ```text
    http://ipaddressofpi:8000
    ```

    Replace "ipaddressofpi" with the actual IP address of your Raspberry Pi.

3. Press Enter, and the web page will display the contents of the shared folder.

Example: `http://192.168.233.24:8000`

This information will help users access the web page served by your Flask application running on the Raspberry Pi from another device on the same local network.

## Additional Notes

* Make sure to modify the scripts according to your specific requirements. Update the topics, file paths, WiFi credentials, and other parameters as needed.
* The scripts assume the presence of specific directories and files. Make sure the directories and files exist or modify the paths accordingly.
* For more information on ROS 2 and its concepts, refer to the ROS 2 documentation.

## Acknowledgments

This work is supported by the UK's Engineering and Physical Sciences Research Council (EPSRC) Programme Grant EP/S016813/1

© 2024 University of Leeds

The author, H. Nasir, has asserted their moral rights.
