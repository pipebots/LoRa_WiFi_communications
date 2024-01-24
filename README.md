
This repository contains the following Python scripts for a ROS 2 (Robot Operating System 2) project:

1. `integrated_node.py`: This script defines the `IntegratedNode` class, which is a ROS 2 node responsible for integrating different functionalities. It subscribes to topics for receiving data from other nodes, manages WiFi connection, runs a Flask server for serving web pages, and writes data to a CSV file. It also communicates with a Pycom LoPy4 device over serial.

2. `file_publisher.py`: This script defines the `PublisherNode` class, which is a ROS 2 node responsible for publishing messages to two topics. It publishes the file path to the "folder_path_topic" and a randomly selected location to the "location_topic" at a fixed interval.

3. `diagnostic_subscriber.py`: This script defines the `DiagnosticSubscriber` class, which is a ROS 2 node responsible for subscribing to the "diagnostic_info" topic and receiving diagnostic information. It logs the received information and writes it to a CSV file.

4. `diagnostic_publisher.py`: This script defines the `DiagnosticPublisher` class, which is a ROS 2 node responsible for publishing diagnostic information to the "diagnostic_info" topic. It retrieves system information such as CPU temperature, CPU usage, and RAM usage, and publishes it as a `DiagnosticStatus` message.

## Prerequisites

- ROS 2: Make sure you have ROS 2 installed on your system. Refer to the ROS 2 documentation for installation instructions.

- Python 3: The scripts are written in Python 3. Make sure you have Python 3 installed on your system.

- Dependencies: Install the necessary dependencies by running the following command:

  ```
  pip install rclpy flask gpiozero psutil
  ```

## How to Use

1. Clone the repository to your local machine.

2. Build the ROS 2 workspace:

   ```
   cd <path_to_workspace>
   colcon build
   ```

3. Source the ROS 2 setup file:

   ```
   source <path_to_workspace>/install/setup.bash
   ```

4. Open multiple terminal windows to run different scripts.

5. In one terminal, run the integrated node:

   ```
   ros2 run <package_name> integrated_node.py
   ```

6. In another terminal, run the file publisher node:

   ```
   ros2 run <package_name> file_publisher.py
   ```

7. In another terminal, run the diagnostic subscriber node:

   ```
   ros2 run <package_name> diagnostic_subscriber.py
   ```

8. In another terminal, run the diagnostic publisher node:

   ```
   ros2 run <package_name> diagnostic_publisher.py
   ```

Note: Replace `<package_name>` with the actual name of the ROS 2 package containing the scripts.

## Additional Notes

- Make sure to modify the scripts according to your specific requirements. Update the topics, file paths, WiFi credentials, and other parameters as needed.

- The scripts assume the presence of specific directories and files. Make sure the directories and files exist or modify the paths accordingly.

- Refer to the comments in the scripts for detailed explanations of each component and its functionality.

- For more information on ROS 2 and its concepts, refer to the ROS 2 documentation.

