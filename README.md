# ROS 2 Action Server-Client Example (Jazzy)

[![ROS 2 Jazzy](https://img.shields.io/badge/ROS_2-Jazzy-green.svg)](https://docs.ros.org/en/jazzy/)  
[![License](https://img.shields.io/badge/License-MIT-blue.svg)](https://opensource.org/licenses/MIT)

This package provides a robust ROS 2 action server and client implementation in C++ for ROS 2 Jazzy, demonstrating goal preemption, feedback, and cancellation handling.

## Overview

The `action_server_client` package implements:
- An action server that processes goals with a 5-second duration
- An action client that subscribes to a topic for goal requests
- Priority-based goal preemption and detailed feedback mechanisms

## Features

- **5-Second Goal Execution**: Goals complete in 5 seconds with progress feedback
- **Priority Handling**: Higher-priority goals preempt lower-priority ones
- **Feedback**: Includes status, progress (0-100%), and remaining time
- **Cancellation**: Graceful handling of goal abortions
- **Thread-Safe**: Mutex-protected operations
- **Configurable**: Default priority adjustable via parameters

## Prerequisites

- ROS 2 Jazzy
- C++17 compatible compiler (e.g., g++)
- Colcon build tool
- Git (optional, for version control)

## Installation

1. **Create and initialize a workspace**:
   ```bash
   mkdir -p ~/ros2_jazzy_ws/src
   cd ~/ros2_jazzy_ws
   ```

2. **Clone the repository**:
   ```bash
   cd src
   git clone https://github.com/yourusername/ros2_action_server_client.git
   ```

3. **Build the package**:
   ```bash
   cd ~/ros2_jazzy_ws
   colcon build --packages-select action_server_client
   ```

4. **Source the workspace**:
   ```bash
   source install/setup.bash
   ```

## Running the Example

Test the action server and client with these steps. Open a new terminal for each command and source the workspace first.

### Start the Action Server
```bash
ros2 run action_server_client action_server
```

### Start the Action Client
```bash
ros2 run action_server_client action_client --ros-args -p default_priority:=2
```
- The `--ros-args -p default_priority:=2` sets the default priority for goals.

### Test with Goals
- **Single Goal**:
  ```bash
  ros2 topic pub /goal_topic std_msgs/msg/Int32 "data: 1"
  ```
- **High-Rate Test (2 Hz)**:
  ```bash
  ros2 topic pub -r 2 /goal_topic std_msgs/msg/Int32 "data: 1"
  ```
  - Demonstrates preemption of previous goals when published at a high rate.
- **Priority Test with Incrementing Publisher**:
  ```bash
  ros2 run action_server_client int_publisher
  ```
  - Publishes incrementing numbers to simulate varying goal IDs.

## Package Structure

```
ros2_action_server_client/
├── action/
│   └── Wait.action        # Action definition
├── src/
│   ├── action_server.cpp  # Action server implementation
│   └── action_client.cpp  # Action client implementation
├── package.xml            # Package manifest
├── CMakeLists.txt         # Build configuration
└── README.md              # This file
```
