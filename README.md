ESP32-S3 micro-ROS Camera Stream
<div align="center"> <img src="https://img.shields.io/badge/Platform-ESP32--S3-red.svg" alt="Platform"/> <img src="https://img.shields.io/badge/ROS2-Humble-blue.svg" alt="ROS2"/> <img src="https://img.shields.io/badge/Arduino-IDE-cyan.svg" alt="Arduino IDE"/> <img src="https://img.shields.io/badge/License-MIT-green.svg" alt="License"/> </div>
A comprehensive implementation of real-time camera streaming from ESP32-S3 to ROS2 using the micro-ROS framework. This project enables JPEG image transmission from XIAO ESP32S3 Sense camera module to ROS2 ecosystem for visualization in RViz2.

🎥 Demo
Stream live camera feed from ESP32-S3 directly to RViz2 with minimal latency and robust WiFi connection handling.

📋 Table of Contents
Features

Hardware Requirements

Environment Setup

Installation

Configuration

Usage

RViz2 Setup

Troubleshooting

Technical Specifications

Contributing

License

✨ Features
Real-time JPEG streaming from ESP32-S3 camera to ROS2

WiFi-based communication with automatic reconnection

RViz2 visualization support with proper TF integration

Configurable image quality and capture intervals

Memory-optimized implementation for embedded systems

Error handling with LED status indication

Cross-platform compatibility with ROS2 Humble

🔧 Hardware Requirements
Component	Specification
Microcontroller	XIAO ESP32S3 Sense
Camera Module	OV2640/OV3660/OV5640 (included with XIAO ESP32S3 Sense)
RAM	512KB SRAM + 2MB PSRAM
WiFi	802.11 b/g/n (2.4GHz)
Development Board	USB-C connector
Computer	Ubuntu 22.04 with ROS2 Humble
🌐 Environment Setup
Prerequisites
This setup follows the environment configuration from this tutorial.

1. Install ROS2 Humble
bash
# Update system packages
sudo apt update && sudo apt upgrade -y

# Add ROS2 repository
sudo apt install software-properties-common curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo sh -c 'echo "deb http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'

# Install ROS2 Humble Desktop
sudo apt update
sudo apt install ros-humble-desktop
2. Install micro-ROS Dependencies
bash
# Source ROS2 environment
source /opt/ros/humble/setup.bash

# Install micro-ROS agent
sudo apt install ros-humble-micro-ros-agent

# Install image transport plugins (Essential for RViz2 image display)
sudo apt install ros-humble-image-transport
sudo apt install ros-humble-image-transport-plugins
sudo apt install ros-humble-compressed-image-transport
3. Setup Arduino IDE
bash
# Download and install Arduino IDE 2.x
wget -O arduino-ide.zip https://downloads.arduino.cc/arduino-ide/arduino-ide_2.2.1_Linux_64bit.zip
unzip arduino-ide.zip
sudo mv arduino-ide_* /opt/arduino-ide
sudo ln -s /opt/arduino-ide/arduino-ide /usr/local/bin/arduino-ide
4. Configure ESP32 Board Package
Open Arduino IDE

Go to File → Preferences

Add ESP32 board manager URL:

text
https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
Go to Tools → Board → Boards Manager

Search "ESP32" and install "esp32 by Espressif Systems"

5. Install micro-ROS Arduino Library
bash
# Clone micro-ROS Arduino library
cd ~/Arduino/libraries
git clone -b humble https://github.com/micro-ROS/micro_ros_arduino.git
🚀 Installation
1. Clone Repository
bash
git clone https://github.com/yourusername/esp32-microros-camera.git
cd esp32-microros-camera
2. Arduino Code Setup
Open esp32_camera_stream.ino in Arduino IDE

Select board: Tools → Board → ESP32 Arduino → XIAO_ESP32S3

Configure the following settings:

USB CDC On Boot: Enabled

CPU Frequency: 240MHz

Flash Size: 8MB

PSRAM: OPI PSRAM

⚙️ Configuration
Update the network configuration in the Arduino code:

cpp
// Network Configuration
char* ssid = "YOUR_WIFI_SSID";           // Replace with your WiFi SSID
char* password = "YOUR_WIFI_PASSWORD";   // Replace with your WiFi password
char* MICRO_ROS_AGENT_IP = "192.168.1.100";  // Replace with your computer's IP
const int MICRO_ROS_AGENT_PORT = 8888;
Finding Your Computer's IP Address
bash
# Get your IP address
ip addr show | grep "inet " | grep -v 127.0.0.1
🎯 Usage
Step 1: Upload Code to ESP32
Connect XIAO ESP32S3 Sense via USB-C

Select correct port in Arduino IDE

Upload the code

Step 2: Start micro-ROS Agent
bash
# Terminal 1: Start micro-ROS agent
source /opt/ros/humble/setup.bash
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
Step 3: Setup Static Transform (Required)
bash
# Terminal 2: Setup TF frame
source /opt/ros/humble/setup.bash
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map camera_link
Step 4: Verify Camera Stream
bash
# Terminal 3: Check topics
source /opt/ros/humble/setup.bash
ros2 topic list
ros2 topic hz /image/compressed
📺 RViz2 Setup
Launch RViz2
bash
# Terminal 4: Launch RViz2
source /opt/ros/humble/setup.bash
rviz2
Configure Image Display
Global Options:

Set Fixed Frame to map

Add Image Display:

Click Add → By display type → Image

Topic: /image/compressed

Transport Hint: compressed ⚠️ This is crucial!

Reliability Policy: Best Effort (if available)

Alternative: Use rqt_image_view for Testing
bash
# Simple image viewer
ros2 run rqt_image_view rqt_image_view
🔍 Troubleshooting
Common Issues and Solutions
Issue	Solution
"Message Filter dropping message"	Ensure static TF publisher is running:
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map camera_link
No image in RViz2	1. Check Transport Hint is set to compressed
2. Verify image transport plugins are installed
3. Try rqt_image_view for testing
WiFi connection fails	1. Use 2.4GHz network (ESP32 doesn't support 5GHz)
2. Check firewall allows port 8888
3. Verify IP address is correct
Camera init failed	1. Check camera module connections
2. Verify PSRAM is detected
3. Try smaller frame size for testing
Agent connection timeout	1. Check network connectivity
2. Verify micro-ROS agent is running
3. Check IP address and port
Debug Commands
bash
# Check ROS2 ecosystem
ros2 node list                    # List active nodes
ros2 topic list                   # List available topics
ros2 topic hz /image/compressed   # Check message frequency

# Verify image data
ros2 topic echo /image/compressed --field header

# Check TF tree
ros2 run tf2_tools view_frames.py

# Visualize node graph
ros2 run rqt_graph rqt_graph
ESP32 Serial Monitor Debug
Monitor the ESP32 output for debugging:

text
🚀 ESP32-S3 micro-ROS2 Camera Stream Starting
📡 Connecting to WiFi: YOUR_SSID
✅ WiFi connected: 192.168.1.109
🤖 micro-ROS Agent: 192.168.1.100:8888
✅ Camera initialized successfully
🔗 Initializing micro-ROS...
🔗 Node created successfully
✅ System ready!
📷 JPEG format confirmed
📷 Image captured: 3364 bytes
📊 Technical Specifications
Specification	Details
Platform	XIAO ESP32S3 Sense
Framework	micro-ROS Arduino
ROS2 Distribution	Humble
Image Format	JPEG Compressed
Default Resolution	QVGA (320x240)
Max Image Size	15KB buffer
Capture Rate	Configurable (default: 5 seconds)
Network Protocol	UDP over WiFi
Memory Usage	Optimized for PSRAM
🤝 Contributing
Contributions are welcome! Please feel free to submit a Pull Request.

Development Setup
Fork the repository

Create a feature branch

Make your changes

Test thoroughly

Submit a pull request

📄 License
This project is licensed under the MIT License - see the LICENSE file for details.

🙏 Acknowledgments
micro-ROS team for the excellent Arduino integration

Espressif for ESP32-S3 platform

ROS2 community for comprehensive documentation

Tutorial reference: YouTube Setup Guide

<div align="center"> <p>⭐ If this project helped you, please give it a star! ⭐</p> </div>