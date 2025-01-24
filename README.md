# ``mult_kinect_ros2``

![ROS2](https://img.shields.io/badge/ROS2-Humble-blue)  ![OpenCV](https://img.shields.io/badge/OpenCV-4.5.4-orange)

## 🚧 Project Status

This project is currently **under development**. Some features may not be fully implemented or may change in future versions.

## 📖 Overview

`mult_kinect_ros2` is a **ROS2 package** designed for the **publication and visualization of images from multiple Kinect sensors**.  Built on the **`libfreenect` library**, this node enables each Kinect **v1 (for the Xbox 360)** to publish its **RGB and depth images separately** in specific ROS2 topics.

## 🔧 Installation

### **1️⃣ Install [libfreenect](https://github.com/OpenKinect/libfreenect)**
This package relies on **libfreenect**, an open-source driver for Kinect v1.  A manual build from the **official GitHub repository** was used for testing, as certain firmware versions require specific build configurations for compatibility.

### **2️⃣ Clone the repository**
Navigate to your **ROS2 workspace** and clone this repository into the `src` directory:
```bash
cd ~/ros2_ws/src
git clone https://github.com/maarinaabatista/mult_kinect_ros2.git
```
### **3️⃣ Install ROS2 dependencies**
Run ``rosdep`` to ensure all necessary ROS2 dependencies are installed:
```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```
### **4️⃣ Build the Package**
```bash
colcon build --symlink-install --packages-select mult_kinect_ros2
```
After building, source the workspace:
```bash
source install/setup.bash
```
