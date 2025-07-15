# servo_driver_uROS
Firmware for ESP32 that subscribes to ROS 2 topics and drives bus servos via micro‑ROS.

# Features
* micro‑ROS subscriber on ESP32 to receive servo trajectory commands
* FreeRTOS-based task scheduling
* Controls HiWonder HTD‑45H bus servos over I2C

# Prerequisites
## Hardware
* ESP32 DevKit (e.g. ESP32‑DevKitC)
* HiWonder HTD‑45H bus servo(s) wired to the ESP32 MCU



# Software
* PlatformIO for VS Code or Arduino CLI
* Docker
* micro‑ROS agent image (microros/micro-ros-agent:humble)


# Getting Started
1. Clone & Checkout
```bash
git clone https://github.com/siangting/servo_driver_uROS.git
cd servo_driver_uROS
git checkout uROS
```


2. Flash Firmware to ESP32
    1. Open the project in VS Code (with PlatformIO) or use Arduino CLI.
    2. Select the correct serial port for your ESP32.
    3. Click Upload (or run pio run --target upload).


> If you see
> Could not find 'cmake' executable
> install CMake:
> ```bash!
> sudo apt install cmake
> ```

3. Launch micro‑ROS Agent
In a separate terminal on your host machine:
```bash
docker run -it --rm --net=host --privileged \
  -v /dev:/dev \
  microros/micro-ros-agent:humble \
  serial --dev /dev/ttyACM0 -b 115200 -v6
```
> Tip: Adjust /dev/ttyACM0 to match your ESP32’s device.


4. Publish Servo Commands
Use the companion servo_publisher project (or any ROS 2 publisher) to send trajectories:
```
# In another terminal:
cd path/to/servo_publisher
bash run_docker.sh
```
Ensure both sides share the same ROS 2 domain:

```
export ROS_DOMAIN_ID=0
ros2 topic list
ros2 run servo_publisher servo_trajectory_pub
```

5. Observe Servo Movement
* The micro‑ROS agent terminal will show incoming messages.
* The ESP32 will actuate connected servos according to the received trajectories.
