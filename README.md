# Sample microROS project for PlatformIO

Quick example usage of microROS for PlatformIO using a Raspberry Pi Pico.

Project could be easily adapted for any board listed on [microROS library for 
PlatformIO](https://github.com/micro-ROS/micro_ros_platformio) Github repository.

This firmware does the following:

* Uses mbed threading and blinks a led
* Wait for microROS agent communication
* Start a ROS2 node named `pio_uros_base_node`
* Start a positive and a negative counter and publish info via topics `/test1` and `/test2`
* Listen to topic `/cmd_topic` and updates positive and negative counter with incoming int32 data
* Positive and negative counter are updated on different threads so this emulates a real environment

# Useful commands

Assuming you have ros2 humble (or newer) and microROS agent ready in your environment:

```bash
# start microros agent - fix for your rpico id
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/serial/by-id/$rpicoid baudrate=115200

# list all ros2 topics detected 
ros2 topic list

# listen specific topic data
ros2 topic echo /test1

# publish specific topic data
ros2 topic pub /cmd_topic std_msgs/Int32 "data: 1000"
```

Compile firmware with:
```bash
pio run -e pico
```

Upload firmware with:
```bash
pio run -e pico --target upload
```

