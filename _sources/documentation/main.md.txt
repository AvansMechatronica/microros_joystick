# Commands

## Under construction

Clone repository
```
cd ~
git clone https://github.com/AvansMechatronica/microros_joystick.git
```



Hardware
USB:

WiFi:
Setup EPS32 lite filesystem see:
[setup filesystem](https://randomnerdtutorials.com/esp32-vs-code-platformio-littlefs/)

Bringup microros:
Wifi
```
ros2 launch p3dx_bringup wifi.launch.py
```
of
```
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888 -v6
```

USB
```
ros2 launch p3dx_bringup usb.launch.py
```
of
```
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -v6
```


Install microros Agent
```
source /opt/ros/$ROS_DISTRO/setup.bash

mkdir uros_ws && cd uros_ws

git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup

rosdep update && rosdep install --from-paths src --ignore-src -y

colcon build

source install/local_setup.bash

ros2 run micro_ros_setup create_agent_ws.sh
ros2 run micro_ros_setup build_agent.sh

echo "source ~/uros_ws/install/local_setup.bash" >> $HOME/.bashrc

source install/local_setup.bash
```