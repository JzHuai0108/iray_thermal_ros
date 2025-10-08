
SUBSYSTEM=="usb", ATTR{idVendor}=="3474", ATTR{idProduct}=="750e", MODE="0666", GROUP="plugdev"

SUBSYSTEM=="usb", ATTR{idVendor}=="0x0424", ATTR{idProduct}=="0x6000", MODE="0666"
KERNEL="hidraw", ATTR{idVendor}=="0x0424", ATTR{idProduct}=="0x6000", MODE="0666"



echo pi | sudo -S env LD_LIBRARY_PATH=/opt/ros/noetic/lib:/opt/ros/noetic/lib/aarch64-linux-gnu /home/pc/data_record_ws/thermal_ros_ws/devel/lib/thermal_ros/thermal_ros_node

echo pi | sudo -S env LD_LIBRARY_PATH=/opt/ros/noetic/lib /home/pc/data_record_ws/thermal_ros_ws/devel/lib/thermal_ros/thermal_ros_node 1 0

sudo usermod -aG plugdev wx
sudo udevadm control --reload-rules
sudo udevadm trigger


roscore

source /home/wx/0project/thermal_ros_ws/devel/setup.bash
rosrun thermal_ros thermal_ros_node 1 0


xuan wang, 2024.06.13


## Find the cameras

For instance, with two thermal cameras plugged in the computer, in a terminal, run
```
ls /dev/video*
```

YOu should see something like
```
/dev/video0  /dev/video1  /dev/video2  /dev/video3 /dev/video4  /dev/video5
```

## Build with ROS1
```
catkin build
```

## Access each camera with SDK

```
sudo env LD_LIBRARY_PATH=/opt/ros/noetic/lib /media/jhuai/docker/vision/thermal_ros_ws/devel/lib/thermal_ros/

thermal_ros_node 2

sudo env LD_LIBRARY_PATH=/opt/ros/noetic/lib /media/jhuai/docker/vision/thermal_ros_ws/devel/lib/thermal_ros/

thermal_ros_node 4
```

## Related code

[1] [The C code for thermal cameras with x86_64 SDK libs](https://github.com/JzHuai0108/iray_c_demo)

[2] [The C code for thermal cameras with arm64 SDK libs](https://github.com/LiuYuha0/thermal_SDK_arm64)

