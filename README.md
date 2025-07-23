# pointcloud_relay
Installing dependencies in Ubuntu if necessary:
```sh
sudo apt install ros-${ROS_DISTRO}-tf2-sensor-msgs ros-${ROS_DISTRO}-pcl-ros ros-${ROS_DISTRO}-pcl-conversions
```
Launching node through
```sh
roslaunch pointcloud_relay relay.launch input_topic:=<your_input_topic> output_topic:=<your_output_topic> input_frame:=<camera_frame> output_frame:=<world_frame>
```
Capture the publishing message by
```sh
rosservice call /pointcloud_relay/capture_pointcloud
```