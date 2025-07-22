# pointcloud_relay
Installing dependencies in Ubuntu if necessary:
```sh
sudo apt install ros-<ros-distro>-pcl-ros ros-<ros-distro>-pcl-conversions
```
Launching node through
```sh
roslaunch pointcloud_relay relay.launch input_topic:=<your_input_topic> output_topic:=<your_output_topic> output_frame:=<your_output_frame>
```
Capture the publishing message by
```sh
rosservice call /pointcloud_relay/capture_pointcloud
```