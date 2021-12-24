# NVILIDAR ROS2 Driver

nvilidar_ros2_driver is a new ros package, which is designed to gradually become the standard driver package for nvilidar devices in the ros2 environment.

## How to install ROS2

[install](https://index.ros.org/doc/ros2/Installation)

## How to Create a ROS2 workspace
[Create a workspace](https://index.ros.org/doc/ros2/Tutorials/Colcon-Tutorial/#create-a-workspace)

## Clone or Download nvilidar_ros2

1. Clone nvilidar_ros2_driver package for github : 

   `git clone https://github.com/nvilidar/nvilidar_ros2.git`

   or 

   `git clone https://gitee.com/nvilidar/nvilidar_ros2.git`

   or get the source in the website: [nvistar](https://www.nvistar.com)

2. copy the file to the "src" dir

3. Build nvilidar_ros2_driver package :

   ```
   cd nvilidar_ros2_ws
   colcon build --symlink-install
   ```
   Note: install colcon [see](https://index.ros.org/doc/ros2/Tutorials/Colcon-Tutorial/#install-colcon)

4. Package environment setup :

   `source ./install/setup.bash`

    Note: Add permanent workspace environment variables.
    It's convenientif the ROS2 environment variables are automatically added to your bash session every time a new shell is launched:
    ```
    $echo "source ~/nvilidar_ros2_ws/install/setup.bash" >> ~/.bashrc
    $source ~/.bashrc
    ```
5. Confirmation
    To confirm that your package path has been set, printenv the `grep -i ROS` variable.
    ```
    $ printenv | grep -i ROS
    ```

6. Create serial port Alias [optional] 
    ```
	$chmod 0777 nvilidar_ros2/startup/*
	$sudo sh nvilidar_ros2/startup/initenv.sh
    ```
   Note: After completing the previous operation, replug the LiDAR again.
	
## Configure LiDAR [paramters](params/nvilidar.yaml)
```
nvilidar_ros2_node:
  ros__parameters:
    serialport_name: "/dev/nvilidar"
    serialport_baud: 921600
    ip_addr: "192.168.1.200"
    lidar_udp_port: 8100
    config_udp_port: 8200
    frame_id: "laser_frame"
    resolution_fixed: true
    auto_reconnect: false
    reversion: false
    inverted: false
    angle_min: -180.0
    angle_max: 180.0
    range_min: 0.0
    range_max: 64.0
    aim_speed: 10.0
    sampling_rate: 10
    sensitive: false
    tailing_level: 6
    single_channel: false
    ignore_array_string: ""
```

## Run nvilidar_ros2

##### Run nvilidar_ros2 using launch file

The command format is : 

 `ros2 launch nvilidar_ros2 [launch file].py`

1. Connect LiDAR uint(s).
   ```
   ros2 launch nvilidar_ros2 nvilidar_launch.py 
   ```
2. RVIZ 
   ```
   ros2 launch nvilidar_ros2 nvilidar_launch_view.py 
   ```
3. echo scan topic
   ```
   ros2 run nvilidar_ros2 nvilidar_ros2_client
   ```
   or
   ```
   ros2 topic echo /scan
   ```







