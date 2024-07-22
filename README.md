# Prometheus_SLAM
Prometheus_SLAM is a centralized multi-robot collaborative dense slam framework based on [**Voxblox**](https://github.com/ethz-asl/voxblox) and [**Voxgraph**](https://github.com/ethz-asl/voxgraph). The pose estimator on the agent side is [**VINS-Fusion-GPU**](https://github.com/pjrambo/VINS-Fusion-gpu) and can be easily changed to other state of the art pose estimators.

![gif1](https://github.com/senselabrobo/Prometheus_SLAM/blob/main/docs/Prometheus_SLAM_euroc.gif)
# 1.Prerequisites
* **Ubuntu 20.04**
* **ROS Noetic**
* **CUDA 11.x (do not install version higher than 11)**

# 2.Building Prometheus_SLAM

Init workspace
```
mkdir ~/Prometheus_SLAM_ws 
cd ~/Prometheus_SLAM_ws && mkdir src 
catkin init
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
cd src
git clone https://github.com/senselabrobo/Prometheus_SLAM.git
cd ..
```

## (1)build opencv3_catkin
```
catkin build -j opencv3_catkin
```

## (2)build ceres_catkin
```
catkin build -j ceres_catkin

```
## (3)build rest packages
```
catkin build -j cv_bridge_new prometheus_slam vins_client_server pose_graph_backend image_undistort camera_models vins loop_fusion
```

# 3.Run EuRoC Dataset (2 agents)
1. Download EuRoC dataset rosbag [files](https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets) (eg. MH_01 & MH02).
2. Change the corresponding bag_file path in src/Prometheus_SLAM/prometheus_slam/prometheus_slam/launch/euroc/run_prometheus_slam_euroc.launch, then run:
```
cd ~/Prometheus_SLAM_ws
source devel/setup.bash
roslaunch prometheus_slam run_prometheus_slam_euroc.launch
```

# 4.Run Agents and Server Seperately (EuRoC Dataset)
On server side, press Ctrl+Alt+T to open a new terminal, then run
```
cd ~/Prometheus_SLAM_ws
source devel/setup.bash
roslaunch prometheus_slam run_prometheus_slam_euroc_server.launch
```
On agent0 side, press Ctrl+Alt+T to open a new terminal, then run
```
cd ~/Prometheus_SLAM_ws
source devel/setup.bash
roslaunch prometheus_slam run_prometheus_slam_euroc_client0.launch
```
On agent1 side, press Ctrl+Alt+T to open a new terminal, then run
```
cd ~/Prometheus_SLAM_ws
source devel/setup.bash
roslaunch prometheus_slam run_prometheus_slam_euroc_client1.launch
```

# 5.Run Agents and Server on different PCs
* All PCs need to be in the same network!
* Find the IP address of the PC intented to run the server and agents using ```ifconfig```. Make sure to pick the IP from the wireless interface.
* On each agent PC, add below lines into ~/.bashrc
```
export ROS_MASTER_URI=http://IP_OF_SERVER:11311
export ROS_IP=IP_OF_EACH_AGENT
```
* On Server PC, add below lines into ~/.bashrc
```
export ROS_MASTER_URI=http://IP_OF_SERVER:11311
export ROS_IP=IP_OF_Server
```
* Start a ```roscore``` on the Server PC.
* Then start server and agents as in section 4.

# 6.Using your own Camera&IMU
![gif2](https://github.com/senselabrobo/Prometheus_SLAM/blob/main/docs/Prometheus_SLAM_senselab.gif)
For using your own camera&imu, you need to change corresponding calibration files and corresponding sensor ros topics:
* For each agent x, change calibration file and camera&imu ros topic in src/Prometheus_SLAM/VINS-Fusion-gpu/config/euroc/euroc_stereo_imu_config_x.yaml, and change the pointcloud ros topic in src/Prometheus_SLAM/prometheus_slam/prometheus_slam/launch/firefly/prometheus_slam_client.launch
* For Server, change calibration file cam_config in src/Prometheus_SLAM/prometheus_slam/prometheus_slam/launch/utils/pose_graph_backend_gpu.launch
