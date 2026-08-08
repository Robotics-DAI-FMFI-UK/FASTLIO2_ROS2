# FASTLIO2 ROS2
## 主要工作
1. 重构[FASTLIO2](https://github.com/hku-mars/FAST_LIO) 适配ROS2
2. 添加回环节点，基于位置先验+ICP进行回环检测，基于GTSAM进行位姿图优化
3. 添加重定位节点，基于由粗到细两阶段ICP进行重定位
4. 增加一致性地图优化，基于[BLAM](https://github.com/hku-mars/BALM) (小场景地图) 和[HBA](https://github.com/hku-mars/HBA) (大场景地图)

## 环境依赖
1. Ubuntu 22.04
2. ROS2 Humble

## 编译依赖
```text
pcl
Eigen
sophus
gtsam
livox_ros_driver2
```

## 详细说明
### 1.编译 LIVOX-SDK2
```shell
git clone https://github.com/Livox-SDK/Livox-SDK2.git
cd ./Livox-SDK2/
mkdir build
cd build
cmake .. && make -j
sudo make install
```

### 2.编译 livox_ros_driver2
```shell
mkdir -r ws_livox/src
git clone https://github.com/Livox-SDK/livox_ros_driver2.git ws_livox/src/livox_ros_driver2
cd ws_livox/src/livox_ros_driver2
source /opt/ros/humble/setup.sh
./build.sh humble
```

### 3.编译 Sophus
```shell
git clone https://github.com/strasdat/Sophus.git
cd Sophus
git checkout 1.22.10
mkdir build && cd build
cmake .. -DSOPHUS_USE_BASIC_LOGGING=ON
make
sudo make install
```

**新的Sophus依赖fmt，可以在CMakeLists.txt中添加add_compile_definitions(SOPHUS_USE_BASIC_LOGGING)去除，否则会报错**


## 实例数据集
```text
链接: https://pan.baidu.com/s/1rTTUlVwxi1ZNo7ZmcpEZ7A?pwd=t6yb 提取码: t6yb 
--来自百度网盘超级会员v7的分享
```

## 部分脚本

### 1.激光惯性里程计 
```shell
ros2 launch fastlio2 lio_launch.py
ros2 bag play your_bag_file
```

### 2.里程计加回环
#### 启动回环节点
```shell
ros2 launch pgo pgo_launch.py
ros2 bag play your_bag_file
```
#### 保存地图
```shell
ros2 service call /pgo/save_maps interface/srv/SaveMaps "{file_path: 'your_save_dir', save_patches: true}"
```

### 3.里程计加重定位
#### 启动重定位节点
```shell
ros2 launch localizer localizer_launch.py
ros2 bag play your_bag_file // 可选
```
#### 设置重定位初始值
```shell
ros2 service call /localizer/relocalize interface/srv/Relocalize "{"pcd_path": "your_map.pcd", "x": 0.0, "y": 0.0, "z": 0.0, "yaw": 0.0, "pitch": 0.0, "roll": 0.0}"
```
#### 检查重定位结果
```shell
ros2 service call /localizer/relocalize_check interface/srv/IsValid "{"code": 0}"
```

### 4.一致性地图优化
#### 启动一致性地图优化节点
```shell
ros2 launch hba hba_launch.py
```
#### 调用优化服务
```shell
ros2 service call /hba/refine_map interface/srv/RefineMap "{"maps_path": "your maps directory"}"
```
**如果需要调用优化服务，保存地图时需要设置save_patches为true**

## 特别感谢
1. [FASTLIO2](https://github.com/hku-mars/FAST_LIO)
2. [BLAM](https://github.com/hku-mars/BALM)
3. [HBA](https://github.com/hku-mars/HBA)
## 性能相关的问题
该代码主要使用timerCB作为频率触发主函数，由于ROS2中的timer、subscriber以及service的回调实际上运行在同一个线程上，在电脑性能不是好的时候，会出现调用阻塞的情况，建议使用线程并发的方式将耗时的回调独立出来(如timerCB)来提升性能

## FAST-LIO registration diagnostics

The G1 configuration enables passive scan-to-map diagnostics. The diagnostics do not change filtering, correspondence selection, the IESKF update, or map insertion.

Published topics:

- `/fastlio2/scan_diagnostics` (`std_msgs/msg/Float64MultiArray`): one estimator summary per LiDAR scan, including the IMU-predicted and corrected state, covariance, map size, map movement, and insertion counts.
- `/fastlio2/iteration_diagnostics` (`std_msgs/msg/Float64MultiArray`): one summary for every IESKF correspondence iteration, including each rejection stage, neighbor distance, residual distribution, plane-normal distribution, range/height bins, and information-matrix eigenvalues.
- `/fastlio2/diagnostic_correspondences` (`sensor_msgs/msg/PointCloud2`): all downsampled world-frame correspondence candidates from the final iteration, throttled by `diagnostic_cloud_hz`. `curvature` contains the rejection stage (`0` accepted, `1` missing neighbors, `2` neighbors too far, `3` plane fit failed, `4` residual score rejected). `intensity` is the signed point-to-plane residual when available and `normal_x/y/z` is the fitted world-frame plane normal.
- `/fastlio2/scan_diagnostics_schema` and `/fastlio2/iteration_diagnostics_schema` (`std_msgs/msg/String`): comma-separated field names in array order, published with transient-local durability.

For a focused failure run, record only the compact diagnostic output plus odometry and IMU:

```shell
ros2 bag record -o ~/bags/fastlio_corner_diagnostics \
  /fastlio2/scan_diagnostics \
  /fastlio2/iteration_diagnostics \
  /fastlio2/diagnostic_correspondences \
  /fastlio2/scan_diagnostics_schema \
  /fastlio2/iteration_diagnostics_schema \
  /fastlio2/lio_odom \
  /dog_odom \
  /utlidar/imu_livox_mid360
```

At 2 Hz the correspondence cloud is much smaller than recording every raw Mid-360 scan. Set `diagnostics_enabled: false` after the experiment to remove the diagnostic computation, or set `diagnostic_cloud_hz: 0.0` to disable only the cloud.

`diagnostics_console: true` also prints one compact `FASTLIO_DIAG` line per scan. The bag topics remain the authoritative machine-readable record.
