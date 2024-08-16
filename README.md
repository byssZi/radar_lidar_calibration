# radar_lidar_static_calibration

一个手动-自动相结合的用于静态标定radar与lidar的工程，项目来源参考了 https://github.com/gloryhry/radar_lidar_static_calibration 与 https://github.com/PJLab-ADG/SensorsCalibration/tree/master/radar2lidar/manual_calib 工程
静态标定通过手动调整x,y,z,roll,pitch,yaw转动radar点云靠近lidar点云使其尽量重合后，通过按下自动标定按钮确定最终的变换矩阵

# Step1
```bash
catkin_make

source devel/setup.bash
```
# Step2
修改launch文件下radar与lidar的话题, radar话题为visualization_msgs::Markerarray格式，lidar话题为sensor_msgs::Pointcloud2格式
```bash
roslaunch radar_lidar_static_calibration calibration.launch
```
# Step3
播放用于标定的rosbag包

# Step4
选取合适的一帧，按空格键进入标定环节

