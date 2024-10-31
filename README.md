# radar_lidar_calibration

一个手动-自动相结合的用于动态标定radar与lidar的工程，项目来源参考了https://github.com/PJLab-ADG/SensorsCalibration/tree/master/radar2lidar/manual_calib 工程</br>
动态标定通过手动调整x,y,z,roll,pitch,yaw或自动标定按钮转动radar点云靠近lidar点云使其尽量重合

# Step1
```bash
catkin_make

source devel/setup.bash
```
# Step2
修改launch文件下radar与lidar的话题, radar话题为visualization_msgs::MarkerArray格式，lidar话题为sensor_msgs::PointCloud2格式
```bash
roslaunch radar_lidar_calibration calibration.launch
```
# Step3
播放用于标定的rosbag包,进入标定界面


