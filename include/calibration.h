#pragma once
#include <iostream>
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <string>


#include <opencv2/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "radar_lidar_calibration/ObjectList.h"
#include "radar_lidar_calibration/ClusterList.h"

class RadarCalibration
{
public:
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud{new pcl::PointCloud<pcl::PointXYZ>()};

    int height; // point.y
    int width;  // point.x
    double resolution = 0.05;
    
    cv::Mat point_mat;
    std::string point_name;

public:
    RadarCalibration(pcl::PointCloud<pcl::PointXYZ>::Ptr &point_cloud, std::string name);
    ~RadarCalibration(){};
    void showPoint();
};