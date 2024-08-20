#include <calibration.h>

RadarCalibration::RadarCalibration(pcl::PointCloud<pcl::PointXYZ>::Ptr &point_msg, std::string name)
{
    point_name = name;
    *point_cloud = *point_msg;

    resolution = 0.05;
    height = 60 / resolution;
    width = 60 / resolution;

    point_mat = cv::Mat::zeros(height, width, CV_8UC1);

    for (auto pt : point_cloud->points)
    {
        int hei, wid;
        wid = int(pt.x / resolution);
        hei = int(-pt.y / resolution) + height / 2;
        if (wid >= 0 && wid < width && hei >= 0 && hei < height)
        {
            point_mat.at<uchar>(hei, wid) = 255;
        }
    }

}

void RadarCalibration::showPoint()
{
    cv::imshow(point_name, point_mat);
    cv::waitKey(1);
}