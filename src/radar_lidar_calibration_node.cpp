#include <ros/package.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <cmath>
#include <fstream>
#include <iostream>

#include <pcl/common/transforms.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/conversions.h>
#include <pcl/registration/icp.h>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <termio.h>
#include <stdio.h>
#include <thread>

#include <boost/filesystem.hpp>
#include <opencv2/opencv.hpp>
#include <pangolin/pangolin.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

#include <Eigen/Core>
#include <string>

#include "extrinsic_param.hpp"
#include "calibration.h"
#include <visualization_msgs/MarkerArray.h>

#define THR_FAP 0.2 // thresthold of false alarm probability
#define GL_GPU_MEM_INFO_CURRENT_AVAILABLE_MEM_NVX 0x9049
#define MAX_RADAR_TIME_GAP 15 * 1e6
using namespace message_filters;
using namespace std;

string pkg_loc;
bool sys_pause = false;
bool start_lidar_continue = false;
bool start_radar_continue = false;

ros::Subscriber lidar_sub;
ros::Subscriber radar_sub;

// 获取键盘输入
pangolin::GlBuffer *vertexBuffer_;
pangolin::GlBuffer *colorBuffer_;

double cali_scale_degree_ = 0.3;
double cali_scale_trans_ = 0.06;
static Eigen::Matrix4d calibration_matrix_ = Eigen::Matrix4d::Identity();
static Eigen::Matrix4d orign_calibration_matrix_ = Eigen::Matrix4d::Identity();
std::vector<Eigen::Matrix4d> modification_list_;
bool display_mode_ = false;
int radar_line_size_ = 4;

struct RGB {
  unsigned char r;
  unsigned char g;
  unsigned char b;
};

bool kbhit() {
  termios term;
  tcgetattr(0, &term);
  termios term2 = term;
  term2.c_lflag &= ~ICANON;
  tcsetattr(0, TCSANOW, &term2);
  int byteswaiting;
  ioctl(0, FIONREAD, &byteswaiting);
  tcsetattr(0, TCSANOW, &term);
  return byteswaiting > 0;
}

void CalibrationInit(Eigen::Matrix4d json_param) {
  Eigen::Matrix4d init_cali;
  init_cali << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;
  calibration_matrix_ = json_param;
  orign_calibration_matrix_ = json_param;
  modification_list_.reserve(12);
  for (int32_t i = 0; i < 12; i++) {
    std::vector<int> transform_flag(6, 0);
    transform_flag[i / 2] = (i % 2) ? (-1) : 1;
    Eigen::Matrix4d tmp = Eigen::Matrix4d::Identity();
    Eigen::Matrix3d rot_tmp;
    rot_tmp =
        Eigen::AngleAxisd(transform_flag[0] * cali_scale_degree_ / 180.0 * M_PI,
                          Eigen::Vector3d::UnitX()) *
        Eigen::AngleAxisd(transform_flag[1] * cali_scale_degree_ / 180.0 * M_PI,
                          Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(transform_flag[2] * cali_scale_degree_ / 180.0 * M_PI,
                          Eigen::Vector3d::UnitZ());
    tmp.block(0, 0, 3, 3) = rot_tmp;
    tmp(0, 3) = transform_flag[3] * cali_scale_trans_;
    tmp(1, 3) = transform_flag[4] * cali_scale_trans_;
    tmp(2, 3) = transform_flag[5] * cali_scale_trans_;
    modification_list_[i] = tmp;
  }
  std::cout << "=>Calibration scale Init!\n";
}

void CalibrationScaleChange() {
  Eigen::Matrix4d init_cali;
  init_cali << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;
  modification_list_.reserve(12);
  for (int32_t i = 0; i < 12; i++) {
    std::vector<int> transform_flag(6, 0);
    transform_flag[i / 2] = (i % 2) ? (-1) : 1;
    Eigen::Matrix4d tmp = Eigen::Matrix4d::Identity();
    Eigen::Matrix3d rot_tmp;
    rot_tmp =
        Eigen::AngleAxisd(transform_flag[0] * cali_scale_degree_ / 180.0 * M_PI,
                          Eigen::Vector3d::UnitX()) *
        Eigen::AngleAxisd(transform_flag[1] * cali_scale_degree_ / 180.0 * M_PI,
                          Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(transform_flag[2] * cali_scale_degree_ / 180.0 * M_PI,
                          Eigen::Vector3d::UnitZ());
    tmp.block(0, 0, 3, 3) = rot_tmp;
    tmp(0, 3) = transform_flag[3] * cali_scale_trans_;
    tmp(1, 3) = transform_flag[4] * cali_scale_trans_;
    tmp(2, 3) = transform_flag[5] * cali_scale_trans_;
    modification_list_[i] = tmp;
  }
  std::cout << "=>Calibration scale update done!\n";
}

void saveResult(const int &frame_id) {
  std::string file_name = pkg_loc + "/data/radar2lidar_extrinsic_" + std::to_string(frame_id) + ".txt";
  std::ofstream fCalib(file_name);
  if (!fCalib.is_open()) {
    std::cerr << "open file " << file_name << " failed." << std::endl;
    return;
  }
  fCalib << "Extrinsic:" << std::endl;
  fCalib << "R:\n"
         << calibration_matrix_(0, 0) << " " << calibration_matrix_(0, 1) << " "
         << calibration_matrix_(0, 2) << "\n"
         << calibration_matrix_(1, 0) << " " << calibration_matrix_(1, 1) << " "
         << calibration_matrix_(1, 2) << "\n"
         << calibration_matrix_(2, 0) << " " << calibration_matrix_(2, 1) << " "
         << calibration_matrix_(2, 2) << std::endl;
  fCalib << "t: " << calibration_matrix_(0, 3) << " "
         << calibration_matrix_(1, 3) << " " << calibration_matrix_(2, 3)
         << std::endl;

  fCalib << "************* json format *************" << std::endl;
  fCalib << "Extrinsic:" << std::endl;
  fCalib << "[" << calibration_matrix_(0, 0) << "," << calibration_matrix_(0, 1)
         << "," << calibration_matrix_(0, 2) << "," << calibration_matrix_(0, 3)
         << "],"
         << "[" << calibration_matrix_(1, 0) << "," << calibration_matrix_(1, 1)
         << "," << calibration_matrix_(1, 2) << "," << calibration_matrix_(1, 3)
         << "],"
         << "[" << calibration_matrix_(2, 0) << "," << calibration_matrix_(2, 1)
         << "," << calibration_matrix_(2, 2) << "," << calibration_matrix_(2, 3)
         << "],"
         << "[" << calibration_matrix_(3, 0) << "," << calibration_matrix_(3, 1)
         << "," << calibration_matrix_(3, 2) << "," << calibration_matrix_(3, 3)
         << "]" << std::endl;
  fCalib.close();
}

bool ManualCalibration(int key_input) {
  char table[] = {'q', 'a', 'w', 's', 'e', 'd', 'r', 'f', 't', 'g', 'y', 'h'};
  bool real_hit = false;
  for (int32_t i = 0; i < 12; i++) {
    if (key_input == table[i]) {
      calibration_matrix_ = calibration_matrix_ * modification_list_[i];
      real_hit = true;
    }
  }
  return real_hit;
}

Eigen::Matrix4d AutoCalibration(const pcl::PointCloud<pcl::PointXYZI>::Ptr cloudLidar, const pcl::PointCloud<pcl::PointXYZ>::Ptr cloudRadar) {
  // 创建一个新的点云对象，类型为 PointXYZ
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZ>);
    
  // 调整输出点云的大小以匹配输入点云的大小
  cloud_out->width = cloudLidar->width;
  cloud_out->height = cloudLidar->height;
  cloud_out->is_dense = cloudLidar->is_dense;
  cloud_out->points.resize(cloudLidar->width * cloud_out->height);
    
  // 遍历输入点云，复制坐标到输出点云
  for (size_t i = 0; i < cloudLidar->points.size(); ++i) {
    cloud_out->points[i].x = cloudLidar->points[i].x;
    cloud_out->points[i].y = cloudLidar->points[i].y;
    cloud_out->points[i].z = cloudLidar->points[i].z;
  }

  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  icp.setInputSource(cloudRadar);
  icp.setInputTarget(cloud_out);
  // Set the max correspondence distance to 5cm (e.g., correspondences with higher distances will be ignored)
  icp.setMaxCorrespondenceDistance(20);
  // Set the maximum number of iterations (criterion 1)
  icp.setMaximumIterations(100);
  // Set the transformation epsilon (criterion 2)
  icp.setTransformationEpsilon(0.001);
  // Set the euclidean distance difference epsilon (criterion 3)
  icp.setEuclideanFitnessEpsilon(0.01);
  pcl::PointCloud<pcl::PointXYZ> Final;
  icp.align(Final);
  if (icp.hasConverged()) {
    std::cout << "ICP has converged, score is " << icp.getFitnessScore() << std::endl;
  } else {
    std::cout << "ICP did not converge." << std::endl;
  }
  return icp.getFinalTransformation().cast<double>();
}

RGB GreyToColorMix(int val) {
  int r, g, b;
  if (val < 128) {
    r = 0;
  } else if (val < 192) {
    r = 255 / 64 * (val - 128);
  } else {
    r = 255;
  }
  if (val < 64) {
    g = 255 / 64 * val;
  } else if (val < 192) {
    g = 255;
  } else {
    g = -255 / 63 * (val - 192) + 255;
  }
  if (val < 64) {
    b = 255;
  } else if (val < 128) {
    b = -255 / 63 * (val - 192) + 255;
  } else {
    b = 0;
  }
  RGB rgb;
  rgb.b = b;
  rgb.g = g;
  rgb.r = r;
  return rgb;
}

void ProcessSingleFrame(const pcl::PointCloud<pcl::PointXYZI>::Ptr cloudLidar,
                        const bool &diaplay_mode) {
  if (vertexBuffer_ != nullptr)
    delete (vertexBuffer_);
  if (colorBuffer_ != nullptr)
    delete (colorBuffer_);
  int pointsNum = cloudLidar->points.size();
  std::cout << "lidar point size: " << pointsNum << std::endl;
  pangolin::GlBuffer *vertexbuffer = new pangolin::GlBuffer(
      pangolin::GlArrayBuffer, pointsNum, GL_FLOAT, 3, GL_DYNAMIC_DRAW);
  pangolin::GlBuffer *colorbuffer = new pangolin::GlBuffer(
      pangolin::GlArrayBuffer, pointsNum, GL_UNSIGNED_BYTE, 3, GL_DYNAMIC_DRAW);

  float *dataUpdate = new float[pointsNum * 3];
  unsigned char *colorUpdate = new unsigned char[pointsNum * 3];
  for (int ipt = 0; ipt < pointsNum; ipt++) {
    Eigen::Vector4d pointPos(cloudLidar->points[ipt].x,
                             cloudLidar->points[ipt].y,
                             cloudLidar->points[ipt].z, 1.0);
    dataUpdate[ipt * 3 + 0] = pointPos.x();
    dataUpdate[ipt * 3 + 1] = pointPos.y();
    dataUpdate[ipt * 3 + 2] = pointPos.z();

    if (diaplay_mode) {
      RGB colorFake = GreyToColorMix(cloudLidar->points[ipt].intensity);
      colorUpdate[ipt * 3 + 0] = static_cast<unsigned char>(colorFake.r);
      colorUpdate[ipt * 3 + 1] = static_cast<unsigned char>(colorFake.g);
      colorUpdate[ipt * 3 + 2] = static_cast<unsigned char>(colorFake.b);
    } else {
      for (int k = 0; k < 3; k++) {
        colorUpdate[ipt * 3 + k] =
            static_cast<unsigned char>(cloudLidar->points[ipt].intensity);
      }
    }
  }

  (vertexbuffer)->Upload(dataUpdate, sizeof(float) * 3 * pointsNum, 0);
  (colorbuffer)->Upload(colorUpdate, sizeof(unsigned char) * 3 * pointsNum, 0);

  vertexBuffer_ = vertexbuffer;
  colorBuffer_ = colorbuffer;
  std::cout << "Process lidar frame!\n";
}


void lidar_callback(const sensor_msgs::PointCloud2::ConstPtr& lidar_msg)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr lidar_show(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*lidar_msg, *lidar_show);
    std::string lidar_name = "lidar_cloud";
    RadarCalibration lidar_cali(lidar_show, lidar_name);
    lidar_cali.showPoint();
    if (sys_pause)
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr lidar_cloud(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::fromROSMsg(*lidar_msg, *lidar_cloud);
        pcl::io::savePCDFileASCII(pkg_loc + "/data/lidar.pcd", *lidar_cloud);
        std::cout << "Lidar PCD file saved!" << std::endl;
        start_lidar_continue = true;
        cv::destroyWindow(lidar_name);
        lidar_sub.shutdown();
    }
}

// void radar_callback(const radar_lidar_static_calibration::ObjectList::ConstPtr& radar_msg)
// {
//     pcl::PointCloud<pcl::PointXYZ>::Ptr radar_show(new pcl::PointCloud<pcl::PointXYZ>());
//     for(auto object : radar_msg->objects){
//         pcl::PointXYZ p;
//         p.x = object.position.pose.position.x;
//         p.y = object.position.pose.position.y;
//         p.z = object.position.pose.position.z;
//         radar_show->push_back(p);
//     }
//     std::string radar_name = "radar_cloud";
//     RadarCalibration radar_cali(radar_show, radar_name);
//     radar_cali.showPoint();
//     if (sys_pause)
//     {
//         pcl::io::savePCDFileASCII(pkg_loc + "/data/radar.pcd", *radar_show);
//         std::cout << "Radar PCD file saved!" << std::endl;
//         start_radar_continue = true;
//         cv::destroyWindow(radar_name);
//         radar_sub.shutdown();
//     }
// }

void radar_callback(const visualization_msgs::MarkerArray::ConstPtr& radar_msg)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr radar_show(new pcl::PointCloud<pcl::PointXYZ>());
    for(auto object : radar_msg->markers){
        pcl::PointXYZ p;
        p.x = object.pose.position.x;
        p.y = object.pose.position.y;
        p.z = object.pose.position.z;
        radar_show->push_back(p);
    }
    std::string radar_name = "radar_cloud";
    RadarCalibration radar_cali(radar_show, radar_name);
    radar_cali.showPoint();
    if (sys_pause)
    {
        pcl::io::savePCDFileASCII(pkg_loc + "/data/radar.pcd", *radar_show);
        std::cout << "Radar PCD file saved!" << std::endl;
        start_radar_continue = true;
        cv::destroyWindow(radar_name);
        radar_sub.shutdown();
    }
} 


// 获取键盘输入
int scanKeyboard()
{
    int in;
    struct termios new_settings;
    struct termios stored_settings;
    tcgetattr(0, &stored_settings);
    new_settings = stored_settings;
    new_settings.c_lflag &= (~ICANON);
    new_settings.c_cc[VTIME] = 0;
    tcgetattr(0, &stored_settings);
    new_settings.c_cc[VMIN] = 1;
    tcsetattr(0, TCSANOW, &new_settings);

    in = getchar();

    tcsetattr(0, TCSANOW, &stored_settings);
    return in;
}

void stop2calibration()
{
    while (ros::ok())
    {
        if (scanKeyboard() == 32)
        {
            sys_pause = true;
            if (sys_pause)
            {
                std::cout << "Start to calibrate!" << std::endl;
            }
            else
            {
                std::cout << "Calibrate end!" << std::endl;
            }
        }
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "radar_lidar_calibration");
    pkg_loc = ros::package::getPath("radar_lidar_calibration");

    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    string lidar_topic, radar_topic;

    private_nh.param<string>("lidar_topic", lidar_topic, "/lidar_PointCloud2");
    private_nh.param<string>("radar_topic", radar_topic, "/radar_PointCloud2");

    radar_sub = nh.subscribe(radar_topic, 1, radar_callback);
    lidar_sub = nh.subscribe(lidar_topic, 1, lidar_callback);

    std::thread stop_to_calibrate(stop2calibration);
    stop_to_calibrate.detach();
    std::cout << "Press Space to capture lidar and radar pcd" << std::endl;

    while (ros::ok())
    {
        if(start_lidar_continue && start_radar_continue){
            break;
        }
        else{
            ros::spinOnce();
        }
    }


    std::cout << "Exiting program." << std::endl;
 

    string lidar_path = pkg_loc + "/data/lidar.pcd";
    string radar_path = pkg_loc + "/data/radar.pcd";
    string extrinsic_json = pkg_loc + "/data/extrinsic.json";

      // load lidar points
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(
        new pcl::PointCloud<pcl::PointXYZI>); // 创建点云（指针）
    if (pcl::io::loadPCDFile<pcl::PointXYZI>(lidar_path, *cloud) ==
        -1) //* 读入PCD格式的文件，如果文件不存在，返回-1
    {
        PCL_ERROR("Couldn't read lidar file \n");
        return (-1);
    }
    pcl::PointCloud<pcl::PointXYZI> pcd = *cloud;
          // load radar points
    pcl::PointCloud<pcl::PointXYZ>::Ptr radar_cloud(
        new pcl::PointCloud<pcl::PointXYZ>); // 创建点云（指针）
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(radar_path, *radar_cloud) ==
        -1) //* 读入PCD格式的文件，如果文件不存在，返回-1
    {
        PCL_ERROR("Couldn't read radar file \n");
        return (-1);
    }
    pcl::PointCloud<pcl::PointXYZ> radar_pcd = *radar_cloud;

    // load extrinsic
    Eigen::Matrix4d json_param;
    LoadExtrinsic(extrinsic_json, json_param);
    std::cout << "radar to lidar extrinsic:\n" << json_param << std::endl;

    cout << "Loading data completed!" << endl;
    CalibrationInit(json_param);
    const int width = 1920, height = 1280;
    pangolin::CreateWindowAndBind("radar2lidar player", width, height);

    glEnable(GL_DEPTH_TEST);
    glDepthMask(GL_TRUE);
    glDepthFunc(GL_LESS);

    pangolin::OpenGlRenderState s_cam(
        pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
        pangolin::ModelViewLookAt(-100, 0, 0, 0, 0, 0, 0.707, 0.0, 0.707));
    pangolin::View &d_cam = pangolin::CreateDisplay()
                                .SetBounds(0.0, 1.0, pangolin::Attach::Pix(150),
                                            1.0, -1.0 * width / height)
                                .SetHandler(new pangolin::Handler3D(s_cam));
    glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
    pangolin::OpenGlMatrix Twc; // camera to world
    Twc.SetIdentity();

    // control panel
    pangolin::CreatePanel("cp").SetBounds(pangolin::Attach::Pix(30), 1.0, 0.0,
                                            pangolin::Attach::Pix(150));
    pangolin::Var<bool> displayMode("cp.Intensity Color", display_mode_,
                                    true); // logscale
    pangolin::Var<double> degreeStep("cp.deg step", cali_scale_degree_, 0,
                                    1); // logscale
    pangolin::Var<double> tStep("cp.t step(cm)", 6, 0, 15);
    pangolin::Var<int> pointSize("cp.radar line size", radar_line_size_, 1, 10);

    pangolin::Var<bool> addXdegree("cp.+ x degree", false, false);
    pangolin::Var<bool> minusXdegree("cp.- x degree", false, false);
    pangolin::Var<bool> addYdegree("cp.+ y degree", false, false);
    pangolin::Var<bool> minusYdegree("cp.- y degree", false, false);
    pangolin::Var<bool> addZdegree("cp.+ z degree", false, false);
    pangolin::Var<bool> minusZdegree("cp.- z degree", false, false);
    pangolin::Var<bool> addXtrans("cp.+ x trans", false, false);
    pangolin::Var<bool> minusXtrans("cp.- x trans", false, false);
    pangolin::Var<bool> addYtrans("cp.+ y trans", false, false);
    pangolin::Var<bool> minusYtrans("cp.- y trans", false, false);
    pangolin::Var<bool> addZtrans("cp.+ z trans", false, false);
    pangolin::Var<bool> minusZtrans("cp.- z trans", false, false);

    pangolin::Var<bool> auto_calibrate("cp. icp_auto_calibrate", false, false);

    pangolin::Var<bool> resetButton("cp.Reset", false, false);
    pangolin::Var<bool> saveImg("cp.Save Image", false, false);

    std::vector<pangolin::Var<bool>> mat_calib_box;
    mat_calib_box.push_back(addXdegree);
    mat_calib_box.push_back(minusXdegree);
    mat_calib_box.push_back(addYdegree);
    mat_calib_box.push_back(minusYdegree);
    mat_calib_box.push_back(addZdegree);
    mat_calib_box.push_back(minusZdegree);
    mat_calib_box.push_back(addXtrans);
    mat_calib_box.push_back(minusXtrans);
    mat_calib_box.push_back(addYtrans);
    mat_calib_box.push_back(minusYtrans);
    mat_calib_box.push_back(addZtrans);
    mat_calib_box.push_back(minusZtrans);

    int frame_num = 0;
    int lidar_point_size = cloud->points.size();
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_radar_cloud(
        new pcl::PointCloud<pcl::PointXYZ>());
    ProcessSingleFrame(cloud, display_mode_);

    std::cout << "\n=>START\n";
    while (!pangolin::ShouldQuit()) {
        s_cam.Follow(Twc);
        d_cam.Activate(s_cam);
        glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        if (displayMode) {
        if (display_mode_ == false) {
            display_mode_ = true;
            ProcessSingleFrame(cloud, display_mode_);
        }
        } else {
        if (display_mode_ == true) {
            display_mode_ = false;
            ProcessSingleFrame(cloud, display_mode_);
        }
        }

        if (degreeStep.GuiChanged()) {
        cali_scale_degree_ = degreeStep.Get();
        CalibrationScaleChange();
        std::cout << "Degree calib scale changed to " << cali_scale_degree_
                    << " degree\n";
        }
        if (tStep.GuiChanged()) {
        cali_scale_trans_ = tStep.Get() / 100.0;
        CalibrationScaleChange();
        std::cout << "Trans calib scale changed to " << cali_scale_trans_ * 100
                    << " cm\n";
        }
        if (pointSize.GuiChanged()) {
        radar_line_size_ = pointSize.Get();
        std::cout << "radar line size changed to " << radar_line_size_
                    << std::endl;
        }
        for (int i = 0; i < 12; i++) {
        if (pangolin::Pushed(mat_calib_box[i])) {
            calibration_matrix_ = calibration_matrix_ * modification_list_[i];
            cout << "\nTransfromation Matrix:\n" << calibration_matrix_ << std::endl;
            std::cout << "Changed!\n";
        }
        }

        if (pangolin::Pushed(auto_calibrate)) {    
        Eigen::Matrix4d matrix_trans = AutoCalibration(cloud, transformed_radar_cloud);
        calibration_matrix_ = matrix_trans * calibration_matrix_;
        cout << "\nTransfromation Matrix:\n" << calibration_matrix_ << std::endl;
        }

        if (pangolin::Pushed(resetButton)) {
        calibration_matrix_ = orign_calibration_matrix_;
        std::cout << "Reset!\n";
        }
        if (pangolin::Pushed(saveImg)) {
        saveResult(frame_num);
        std::cout << "\n==>Save Result " << frame_num << std::endl;
        Eigen::Matrix4d transform = calibration_matrix_;
        cout << "Transfromation Matrix:\n" << transform << std::endl;
        frame_num++;
        }

        if (kbhit()) {
        int c = getchar();
        if (ManualCalibration(c)) {
            Eigen::Matrix4d transform = calibration_matrix_;
            cout << "\nTransfromation Matrix:\n" << transform << std::endl;
        }
        }

        // draw points
        glDisable(GL_LIGHTING);
        glPointSize(2);
        // draw lidar points
        colorBuffer_->Bind();
        glColorPointer(colorBuffer_->count_per_element, colorBuffer_->datatype, 0,
                    0);
        glEnableClientState(GL_COLOR_ARRAY);
        vertexBuffer_->Bind();
        glVertexPointer(vertexBuffer_->count_per_element, vertexBuffer_->datatype,
                        0, 0);
        glEnableClientState(GL_VERTEX_ARRAY);
        glDrawArrays(GL_POINTS, 0, lidar_point_size);
        glDisableClientState(GL_VERTEX_ARRAY);
        vertexBuffer_->Unbind();
        glDisableClientState(GL_COLOR_ARRAY);
        colorBuffer_->Unbind();

        // draw radar lines

        transformed_radar_cloud->clear();
        pcl::transformPointCloud(*radar_cloud, *transformed_radar_cloud,
                                calibration_matrix_);
        for (int i = 0; i < radar_cloud->points.size(); i++) {
        double x = transformed_radar_cloud->points[i].x;
        double y = transformed_radar_cloud->points[i].y;
        double z = transformed_radar_cloud->points[i].z;
        glLineWidth(radar_line_size_);
        glColor3f(1.0f, 0.0f, 0.0f);
        glBegin(GL_LINES);
        glVertex3d(x, y, z);
        glVertex3d(x, y, z + 2.5);
        glEnd();
        }
        pangolin::FinishFrame();
        usleep(100);
        glFinish();
    }

    // delete[] imageArray;

    Eigen::Matrix4d transform = calibration_matrix_;
    cout << "\nFinal Transfromation Matrix:\n" << transform << std::endl;
    return 0;
}

