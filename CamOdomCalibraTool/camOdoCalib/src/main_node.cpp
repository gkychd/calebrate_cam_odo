/*******************************************************
 * Copyright (C) 2019, SLAM Group, Megvii-R
 *******************************************************/

#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>
#include <stdio.h>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <queue>
#include <thread>
#include <fstream>//zdf 
#include <math.h>
#include <chrono>

#include <sensor_msgs/Image.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/image_encodings.h>

#include "camera_models/include/Camera.h"
#include "camera_models/include/CameraFactory.h"
#include "calc_cam_pose/calcCamPose.h"

#include "solveQyx.h" 
#include "cam_odo_cal/cam_data.h"
typedef boost::shared_ptr<nav_msgs::Odometry const> OdomConstPtr;
typedef boost::shared_ptr<cam_odo_cal::cam_data const> CamConstPtr;

std::queue<OdomConstPtr> odo_buf;
std::queue<sensor_msgs::ImageConstPtr> img_buf;
std::mutex m_buf;
bool start = false ;//zdf
std::vector<data_selection::odo_data> odoDatas;
std::vector<data_selection::cam_data> camDatas;

//record the first frame calculated successfully
bool fisrt_frame = true;
Eigen::Matrix3d Rwc0;
Eigen::Vector3d twc0;
//decide if the frequent is decreased
bool halfFreq = false;
int frame_index = 0;

void wheel_callback(const OdomConstPtr &odo_msg)
{
  double time = odo_msg->header.stamp.toSec();
  //ROS_INFO("odoDatas.time: %lf", time);
  Eigen::Vector3d linear = { odo_msg->twist.twist.linear.x,
    odo_msg->twist.twist.linear.y,
    odo_msg->twist.twist.linear.z };
    Eigen::Vector3d angular = { odo_msg->twist.twist.angular.x,
      odo_msg->twist.twist.angular.y,
      odo_msg->twist.twist.angular.z };
      data_selection::odo_data odo_tmp;

    odo_tmp.time = time;
    odo_tmp.v_left = linear[0] / 0.1 - angular[2]*0.57 / (2*0.1);// linear velcity of x axis 
    odo_tmp.v_right = linear[0] / 0.1 + angular[2]*0.57 / (2*0.1);// angular velcity of z axis
    odoDatas.push_back(odo_tmp);
    //ROS_INFO("odoDatas.size: %d", odoDatas.size());
  }

void cam_callback(const CamConstPtr &img_msg)
{
  data_selection::cam_data cam_tmp;
  Eigen::Quaterniond qcl;
  Eigen::Matrix3d Rcl;
  qcl.w() = img_msg->qcl[0];
  qcl.x() = img_msg->qcl[1];
  qcl.y() = img_msg->qcl[2];
  qcl.z() = img_msg->qcl[3];
  //四元数---->旋转矩阵
  Rcl = qcl.normalized().toRotationMatrix();
  //四元数----->旋转向量
  Eigen::AngleAxisd rotation_vector(qcl);
  Eigen::Vector3d axis = rotation_vector.axis();
  double angle = rotation_vector.angle();

  cam_tmp.start_t = img_msg->start_t;
  cam_tmp.end_t = img_msg->end_t;
  //cam_tmp.theta_y = theta_y;
  cam_tmp.deltaTheta = img_msg->deltaTheta; // cam_tmp.deltaTheta is deltaTheta_lc
  cam_tmp.axis = axis;
  cam_tmp.Rcl =  Rcl;
  if(cam_tmp.axis[1] < 0){
    cam_tmp.axis *= -1;
    cam_tmp.deltaTheta *= -1;
  }
  cam_tmp.tlc[0] =  img_msg->tlc[0];
  cam_tmp.tlc[1] =  img_msg->tlc[1];
  cam_tmp.tlc[2] =  img_msg->tlc[2];
  camDatas.push_back(cam_tmp);
  ROS_INFO("camDatas.size: %d", camDatas.size());
}

// extract images with same timestamp from two topics
void calc_process(const CameraPtr &cam)
{
  Eigen::Matrix3d Rwl;
  Eigen::Vector3d twl;
  double t_last = 0.0;//time of last image
  bool first = true; //judge if last frame was calculated successfully 

  std::cout << std::endl << "images counts waiting for processing: " << std::endl;
  while (1)
  {
    if(camDatas.size() == 1482){
      start = true;
    }
    if(start){
      SolveQyx cSolveQyx;
      std::vector<data_selection::cam_data> camDatas2 = camDatas;
      std::vector<data_selection::odo_data> odoDatas2 = odoDatas;
      std::cout << "============ calibrating... ===============" << std::endl;
      data_selection ds;
      std::vector<data_selection::sync_data> sync_result;
      ds.selectData(odoDatas2,camDatas2,sync_result);

     //first estimate the Ryx and correct tlc of camera
     Eigen::Matrix3d Ryx;
     cSolveQyx.estimateRyx(sync_result,Ryx);
     std::cout << "cam.size(): " << camDatas2.size() << std::endl;
     std::cout << "sync_result.size(): " << sync_result.size() << std::endl;
     cSolveQyx.correctCamera(sync_result,camDatas2,Ryx);

      //calibrate r_L  r_R  axle  lx  ly  yaw
      cSolver cSolve;
      cSolver::calib_result paras;//radius_l,radius_r,axle,l[3]
      cSolve.calib(sync_result, 4,paras); // by svd

      //secondly estimate the Ryx
      cSolveQyx.estimateRyx(sync_result,Ryx);

      //refine all the extrinal parameters
      cSolveQyx.refineExPara(sync_result,paras,Ryx);
      
      break;
    } 
    std::chrono::milliseconds dura(2);
    std::this_thread::sleep_for(dura);
 }
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "simulation_calebration");
    ros::NodeHandle n("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);

    std::string config_file = argv[1];
    cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
    if (!fsSettings.isOpened())
    {
      std::cerr << "ERROR: Wrong path to settings" << std::endl;
    }

    std::string WHEEL_TOPIC, IMAGE_TOPIC;
    fsSettings["wheel_topic"] >> WHEEL_TOPIC;
    fsSettings["image_topic"] >> IMAGE_TOPIC;
    std::cout << "wheel_topic: " << WHEEL_TOPIC << std::endl;
    std::cout << "image_topic: " << IMAGE_TOPIC << std::endl;
    CameraPtr camera = CameraFactory::instance()->generateCameraFromYamlFile(config_file);

    //the following three rows are to run the calibrating project through playing bag package
    ros::Subscriber sub_imu = n.subscribe(WHEEL_TOPIC, 500, wheel_callback, ros::TransportHints().tcpNoDelay());
    ros::Subscriber sub_img = n.subscribe(IMAGE_TOPIC, 200, cam_callback);
    std::thread calc_thread = std::thread{calc_process, std::ref(camera)};
    
     ros::spin();
     return 0;
}
