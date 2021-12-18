//
// Created by vuong on 16/12/2021.
//

#ifndef VSLAM_NODE_H
#define VSLAM_NODE_H

// From ORB_SLAM3
#include <System.h>
#include <LocalMapping.h>
// From ROS
//  Core
#include <ros/ros.h>
#include <ros/time.h>
#include <cv_bridge/cv_bridge.h>
//  tf2
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
//  Msg Types
#include <geometry_msgs/PoseStamped.h>
#include <image_transport/image_transport.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
// From Eigen & Sophus
#include <eigen3/Eigen/Dense>
#include <sophus/se3.hpp>

// From OpenCV
#include <opencv2/core/eigen.hpp>

#include "utils/Utils.h"

class Viewer;
class FrameDrawer;
class Atlas;
class Tracking;
class LocalMapping;

class node {
 public:
  node(ORB_SLAM3::System::eSensor sensor,
       ros::NodeHandle& node_handle,
       image_transport::ImageTransport& image_transport,
       std::string strOutput = std::string());
  ~node();

  // Advertise publish topics
  void Init();

//  cv::Mat cvTcw;

 protected:
  // Update state and publish data
  void Update();
  // Pointer to ORB_SLAM3 threads
  // System
  ORB_SLAM3::System* mpORB_SLAM3{};
  // Tracking
  //  ORB_SLAM3::Tracking* mpTracking{};
  // Local Mapping
  ORB_SLAM3::LocalMapping* mpLocalMapping;  // Used
  // Loop Closing
  //  ORB_SLAM3::LoopClosing* mpLoopClosing{};
  // Map Drawer
  ORB_SLAM3::MapDrawer* mpMapDrawer{};  // Used
  // Atlas
  ORB_SLAM3::Atlas* mpAtlas{};  // Used

  // ORB_SLAM3 variables
  ORB_SLAM3::System::eSensor mSensor;
  // Path variables
  std::string strOutputFile;

 private:
  // Functions for Publish
  void PublishPoseAsTransform(const Sophus::SE3d& Twc, double timestamp);
  void PublishPoseAsOdometry(const Sophus::SE3d& Twc, double timestamp);
  void PublishMapPointsAsPCL2(std::vector<ORB_SLAM3::MapPoint*> vpMapPoints,
                              double timestamp);
  void PublishKF(ORB_SLAM3::KeyFrame* pKF);
  // initialization Transform listener
  boost::shared_ptr<tf2_ros::Buffer> tfBuffer;
  boost::shared_ptr<tf2_ros::TransformListener> tfListener;


  ros::NodeHandle nh_;

  // Node's name
  std::string mNodeName;

  // Frame IDs for Odometry Publish
  std::string world_frame_id_;
  std::string left_cam_frame_id_;
  std::string point_cloud_frame_id_;


  std::string strVocFile;
  std::string strSettingsFile;

  // Publish variables
  // Map
  image_transport::ImageTransport image_transport_;
  image_transport::Publisher mDebugImagePub;
  ros::Publisher mPosePub;
  ros::Publisher mMapPointsPub;

  // KF for Depth Estimation
  ros::Publisher mKFPosePub;
  image_transport::Publisher mKFDebugImagePub;
  ros::Publisher mMPsObsbyKFPub;
  ros::Publisher mKFsFeatures;

  // Robot state
  Eigen::Matrix4d eTcw;
  Sophus::SE3d spTwc;
};

#endif  // VSLAM_NODE_H
