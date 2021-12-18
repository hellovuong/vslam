//
// Created by vuong on 16/12/2021.
//

#ifndef VSLAM_UTILS_H
#define VSLAM_UTILS_H

// From ORB_SLAM3
#include <MapPoint.h>

// From ROS
#include <sensor_msgs/PointCloud2.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// From Sophus
#include <sophus/se3.hpp>

namespace Utils {

tf2::Transform toTransformMsg(Sophus::SE3d Twc);
ros::Time toROSTime(double timestamp);
//void TftoTarget()
};

#endif  // VSLAM_UTILS_H
