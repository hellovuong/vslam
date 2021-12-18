//
// Created by vuong on 16/12/2021.
//

#include "utils/Utils.h"
tf2::Transform Utils::toTransformMsg(Sophus::SE3d Twc) {
  // TODO (Vuong): Do we need to change ORB coord to ROS Coordinate
  tf2::Quaternion tf2R_quat(Twc.so3().unit_quaternion().x(),
                            Twc.so3().unit_quaternion().y(),
                            Twc.so3().unit_quaternion().z(),
                            Twc.so3().unit_quaternion().w());
  tf2::Vector3 tf2p(
      Twc.translation().x(), Twc.translation().y(), Twc.translation().z());
  return tf2::Transform(tf2R_quat, tf2p);
}
ros::Time Utils::toROSTime(double timestamp) {
  ros::Time output;
  output.fromSec(timestamp);
  return output;
}
