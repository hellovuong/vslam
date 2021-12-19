//
// Created by vuong on 16/12/2021.
//

#include "node.h"

#include <utility>
node::node(ORB_SLAM3::System::eSensor sensor,
           ros::NodeHandle& node_handle,
           image_transport::ImageTransport& image_transport,
           std::string strOutput)
    : mSensor(sensor),
      nh_(node_handle),
      image_transport_(image_transport),
      strOutputFile(std::move(strOutput)) {
  mNodeName = ros::this_node::getName();
}

node::~node() {}
void node::Init() {
  // Retrieve static parameters
  nh_.getParam("Vocab_path", strVocFile);
  nh_.getParam("Params", strSettingsFile);
  nh_.getParam("Visualize", mbViewer);
  // Retrieve frame id parameters
  nh_.getParam("world_frame_id", world_frame_id_);  // world_frame
  nh_.getParam("left_camera_frame_id",
               left_cam_frame_id_);  // left_camera_frame
  nh_.getParam("point_cloud_frame_id", point_cloud_frame_id_);  // point cloud

  // Init ORB_SLAM3
  mpORB_SLAM3 =
      new ORB_SLAM3::System(strVocFile, strSettingsFile, mSensor, mbViewer);

  // Bring the constructors here to get the state robot easier
  // TODO: Optimize this
  //  mpORB_SLAM3->SetThreads(mpLocalMapping, mpMapDrawer, mpAtlas);
  mpLocalMapping = mpORB_SLAM3->mpLocalMapper;
  mpMapDrawer = mpORB_SLAM3->mpMapDrawer;
  mpAtlas = mpORB_SLAM3->mpAtlas;
  // Initialization transformation listener
  //  tfBuffer.reset(new tf2_ros::Buffer);
  //  tfListener.reset(new tf2_ros::TransformListener(*tfBuffer));

  // Advertise topics
  // Debug image
  //  mDebugImagePub = image_transport_.advertise(mNodeName + "/DebugImage", 1);
  // Map Point
  // TODO: Only start to publish when ORB_SLAM3 finished InitializeIMU or
  // InertialBA2
  mMapPointsPub =
      nh_.advertise<sensor_msgs::PointCloud2>(mNodeName + "/MapPoints", 1);
  // Pose
  // TODO: Only start to publish when ORB_SLAM3 finished InitializeIMU or
  // InertialBA2
  mPosePub = nh_.advertise<nav_msgs::Odometry>(mNodeName + "/Pose", 1);

  // For Depth Estimation
  mKFDebugImagePub =
      image_transport_.advertise(mNodeName + "/KF_DebugImage", 1);
  mKFPosePub = nh_.advertise<nav_msgs::Odometry>(mNodeName + "/KF_Pose", 1);
  mMPsObsbyKFPub =
      nh_.advertise<sensor_msgs::PointCloud>(mNodeName + "/KF_MapPoints", 1);
  mKFsFeatures =
      nh_.advertise<sensor_msgs::PointCloud>(mNodeName + "/KF_Features", 1);
  //  cout << " here1" << endl;
}
void node::Update() {
  //  cout << " here2" << endl;
  if (!mpAtlas->GetCurrentMap()->isImuInitialized()) return;
  cv::Mat cvTcw;
  double timestamp;
  mpMapDrawer->GetCurrentCameraPose(cvTcw);
  //  cout << " here3" << endl;
  mpMapDrawer->GetCurrentCameraTimestamp(timestamp);

  if (!cvTcw.empty()) {
    //    cout << " here3" << endl;
    cv::cv2eigen(cvTcw, eTcw);
    eTcw.block<3, 3>(0, 0) = Eigen::Quaterniond(eTcw.block<3, 3>(0, 0))
                                 .normalized()
                                 .toRotationMatrix();
    Sophus::SE3d spTcw = Sophus::SE3d(eTcw);
    spTwc = spTcw.inverse();
    // Publish current pose (Transformation from camera to world)
    //    PublishPoseAsTransform(spTwc, timestamp);
    PublishPoseAsOdometry(spTwc, timestamp);
    std::vector<ORB_SLAM3::MapPoint*> vpMapPoints =
        mpAtlas->GetCurrentMap()->GetAllMapPoints();
    PublishMapPointsAsPCL2(vpMapPoints, 0);
  }
  if (!mpLocalMapping->mlProcessdKFs.empty()) {
    // TODO: Adding mutex
    ORB_SLAM3::KeyFrame* pKF = mpLocalMapping->mlProcessdKFs.front();
    mpLocalMapping->mlProcessdKFs.pop_front();
    PublishKF(pKF);
  }
}
void node::PublishPoseAsTransform(const Sophus::SE3d& Twc, double timestamp) {
  tf2::Transform tfCamPose;
  // Convert to tf format
  tfCamPose = Utils::toTransformMsg(Twc);
  //  ros::Time rosTimeStamp;
  // Generate Msg
  tf2::Stamped<tf2::Transform> tfStamped = tf2::Stamped<tf2::Transform>(
      tfCamPose, Utils::toROSTime(timestamp), world_frame_id_);
  geometry_msgs::TransformStamped tfMsg;
  tf2::toMsg(tfMsg);
  // Broadcast tf
  static tf2_ros::TransformBroadcaster tf_broadcaster;
  tf_broadcaster.sendTransform(tfMsg);
}
void node::PublishMapPointsAsPCL2(std::vector<ORB_SLAM3::MapPoint*> vpMapPoints,
                                  double timestamp) {
  if (vpMapPoints.empty()) {
    ROS_WARN("Empty Map Points");
    return;
  }
  sensor_msgs::PointCloud2 cloud;

  const int num_channels = 3;  // x y z

  cloud.header.stamp = Utils::toROSTime(timestamp);
  cloud.header.frame_id = point_cloud_frame_id_;
  cloud.height = 1;
  cloud.width = vpMapPoints.size();
  cloud.is_bigendian = false;
  cloud.is_dense = true;
  cloud.point_step = num_channels * sizeof(float);
  cloud.row_step = cloud.point_step * cloud.width;
  cloud.fields.resize(num_channels);

  std::string channel_id[] = {"x", "y", "z"};
  for (int i = 0; i < num_channels; i++) {
    cloud.fields[i].name = channel_id[i];
    cloud.fields[i].offset = i * sizeof(float);
    cloud.fields[i].count = 1;
    cloud.fields[i].datatype = sensor_msgs::PointField::FLOAT32;
  }
  cloud.data.resize(cloud.row_step * cloud.height);

  unsigned char* cloud_data_ptr = &(cloud.data[0]);

  float data_array[num_channels];
  int min_observations_per_point_ = 2;
  for (unsigned int i = 0; i < cloud.width; i++) {
    // TODO: check this: coord diff between
    if (vpMapPoints.at(i)->nObs >= min_observations_per_point_) {
      data_array[0] = vpMapPoints.at(i)->GetWorldPos().at<float>(0);  // x.
      data_array[1] = vpMapPoints.at(i)->GetWorldPos().at<float>(1);  // y.
      data_array[2] = vpMapPoints.at(i)->GetWorldPos().at<float>(2);  // z.
      memcpy(cloud_data_ptr + (i * cloud.point_step),
             data_array,
             num_channels * sizeof(float));
    }
  }
  mMapPointsPub.publish(cloud);
}
void node::PublishKF(ORB_SLAM3::KeyFrame* pKF) {
  Eigen::Matrix4d eTwc;
  cv::cv2eigen(pKF->GetPoseInverse(), eTwc);
  // Get Pose (Twc)
  eTwc.block<3, 3>(0, 0) = Eigen::Quaterniond(eTwc.block<3, 3>(0, 0))
                               .normalized()
                               .toRotationMatrix();
  Sophus::SE3d Twc(eTwc);

  // Get MapPoints
  std::vector<ORB_SLAM3::MapPoint*> vpMapPoints = pKF->GetMapPointMatches();

  // Get timestamps
  double timestamp = pKF->mTimeStamp;

  // Get Image
  cv::Mat imKF = pKF->imgLeft.clone();
  // Start Publish
  // Pose (Twc)
  nav_msgs::Odometry PoseMsg;
  PoseMsg.header.stamp = Utils::toROSTime(timestamp);
  PoseMsg.header.frame_id = std::to_string(pKF->mnId);

  PoseMsg.pose.pose.orientation.x = Twc.unit_quaternion().x();
  PoseMsg.pose.pose.orientation.y = Twc.unit_quaternion().y();
  PoseMsg.pose.pose.orientation.z = Twc.unit_quaternion().z();
  PoseMsg.pose.pose.orientation.w = Twc.unit_quaternion().w();

  PoseMsg.pose.pose.position.x = Twc.translation().x();
  PoseMsg.pose.pose.position.y = Twc.translation().y();
  PoseMsg.pose.pose.position.z = Twc.translation().z();

  mKFPosePub.publish(PoseMsg);

  // Map Points
  sensor_msgs::PointCloud cloud;
  cloud.header.stamp = Utils::toROSTime(timestamp);
  cloud.header.frame_id = std::to_string(pKF->mnId);
  cloud.points.resize(vpMapPoints.size());
  // we'll also add an intensity channel to the cloud
  cloud.channels.resize(1);
  cloud.channels[0].name = "intensities";
  cloud.channels[0].values.resize(vpMapPoints.size());

  // Corresponding Features
  sensor_msgs::PointCloud cloud_feature;
  cloud_feature.header.stamp = Utils::toROSTime(timestamp);
  cloud_feature.header.frame_id = std::to_string(pKF->mnId);
  cloud_feature.points.resize(vpMapPoints.size());
  // we'll also add an intensity channel to the cloud
  cloud_feature.channels.resize(1);
  cloud_feature.channels[0].name = "intensities";
  cloud_feature.channels[0].values.resize(vpMapPoints.size());

  for (size_t i = 0; i < vpMapPoints.size(); i++) {
    ORB_SLAM3::MapPoint* pMP = vpMapPoints[i];
    if (pMP && !pMP->isBad()) {
      cloud.points[i].x = pMP->GetWorldPos().at<float>(0);
      cloud.points[i].y = pMP->GetWorldPos().at<float>(1);
      cloud.points[i].z = pMP->GetWorldPos().at<float>(2);

      cloud_feature.points[i].x = pKF->mvKeys[i].pt.x;
      cloud_feature.points[i].y = pKF->mvKeys[i].pt.y;
      cloud_feature.points[i].z = 0;
    }
  }
  //  cv::imwrite("KF.png",imKF);
  // Image
  sensor_msgs::ImagePtr img_msg;
  sensor_msgs::Image std_img_msg;
  std_img_msg.header.stamp.fromSec(timestamp);
  std_img_msg.header.frame_id = "Left_frame";
  img_msg = cv_bridge::CvImage(std_img_msg.header, "mono8", imKF).toImageMsg();
  // Publish
  mMPsObsbyKFPub.publish(cloud);
  mKFsFeatures.publish(cloud_feature);
  mKFDebugImagePub.publish(img_msg);
}
void node::PublishPoseAsOdometry(const Sophus::SE3d& Twc, double timestamp) {
  nav_msgs::Odometry PoseMsg;
  PoseMsg.header.stamp = Utils::toROSTime(timestamp);
  PoseMsg.header.frame_id = "left_cam_pose";

  PoseMsg.pose.pose.orientation.x = Twc.unit_quaternion().x();
  PoseMsg.pose.pose.orientation.y = Twc.unit_quaternion().y();
  PoseMsg.pose.pose.orientation.z = Twc.unit_quaternion().z();
  PoseMsg.pose.pose.orientation.w = Twc.unit_quaternion().w();

  PoseMsg.pose.pose.position.x = Twc.translation().x();
  PoseMsg.pose.pose.position.y = Twc.translation().y();
  PoseMsg.pose.pose.position.z = Twc.translation().z();

  mPosePub.publish(PoseMsg);
}
