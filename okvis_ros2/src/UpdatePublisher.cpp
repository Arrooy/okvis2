/*********************************************************************************
 *  OKVIS - Open Keyframe-based Visual-Inertial SLAM
 *  Copyright (c) 2015, Autonomous Systems Lab / ETH Zurich
 *  Copyright (c) 2020, Smart Robotics Lab / Imperial College London
 *  Copyright (c) 2024, Smart Robotics Lab / Technical University of Munich
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.
 *   * Neither the name of Autonomous Systems Lab, ETH Zurich, Smart Robotics Lab,
 *     Imperial College London, Technical University of Munich, nor the names of
 *     its contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 *  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************************/

/**
 * @file UpdatePublisher.cpp
 * @brief Source file for the UpdatePublisher class.
 * @author Stefan Leutenegger
 * @author Andreas Forster
 */

#include <glog/logging.h>
#include <okvis/ros2/UpdatePublisher.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <okvis/FrameTypedefs.hpp>

#include "okvis_ros2_interfaces/msg/update.hpp"

/// \brief okvis Main namespace of this package.
namespace okvis {

// Default constructor.
UpdatePublisher::UpdatePublisher(std::shared_ptr<rclcpp::Node> node)
    : trajectoryOutput_(false), trajectoryLocked_(false)
{
  setupNode(node);
}

void UpdatePublisher::setupNode(std::shared_ptr<rclcpp::Node> node)
{
  // set up node
  node_ = node;

  // set up publishers
  pubTf_.reset(new tf2_ros::TransformBroadcaster(node_));
  pubObometry_ = node_->create_publisher<nav_msgs::msg::Odometry>("okvis_odometry", 1);
  pubPath_ = node_->create_publisher<visualization_msgs::msg::Marker>("okvis_path", 1);
  pubTransform_ = node_->create_publisher<geometry_msgs::msg::TransformStamped>(
      "okvis_transform", 1);
  pubMesh_ = node_->create_publisher<visualization_msgs::msg::Marker>("okvis_mesh", 0 );
  pubPointsMatched_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
      "okvis_points_matched", 0 );

  // get the mesh, if there is one
  // where to get the mesh from
  std::string mesh_file;
  bool loaded_mesh;
  node_->declare_parameter("mesh_file", "");
  loaded_mesh = node_->get_parameter("mesh_file", mesh_file);
  if (loaded_mesh) {
    meshMsg_.mesh_resource = mesh_file;

    // fill orientation
    meshMsg_.pose.orientation.x = 0;
    meshMsg_.pose.orientation.y = 0;
    meshMsg_.pose.orientation.z = 0;
    meshMsg_.pose.orientation.w = 1;

    // fill position
    meshMsg_.pose.position.x = 0;
    meshMsg_.pose.position.y = 0;
    meshMsg_.pose.position.z = 0;

    // scale -- needed
    meshMsg_.scale.x = 1.0;
    meshMsg_.scale.y = 1.0;
    meshMsg_.scale.z = 1.0;

    meshMsg_.action = visualization_msgs::msg::Marker::ADD;
    meshMsg_.color.a = 1.0; // Don't forget to set the alpha!
    meshMsg_.color.r = 1.0;
    meshMsg_.color.g = 1.0;
    meshMsg_.color.b = 1.0;

    // embedded material / colour
    //meshMsg_.mesh_use_embedded_materials = true;
  } else {
    LOG(INFO) << "no mesh found for visualisation, set ros param mesh_file, if desired";
    meshMsg_.mesh_resource = "";
  }
}

UpdatePublisher::~UpdatePublisher()
{
}

void UpdatePublisher::setBodyTransform(const okvis::kinematics::Transformation& T_BS) {
  T_BS_ = T_BS;

  // publish pose:
  geometry_msgs::msg::TransformStamped poseMsg; // Pose message.
  poseMsg.child_frame_id = "sensor";
  poseMsg.header.frame_id = "body";
  poseMsg.header.stamp = node_->now();

  // fill orientation
  Eigen::Quaterniond q = T_BS_.q();
  poseMsg.transform.rotation.x = q.x();
  poseMsg.transform.rotation.y = q.y();
  poseMsg.transform.rotation.z = q.z();
  poseMsg.transform.rotation.w = q.w();

  // fill position
  Eigen::Vector3d r = T_BS_.r();
  poseMsg.transform.translation.x = r[0];
  poseMsg.transform.translation.y = r[1];
  poseMsg.transform.translation.z = r[2];
  static tf2_ros::StaticTransformBroadcaster br(node_);
  br.sendTransform(poseMsg);
}

void UpdatePublisher::setCsvFile(const std::string & filename, bool rpg)
{
  return trajectoryOutput_.setCsvFile(filename, rpg);
}

bool UpdatePublisher::realtimePredictAndPublish(const okvis::Time& stamp,
                         const Eigen::Vector3d& alpha,
                         const Eigen::Vector3d& omega) {

  // store in any case
  imuMeasurements_.push_back(ImuMeasurement(stamp, ImuSensorReadings(omega,alpha)));

  // add to Trajectory if possible
  bool success = false;
  State state;
  if(!trajectoryLocked_) {
    trajectoryLocked_ = true;
    for(auto & imuMeasurement : imuMeasurements_) {
      State propagatedState;
      if(trajectory_.addImuMeasurement(imuMeasurement.timeStamp,
                                       imuMeasurement.measurement.accelerometers,
                                       imuMeasurement.measurement.gyroscopes,
                                       propagatedState)) {
        state = propagatedState;
        success = true;
      }
    }
    imuMeasurements_.clear();
    trajectoryLocked_ = false;
    if(!success) {
      return false;
    }
  } else {
    return false;
  }

  // only publish according to rate
  if((state.timestamp - lastTime_).toSec()
      < (1.0/double(odometryPublishingRate_))) {
    return false;
  }
  lastTime_ = state.timestamp;

  rclcpp::Time t(state.timestamp.sec, state.timestamp.nsec); // Header timestamp.
  const okvis::kinematics::Transformation T_WS = state.T_WS;
  const kinematics::Transformation T_SB = T_BS_.inverse();
  const okvis::kinematics::Transformation T_WB = T_WS * T_SB;

  // Odometry
  nav_msgs::msg::Odometry odometryMsg;  // Odometry message.
  odometryMsg.header.frame_id = "world";
  odometryMsg.child_frame_id = "body";
  odometryMsg.header.stamp = t;

  // fill orientation
  const Eigen::Quaterniond q = T_WB.q();
  odometryMsg.pose.pose.orientation.x = q.x();
  odometryMsg.pose.pose.orientation.y = q.y();
  odometryMsg.pose.pose.orientation.z = q.z();
  odometryMsg.pose.pose.orientation.w = q.w();

  // fill position
  const Eigen::Vector3d r = T_WB.r();
  odometryMsg.pose.pose.position.x = r[0];
  odometryMsg.pose.pose.position.y = r[1];
  odometryMsg.pose.pose.position.z = r[2];

  // note: velocity and angular velocity needs to be expressed in child frame, i.e. "body"
  // see http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Odometry.html

  // fill velocity
  const kinematics::Transformation T_BW = T_WB.inverse();
  const Eigen::Vector3d v = T_BW.C() * state.v_W + T_BS_.C() * T_SB.r().cross(state.omega_S);
  // ...of body orig. represented in body
  odometryMsg.twist.twist.linear.x = v[0];
  odometryMsg.twist.twist.linear.y = v[1];
  odometryMsg.twist.twist.linear.z = v[2];

  // fill angular velocity
  const Eigen::Matrix3d C_BS = T_BS_.C();
  const Eigen::Vector3d omega_B = C_BS * state.omega_S; // of body represented in body
  odometryMsg.twist.twist.angular.x = omega_B[0];
  odometryMsg.twist.twist.angular.y = omega_B[1];
  odometryMsg.twist.twist.angular.z = omega_B[2];

  // publish odometry
  pubObometry_->publish(odometryMsg);
  return true;
}

void cvtState2Msg(const State& state, okvis_ros2_interfaces::msg::State & stateMsg) 
{
  // state
  stateMsg.header.frame_id = "world";
  stateMsg.header.stamp = rclcpp::Time(state.timestamp.sec, state.timestamp.nsec);
  // Transformation between World W and Sensor S.
  stateMsg.pose_ws.position.x = state.T_WS.r()[0];
  stateMsg.pose_ws.position.y = state.T_WS.r()[1];
  stateMsg.pose_ws.position.z = state.T_WS.r()[2];
  stateMsg.pose_ws.orientation.w = state.T_WS.q().w();
  stateMsg.pose_ws.orientation.x = state.T_WS.q().x();
  stateMsg.pose_ws.orientation.y = state.T_WS.q().y();
  stateMsg.pose_ws.orientation.z = state.T_WS.q().z();
  // Velocity in frame W [m/s].
  stateMsg.v_w.x = state.v_W[0];
  stateMsg.v_w.y = state.v_W[1];
  stateMsg.v_w.z = state.v_W[2];
  // Gyro bias [rad/s].
  stateMsg.b_g.x = state.b_g[0];
  stateMsg.b_g.y = state.b_g[1];
  stateMsg.b_g.z = state.b_g[2];
  // Accelerometer bias [m/s^2].
  stateMsg.b_a.x = state.b_a[0];
  stateMsg.b_a.y = state.b_a[1];
  stateMsg.b_a.z = state.b_a[2];
  // Rotational velocity in frame S [rad/s].
  stateMsg.omega_s.x = state.omega_S[0];
  stateMsg.omega_s.y = state.omega_S[1];
  stateMsg.omega_s.z = state.omega_S[2];
  // Frame Id.
  stateMsg.id = state.id.value();
  // IMU measurements up to this state's time.
  for(const auto& imu : state.previousImuMeasurements) {
    ::okvis_ros2_interfaces::msg::ImuCompact imuCompactMsg;
    imuCompactMsg.header.frame_id = "world";
    imuCompactMsg.header.stamp = rclcpp::Time(imu.timeStamp.sec, imu.timeStamp.nsec);
    imuCompactMsg.angular_velocity.x = imu.measurement.gyroscopes[0];
    imuCompactMsg.angular_velocity.y = imu.measurement.gyroscopes[1];
    imuCompactMsg.angular_velocity.z = imu.measurement.gyroscopes[2];
    imuCompactMsg.linear_acceleration.x = imu.measurement.accelerometers[0];
    imuCompactMsg.linear_acceleration.y = imu.measurement.accelerometers[1];
    imuCompactMsg.linear_acceleration.z = imu.measurement.accelerometers[2];
    stateMsg.previous_imu_measurements.push_back(imuCompactMsg);
  }
  stateMsg.is_keyframe = state.isKeyframe; // Is it a keyframe?
  for(const auto & id : state.covisibleFrameIds) {
    stateMsg.covisible_frame_ids.push_back(id.value()); // Covisible frame IDs.
  }
  stateMsg.state_changed = state.stateChanged; // Indicates if the state itself changed, or only other attributes (covis.).
  ///stateMsg.is_online_extrinsics = state.isOnlineExtrinsics; // Is the extrinsic online-calibrated?
  /*for(const auto& extrinsic : state.extrinsics) { // up-to-date cam-IMU extrinsics T_SCs if online calibration.
    geometry_msgs::msg::Pose pose;
    pose.position.x = extrinsic.second.r()[0];
    pose.position.y = extrinsic.second.r()[1];
    pose.position.z = extrinsic.second.r()[2];
    pose.orientation.w = extrinsic.second.q().w();
    pose.orientation.x = extrinsic.second.q().x();
    pose.orientation.y = extrinsic.second.q().y();
    pose.orientation.z = extrinsic.second.q().z();
    stateMsg.extrinsics.push_back(pose);
  }*/
  ///stateMsg.pose_gw.position.x = state.T_GW.r()[0];
  ///stateMsg.pose_gw.position.y = state.T_GW.r()[1];
  ///stateMsg.pose_gw.position.z = state.T_GW.r()[2];
  ///stateMsg.pose_gw.orientation.w = state.T_GW.q().w();
  ///stateMsg.pose_gw.orientation.x = state.T_GW.q().x();
  ///stateMsg.pose_gw.orientation.y = state.T_GW.q().y();
  ///stateMsg.pose_gw.orientation.z = state.T_GW.q().z();
}

void UpdatePublisher::publishEstimatorUpdate(
  const State& state, const TrackingState & trackingState,
  std::shared_ptr<const AlignedMap<StateId, State>> updatedStates,
  std::shared_ptr<const MapPointVector> landmarks) {

  // forward to existing writer
  trajectoryOutput_.processState(state, trackingState, updatedStates, landmarks);

  // update Trajectory object
  std::set<okvis::StateId> affectedStateIds;
  while(trajectoryLocked_);
  trajectoryLocked_ = true;
  trajectory_.update(trackingState, updatedStates, affectedStateIds);
  trajectoryLocked_ = false;

  //// assemble and publish the update message:
  // current state
  okvis_ros2_interfaces::msg::State stateMsg;
  cvtState2Msg(state, stateMsg);

  // tracking state
  okvis_ros2_interfaces::msg::TrackingState trackingStateMsg;
  trackingStateMsg.id = trackingState.id.value(); // ID this tracking info refers to.
  trackingStateMsg.is_keyframe = trackingState.isKeyframe; // Is it a keyframe?
  ///trackingStateMsg.is_lidar_keyframe = trackingState.isLidarKeyframe; // Is it a keyframe triggered by lidar?
  trackingStateMsg.tracking_quality = 
      (trackingState.trackingQuality == TrackingQuality::Good) ? 2 : 
      ((trackingState.trackingQuality == TrackingQuality::Lost) ? 0 : 1); // The tracking quality.
  trackingStateMsg.recognised_place = trackingState.recognisedPlace; // Has this fram recognised a place / relocalised / loop-closed?
  trackingStateMsg.is_full_graph_optimising = trackingState.isFullGraphOptimising; // Is the background loop closure optimisation currently ongoing?
  trackingStateMsg.current_keyframe_id = trackingState.currentKeyframeId.value(); // The ID of the current keyframe.

  // compose the overall message
  okvis_ros2_interfaces::msg::Update updateMsg;  // Update message.
  updateMsg.state = stateMsg;
  updateMsg.tracking_state = trackingStateMsg;
  
  // add all updated states
  for(const auto & updatedState : *updatedStates) {
    okvis_ros2_interfaces::msg::State updatedStateMsg;
    cvtState2Msg(updatedState.second, updatedStateMsg);
    updateMsg.updated_states.push_back(updatedStateMsg);
    updateMsg.updated_states_ids.push_back(updatedState.first.value());
  }
  ////

  // finally the landmarks
  pcl::PointCloud<pcl::PointXYZRGB> pointsMatched; // Point cloud for matched points.
  pointsMatched.reserve(landmarks->size());

  // transform points into custom world frame:
  /// \todo properly from ros params -- also landmark thresholds below
  for (const auto & lm : *landmarks) {
    // check infinity
    if (fabs((double) (lm.point[3])) < 1.0e-8)
      continue;

    // check quality
    if (lm.quality < 0.01)
      continue;

    pointsMatched.push_back(pcl::PointXYZRGB());
    const Eigen::Vector4d point = lm.point;
    pointsMatched.back().x = point[0] / point[3];
    pointsMatched.back().y = point[1] / point[3];
    pointsMatched.back().z = point[2] / point[3];
    pointsMatched.back().g = 255 * (std::min(0.1f, (float)lm.quality) / 0.1f);
  }
  pointsMatched.header.frame_id = "world";
  sensor_msgs::msg::PointCloud2 pointsMatchedMsg;
  pcl::toROSMsg(pointsMatched, pointsMatchedMsg);
  pointsMatchedMsg.header.frame_id = "world";

#if PCL_VERSION >= PCL_VERSION_CALC(1,7,0)
  std_msgs::msg::Header header;
  header.stamp = stateMsg.header.stamp;
  pointsMatched.header.stamp = pcl_conversions::toPCL(header).stamp;
#else
  pointsMatched.header.stamp = stateMsg.header.stamp;
#endif

  // and now publish them:
  pubPointsMatched_->publish(pointsMatchedMsg);

}

void UpdatePublisher::setupImageTopics(const okvis::cameras::NCameraSystem & nCameraSystem) {
  imagesTransport_.clear();
  pubImages_.clear();
  for(size_t i=0; i< nCameraSystem.numCameras(); ++i) {
    std::string name;
    if(nCameraSystem.cameraType(i).isColour) {
      name = "rgb"+std::to_string(i);
    } else {
      name = "cam"+std::to_string(i);
    }
    imagesTransport_[name] = std::shared_ptr<image_transport::ImageTransport>(
      new image_transport::ImageTransport(node_));
    pubImages_[name] = imagesTransport_.at(name)->advertise(name+"_matches", 1);
  }
  std::string name = "Top Debug View";
  imagesTransport_[name] = std::shared_ptr<image_transport::ImageTransport>(
    new image_transport::ImageTransport(node_));
  pubImages_[name] = imagesTransport_.at(name)->advertise("top_debug_view", 1);
}

bool UpdatePublisher::publishImages(const std::map<std::string, cv::Mat>& images) const {
  for(const auto & image : images) {
    sensor_msgs::msg::Image::SharedPtr msg
      = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", image.second).toImageMsg();
    const auto & pubIter = pubImages_.find(image.first);
    if(pubIter == pubImages_.end()) {
      LOG(WARNING) << image.first
                   << " topic not set up. Did you call Publisher::setupImageTopics (correctly)?";
    }
    pubIter->second.publish(msg);
  }
  return true;
}

}  // namespace okvis
