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
 * @file okvis_node_receiver.cpp
 * @brief This file includes the ROS node implementation -- subscribe to sensor topics.
 * @author Stefan Leutenegger
 */

#include <functional>
#include <iostream>
#include <fstream>
#include <iostream>
#include <signal.h>
#include <stdlib.h>

#include <glog/logging.h>

#include <okvis/ros2/Publisher.hpp>
#include "okvis_interfaces/msg/update.hpp"

#include <okvis/ThreadedSlam.hpp>
#include <okvis/DatasetWriter.hpp>
#include <okvis/ViParametersReader.hpp>

class UpdateSubscriber
{
  public:
    UpdateSubscriber(std::shared_ptr<rclcpp::Node> node, okvis::Publisher &publisher)
    {
      node_ = node;
      publisher_ = &publisher;
      subscription_ = node_->create_subscription<okvis_interfaces::msg::Update>(
      "/okvis/okvis_update", 10, std::bind(&UpdateSubscriber::updateCallback, this, std::placeholders::_1));
    }

  private:
    void stateMsg2State(const okvis_interfaces::msg::State &stateMsg, okvis::State &state) const
    {
      // state
      state.T_WS = okvis::kinematics::Transformation(
          Eigen::Vector3d(stateMsg.pose_ws.position.x, stateMsg.pose_ws.position.y, stateMsg.pose_ws.position.z),
          Eigen::Quaterniond(stateMsg.pose_ws.orientation.w, stateMsg.pose_ws.orientation.x, stateMsg.pose_ws.orientation.y, stateMsg.pose_ws.orientation.z)
      );
      state.v_W = Eigen::Vector3d(stateMsg.v_w.x, stateMsg.v_w.y, stateMsg.v_w.z);
      state.b_g = Eigen::Vector3d(stateMsg.b_g.x, stateMsg.b_g.y, stateMsg.b_g.z);
      state.b_a = Eigen::Vector3d(stateMsg.b_a.x, stateMsg.b_a.y, stateMsg.b_a.z);
      state.omega_S = Eigen::Vector3d(stateMsg.omega_s.x, stateMsg.omega_s.y, stateMsg.omega_s.z);
      state.timestamp = okvis::Time(stateMsg.header.stamp.sec, stateMsg.header.stamp.nanosec); // Timestamp corresponding to this state.
      state.id = okvis::StateId(stateMsg.id); // Frame Id.

      // IMU measurements up to this state's time.
      for(const auto& imu : stateMsg.previous_imu_measurements) {
        okvis::ImuMeasurement imuMeasurement;
        imuMeasurement.timeStamp = okvis::Time(imu.header.stamp.sec, imu.header.stamp.nanosec);
        imuMeasurement.measurement.gyroscopes = Eigen::Vector3d(imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z);
        imuMeasurement.measurement.accelerometers = Eigen::Vector3d(imu.linear_acceleration.x, imu.linear_acceleration.y, imu.linear_acceleration.z);
        state.previousImuMeasurements.push_back(imuMeasurement);
      }
      state.isKeyframe = stateMsg.is_keyframe; // Is it a keyframe?
      //state.T_AiW = okvis::AlignedMap<uint64_t, kinematics::Transformation>(); // Agent i to World pose (if available).
      // Covisible frame IDs.
      for(const auto & id : stateMsg.covisible_frame_ids) {
        state.covisibleFrameIds.insert(okvis::StateId(id));
      }
    }

    void updateCallback(const okvis_interfaces::msg::Update::SharedPtr msg) const
    {
      okvis::State state;
      stateMsg2State(msg->state, state);

      okvis::TrackingState trackingState;
      trackingState.id = okvis::StateId(msg->tracking_state.id); // ID this tracking info refers to.
      trackingState.isKeyframe = msg->tracking_state.is_keyframe; // Is it a keyframe?
      trackingState.trackingQuality = static_cast<okvis::TrackingQuality>(msg->tracking_state.tracking_quality); // The tracking quality.
      trackingState.recognisedPlace = msg->tracking_state.recognised_place; // Has this fram recognised a place / relocalised / loop-closed?
      trackingState.isFullGraphOptimising = msg->tracking_state.is_full_graph_optimising; // Is the background loop closure optimisation currently ongoing?
      trackingState.currentKeyframeId = okvis::StateId(msg->tracking_state.current_keyframe_id); // The ID of the current keyframe.

      std::shared_ptr<okvis::AlignedMap<okvis::StateId, okvis::State>> updatedStates 
      = std::make_shared<okvis::AlignedMap<okvis::StateId, okvis::State>>();
      for (const auto& state : msg->updated_states) {
        okvis::State s;
        stateMsg2State(state, s);
        (*updatedStates)[okvis::StateId(state.id)] = s;
      }

      std::shared_ptr<okvis::MapPointVector> landmarks = std::make_shared<okvis::MapPointVector>();
      for (const auto& landmark : msg->landmarks) {
        okvis::MapPoint mp;
        mp.id = landmark.id;
        mp.quality = landmark.quality;
        mp.point = Eigen::Vector4d(landmark.point.x, landmark.point.y, landmark.point.z, 1);
        landmarks->push_back(mp);
      }

      publisher_->publishEstimatorUpdate(state, trackingState, updatedStates, landmarks);
    }
    std::shared_ptr<rclcpp::Node> node_; ///< The node handle.
    okvis::Publisher *publisher_; ///< The publisher.
    rclcpp::Subscription<okvis_interfaces::msg::Update>::SharedPtr subscription_;
};

std::atomic_bool shtdown; ///< Shutdown requested?

/// \brief Main
/// \param argc argc.
/// \param argv argv.
int main(int argc, char **argv) {

  google::InitGoogleLogging(argv[0]);
  FLAGS_stderrthreshold = 0;  // INFO: 0, WARNING: 1, ERROR: 2, FATAL: 3
  FLAGS_colorlogtostderr = 1;

  // ros2 setup
  rclcpp::init(argc, argv);

  // set up the node
  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("okvis_node_receiver");
  
  // Setting up paramaters
  std::string configFilename("");
  node->declare_parameter("config_filename", "");
  node->get_parameter("config_filename", configFilename);
  if (configFilename.compare("")==0){
    LOG(ERROR) << "ros parameter 'config_filename' not set";
    return EXIT_FAILURE;
  }

  // publisher
  okvis::Publisher publisher(node);

  // construct OKVIS side
  okvis::ViParametersReader viParametersReader(configFilename);
  okvis::ViParameters parameters;
  viParametersReader.getParameters(parameters);

  // output publishing
  publisher.setBodyTransform(parameters.imu.T_BS);

  // subscriber
  UpdateSubscriber updateSubscriber(node, publisher);

  // require a special termination handler to properly close
  shtdown = false;
  signal(SIGINT, [](int) { 
    shtdown = true; 
  });
  
  // Main loop
  while (true) {
    rclcpp::spin_some(node);
    if(shtdown) {
      break;
    }
  }

  return EXIT_SUCCESS;
}
