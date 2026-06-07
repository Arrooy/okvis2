
std_msgs/Header header # includes timestamp.
geometry_msgs/Pose pose_ws # Transformation between World W and Sensor S.
geometry_msgs/Vector3 v_w # Velocity in frame W [m/s].
geometry_msgs/Vector3 b_g # Gyro bias [rad/s].
geometry_msgs/Vector3 b_a # Accelerometer bias [m/s^2].
geometry_msgs/Vector3 omega_s # Rotational velocity in frame S [rad/s].
uint64 id # Frame Id.
ImuCompact[] previous_imu_measurements # IMU measurements up to this state's time.
bool is_keyframe # Is it a keyframe?
# TODO AlignedMap<uint64_t, kinematics::Transformation> T_AiW # Agent i to World pose (if available).
uint64[] covisible_frame_ids # Covisible frame IDs.
bool state_changed # Indicates if the state itself changed, or only other attributes (covis.).
bool is_online_extrinsics # Is the extrinsic online-calibrated?
geometry_msgs/Pose[] extrinsics # up-to-date cam-IMU extrinsics T_SCs if online calibration.
geometry_msgs/Pose pose_gw # The current estimate of the Extrinsics World to GPS