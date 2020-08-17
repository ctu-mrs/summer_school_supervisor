
/** MRS Summer School 2020
 * UVDAR Leader - Follower Task
 * @author Petr Štibinger <stibipet@fel.cvut.cz>
 */

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <mrs_msgs/TrajectoryReference.h>
#include <mrs_msgs/SpeedTrackerCommand.h>
#include <mrs_msgs/ReferenceStamped.h>
#include <mrs_msgs/String.h>
#include <mrs_msgs/PoseWithCovarianceArrayStamped.h>
#include <dynamic_reconfigure/server.h>

#include <eigen3/Eigen/Core>

#include <mrs_lib/attitude_converter.h>
#include <mrs_lib/param_loader.h>

#include <uvdar_leader_follower/FollowerConfig.h>
#include <uvdar_leader_follower/velocity_estimator.h>
#include <uvdar_leader_follower/message_utils.h>


// macros for indexing the state vector
#define POS_X 0
#define POS_Y 1
#define POS_Z 2
#define VEL_X 3
#define VEL_Y 4
#define VEL_Z 5

typedef VelocityEstimator::kalman3D::x_t Vector6;  // x, y, z, dx, dy, dz

// params to be loaded from file config/follower.yaml
std::string uav_frame;                // global coordinate frame id
double      uvdar_msg_interval;       // time interval between uvdar messages
double      control_action_interval;  // time interval between control actions

// dynamically reconfigurable parameters
Eigen::Vector3d desired_offset;             // desired position offset from the leader position
double          heading_offset;             // desired heading offset
bool            use_speed_tracker = false;  // use the SpeedTracker instead of MPC tracker
bool            use_trajectory    = false;  // use multiple points as the reference for the MPC tracker
bool            filter_data       = false;  // use a Kalman filter to reduce leader position noise and predict its velocity
bool            discard_outliers  = false;  // discard measurements which are very far from the previous leader position
double          outlier_radius    = 2.5;    // max allowed distance between the estimated leader position and a new measurement

// state of the follower UAV
double          uav_heading;           // orientation of the UAV forward(X) axis with respect to the global frame
Eigen::Vector3d uav_position;          // position of the UAV in the global frame
Eigen::Vector3d uav_rpy;               // roll, pitch, yaw angles with respect to the global frame
Eigen::Vector3d uav_linear_velocity;   // translational motion vector
Eigen::Vector3d uav_angular_velocity;  // rotational motion vector

// state of the leader UAV
Eigen::Vector3d leader_position;  // position of the leader UAV in the global frame
Eigen::Vector3d leader_velocity;  // translational motion vector

// subscribers
ros::Subscriber uvdar_subscriber;     // information about the leader from UVDAR
ros::Subscriber odometry_subscriber;  // follower odometry

// publishers
ros::Publisher mpc_reference_publisher;           // set reference point for the MPC tracker
ros::Publisher mpc_trajectory_publisher;          // set reference trajectory for the MPC tracker
ros::Publisher speed_tracker_command_publisher;   // set command for the speed tracker
ros::Publisher leader_state_estimator_publisher;  // debug publisher for logging estimator output

// service clients
ros::ServiceClient switch_tracker_client;  // switch between MPC and Speed tracker

// helper subsystems
ros::Timer        control_action_timer;                    // keeps running the main control loop
VelocityEstimator velocity_estimator;                      // estimates velocity of the leader from position measurements
bool              velocity_estimator_initialized = false;  // switches on automatically
bool              got_odometry                   = false;  // flag to check if odometry callback is called

/* controlActionRoutine //{ */
/**
 * @brief YOUR CODE GOES HERE
 * @param event - unused in the default implementation
 */
void controlActionRoutine(const ros::TimerEvent /* &event */) {

  auto            leader_prediction = velocity_estimator.predict(Eigen::Vector3d(0, 0, 0), control_action_interval);
  Eigen::Vector3d leader_position_filtered(leader_prediction[POS_X], leader_prediction[POS_Y], leader_prediction[POS_Z]);
  Eigen::Vector3d leader_velocity(leader_prediction[VEL_X], leader_prediction[VEL_Y], 0.0);

  Eigen::Vector3d desired_position;
  if (filter_data) {
    desired_position = leader_position_filtered + desired_offset;
  } else {
    desired_position = leader_position + desired_offset;
  }

  double desired_heading = 0.0 + heading_offset;
  auto   reference_msg   = buildMpcReference(desired_position, desired_heading, uav_frame);
  mpc_reference_publisher.publish(reference_msg);

  auto leader_state_odom                = buildOdometryMessage(leader_position_filtered, leader_velocity, uav_frame);
  leader_state_odom.pose.covariance[0]  = velocity_estimator.getCovariance()(0, 0);
  leader_state_odom.pose.covariance[4]  = velocity_estimator.getCovariance()(1, 1);
  leader_state_odom.pose.covariance[8]  = velocity_estimator.getCovariance()(2, 2);
  leader_state_odom.pose.covariance[12] = velocity_estimator.getCovariance()(3, 3);
  leader_state_odom.pose.covariance[16] = velocity_estimator.getCovariance()(4, 4);
  leader_state_estimator_publisher.publish(leader_state_odom);

  auto speed_tracker_cmd = buildSpeedTrackerCommand(leader_velocity, desired_heading, desired_position.z(), uav_frame);
  speed_tracker_command_publisher.publish(speed_tracker_cmd);
}
//}

/* uvdarCallback //{ */
/**
 * @brief Get position of the leader from the UVDAR system
 *
 * @param uvdar_msg relative poses of other drones seen by onboard UV cameras (poses are given in the reference frame of this drone)
 */
void uvdarCallback(const mrs_msgs::PoseWithCovarianceArrayStamped& uvdar_msg) {
  // measurement sanity checks
  if (uvdar_msg.poses.size() < 1) {
    ROS_WARN("[%s]: No other vehicle detected by the UVDAR system!", ros::this_node::getName().c_str());
    return;
  }
  Eigen::Vector3d measured_position(uvdar_msg.poses[0].pose.position.x, uvdar_msg.poses[0].pose.position.y, uvdar_msg.poses[0].pose.position.z);
  if (discard_outliers && (leader_position - measured_position).norm() > outlier_radius) {
    ROS_WARN("[%s]: Outlier measurement discarded!", ros::this_node::getName().c_str());
    return;
  }
  leader_position.x() = uvdar_msg.poses[0].pose.position.x;
  leader_position.y() = uvdar_msg.poses[0].pose.position.y;
  leader_position.z() = uvdar_msg.poses[0].pose.position.z;

  velocity_estimator.fuse(leader_position);
}
//}

/* odometryCallback //{ */
/**
 * @brief Get information from odometry message and update our knowledge of the follower UAV state
 *
 * @param odometry_msg data from follower odometry node
 */
void odometryCallback(const nav_msgs::Odometry& odometry_msg) {
  uav_position.x() = odometry_msg.pose.pose.position.x;
  uav_position.y() = odometry_msg.pose.pose.position.y;
  uav_position.z() = odometry_msg.pose.pose.position.z;

  mrs_lib::AttitudeConverter ac(odometry_msg.pose.pose.orientation);
  uav_rpy[0] = ac.getRoll();
  uav_rpy[1] = ac.getPitch();
  uav_rpy[2] = ac.getYaw();

  uav_heading = ac.getHeading();

  uav_linear_velocity.x() = odometry_msg.twist.twist.linear.x;
  uav_linear_velocity.y() = odometry_msg.twist.twist.linear.y;
  uav_linear_velocity.z() = odometry_msg.twist.twist.linear.z;

  uav_angular_velocity.x() = odometry_msg.twist.twist.angular.x;
  uav_angular_velocity.y() = odometry_msg.twist.twist.angular.y;
  uav_angular_velocity.z() = odometry_msg.twist.twist.angular.z;
  if (!got_odometry) {
    got_odometry = true;
  }
}
//}

/* switchToSpeedTracker //{ */
/**
 * @brief Switch from MPC tracker to SpeedTracker
 * Use the SpeedTracker to access higher level of control in the form of velocity or acceleration
 *
 * Note that speed tracker commands need to be published before the tracker can be activated
 * YOU SHOULD TO CONSTRUCT AND PUBLISH THE COMMAND IN THE controlActionRoutine ON YOUR OWN
 *
 * @return
 */
bool switchToSpeedTracker() {
  mrs_msgs::String msg;
  msg.request.value = "SpeedTracker";
  switch_tracker_client.call(msg.request, msg.response);
  if (!msg.response.success) {
    ROS_WARN("[%s]: %s", ros::this_node::getName().c_str(), msg.response.message.c_str());
  } else {
    ROS_INFO("[%s]: Switched to SpeedTracker", ros::this_node::getName().c_str());
  }
  return msg.response.success;
}
//}

/* switchToMpcTracker //{ */
/**
 * @brief Switch from SpeedTracker to MPC tracker
 * Use the MPC tracker to set a reference position or reference trajectory to the UAV and let the control pipeline handle the rest
 *
 * @return
 */
bool switchToMpcTracker() {
  mrs_msgs::String msg;
  msg.request.value = "MpcTracker";
  switch_tracker_client.call(msg.request, msg.response);
  if (!msg.response.success) {
    ROS_WARN("[%s]: %s", ros::this_node::getName().c_str(), msg.response.message.c_str());
  } else {
    ROS_INFO("[%s]: Switched to MPC tracker", ros::this_node::getName().c_str());
  }
  return msg.response.success;
}
//}

/* dynamicReconfigureCallback //{ */
/**
 * @brief Callback for the dynamic reconfigure server. Use this to change parameters of the system online.
 * To start the dynamic reconfigure GUI, use the following command:
 *
 * rosrun rqt_reconfigure rqt_reconfigure
 *
 * @param config - Struct containing new parameter values
 * @param level - Unused
 */
void dynamicReconfigureCallback(uvdar_leader_follower::FollowerConfig& config, [[maybe_unused]] uint32_t level) {
  desired_offset.x() = config.desired_offset_x;
  desired_offset.y() = config.desired_offset_y;
  desired_offset.z() = config.desired_offset_z;
  heading_offset     = config.heading_offset;
  filter_data        = config.filter_data;
  discard_outliers   = config.discard_outliers;
  outlier_radius     = config.outlier_radius;
  if (!use_speed_tracker && config.use_speed_tracker) {
    bool speed_tracker_on = switchToSpeedTracker();
    use_speed_tracker     = speed_tracker_on;
  }
  if (use_speed_tracker && !config.use_speed_tracker) {
    switchToMpcTracker();
    use_speed_tracker = config.use_speed_tracker;
  }
}
//}

int main(int argc, char** argv) {

  //| ------------------ intialize ROS interface ----------------------- |//
  ros::init(argc, argv, "uvdar_follower");
  ros::NodeHandle      nh = ros::NodeHandle("~");
  mrs_lib::ParamLoader param_loader(nh, "uvdar_follower");
  //| ------------------------------------------------------------------ |//

  //| ------ load params from config file (config/follower.yaml) ------- |//
  std::string                 reference_frame;
  Eigen::Matrix<double, 6, 6> Q;
  Eigen::Matrix<double, 3, 3> R;
  param_loader.loadMatrixStatic("Q", Q);
  param_loader.loadMatrixStatic("R", R);
  param_loader.loadParam("reference_frame", reference_frame);
  param_loader.loadParam("control_action_interval", control_action_interval);
  param_loader.loadParam("uvdar_msg_interval", uvdar_msg_interval);
  param_loader.loadParam("desired_offset/x", desired_offset.x());
  param_loader.loadParam("desired_offset/y", desired_offset.y());
  param_loader.loadParam("desired_offset/z", desired_offset.z());
  std::stringstream ss;
  ss << std::getenv("UAV_NAME") << "/" << reference_frame;
  uav_frame = ss.str();
  //| ------------------------------------------------------------------ |//

  //| -------------------- initialize communication -------------------- |//
  odometry_subscriber = nh.subscribe("odometry_in", 10, &odometryCallback);
  /* uvdar_subscriber                 = nh.subscribe("fake_uvdar_in", 10, &FAKEuvdarCallback); */
  uvdar_subscriber                 = nh.subscribe("uvdar_in", 10, &uvdarCallback);
  mpc_reference_publisher          = nh.advertise<mrs_msgs::ReferenceStamped>("reference_point_out", 1);
  mpc_trajectory_publisher         = nh.advertise<mrs_msgs::TrajectoryReference>("reference_trajectory_out", 1);
  speed_tracker_command_publisher  = nh.advertise<mrs_msgs::SpeedTrackerCommand>("speed_tracker_command_out", 1);
  leader_state_estimator_publisher = nh.advertise<nav_msgs::Odometry>("estimate_out", 1);
  switch_tracker_client            = nh.serviceClient<mrs_msgs::String>("switch_tracker_srv_out");
  //| ------------------------------------------------------------------ |//


  //| ----------------- initialize dynamic reconfigure ----------------- |//
  //| ---------- using default values from config/follower.yaml ---------|//
  dynamic_reconfigure::Server<uvdar_leader_follower::FollowerConfig>               dynamic_reconfigure_server;
  dynamic_reconfigure::Server<uvdar_leader_follower::FollowerConfig>::CallbackType dynamic_reconfigure_callback_t;
  uvdar_leader_follower::FollowerConfig                                            dr_config;
  dr_config.desired_offset_x  = desired_offset.x();
  dr_config.desired_offset_y  = desired_offset.y();
  dr_config.desired_offset_z  = desired_offset.z();
  dr_config.filter_data       = filter_data;
  dr_config.discard_outliers  = discard_outliers;
  dr_config.outlier_radius    = outlier_radius;
  dr_config.use_speed_tracker = use_speed_tracker;
  dynamic_reconfigure_server.updateConfig(dr_config);
  dynamic_reconfigure_callback_t = boost::bind(&dynamicReconfigureCallback, _1, _2);
  dynamic_reconfigure_server.setCallback(dynamic_reconfigure_callback_t);
  //| ------------------------------------------------------------------ |//

  ROS_INFO("[%s]: Waiting for odometry data...", ros::this_node::getName().c_str());
  while (ros::ok()) {
    if (got_odometry) {
      break;
    }
    ros::spinOnce();
    ros::Duration(0.02).sleep();
  }

  //| ----- initialize Kalman filter for leader velocity estimation ---- |//
  Vector6 initial_states;
  initial_states << uav_position.x() - desired_offset.x(), uav_position.y() - desired_offset.y(), uav_position.z() - desired_offset.z(), 0, 0, 0;
  velocity_estimator             = VelocityEstimator(Q, R, initial_states, uvdar_msg_interval);
  velocity_estimator_initialized = true;
  //| ------------------------------------------------------------------ |//

  ROS_INFO("[%s]: Node initialized", ros::this_node::getName().c_str());

  //| ----------- continuously run the controlActionRoutine ------------ |//
  control_action_timer = nh.createTimer(ros::Duration(control_action_interval), &controlActionRoutine);
  ros::spin();
  //| ------------------------------------------------------------------ |//

  return 0;
}
