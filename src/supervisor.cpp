/** MRS Summer School 2020
 * UVDAR Leader - Follower Task
 * Supervisor node
 * @author Petr Štibinger <stibipet@fel.cvut.cz>
 */

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Int64.h>
#include <std_srvs/Trigger.h>
#include <mrs_msgs/ImagePointsWithFloatStamped.h>
#include <dynamic_reconfigure/server.h>

#include <eigen3/Eigen/Core>

#include <mrs_lib/attitude_converter.h>
#include <mrs_lib/param_loader.h>

#include <mrs_msgs/ReferenceStamped.h>
#include <mrs_msgs/SpeedTrackerCommand.h>
#include <mrs_msgs/String.h>
#include <mrs_msgs/PoseWithCovarianceArrayStamped.h>

#include <summer_school_supervisor/message_utils.h>
#include <uvdar_leader_follower/follower.h>
#include <uvdar_leader_follower/FollowerConfig.h>

#define MAX_ERRONEOUS_COMMANDS_COUNT 10

#define MIN_COMMAND_HEIGHT 1.0              // [m]
#define MAX_COMMAND_HEIGHT 15.0             // [m]
#define MAX_COMMAND_DISTANCE_THRESHOLD 8.0  // [m]
#define MAX_VELOCITY_MAGNITUDE 5.0          // [m/s]

bool initialized         = false;
bool using_speed_tracker = false;
bool contact_broken      = false;
bool counting_score      = false;

double visual_contact_timeout  = 1.7;   // [s]
double command_timeout         = 1.0;   // [s]
double score_timer_interval    = 0.01;  // [s]
double control_action_interval = 0.0;   // [s]

ros::Time last_contact_time;
ros::Time last_command_time;
int       score                    = 0;
int       left_blinkers            = 0;
int       right_blinkers           = 0;
int       erroneous_commands_count = 0;

Eigen::Vector3d leader_raw_pos;

ros::Subscriber odometry_subscriber;
ros::Subscriber uvdar_subscriber;
ros::Subscriber left_blinkers_subscriber;
ros::Subscriber right_blinkers_subscriber;
ros::Publisher  score_publisher;

ros::Publisher leader_raw_odom_publisher;
ros::Publisher leader_estim_odom_publisher;

ros::Timer score_timer, control_action_timer;

ros::ServiceClient switch_tracker_client;
ros::ServiceServer start_score_counting_server;
ros::Publisher     mpc_reference_publisher;
ros::Publisher     speed_tracker_command_publisher;

Eigen::Vector3d follower_position;

FollowerController fc;
std::string        uav_frame;

/* switchToSpeedTracker //{ */
void switchToSpeedTracker() {
  if (using_speed_tracker) {
    return;
  }
  mrs_msgs::String msg;
  msg.request.value = "SpeedTracker";
  switch_tracker_client.call(msg.request, msg.response);
  if (!msg.response.success) {
    ROS_WARN("[%s]: %s", ros::this_node::getName().c_str(), msg.response.message.c_str());
  } else {
    ROS_INFO("[%s]: Switched to SpeedTracker", ros::this_node::getName().c_str());
    using_speed_tracker = true;
  }
}
//}

/* switchToMpcTracker //{ */
void switchToMpcTracker() {
  if (!using_speed_tracker) {
    return;
  }
  mrs_msgs::String msg;
  msg.request.value = "MpcTracker";
  switch_tracker_client.call(msg.request, msg.response);
  if (!msg.response.success) {
    ROS_WARN("[%s]: %s", ros::this_node::getName().c_str(), msg.response.message.c_str());
  } else {
    ROS_INFO("[%s]: Switched to MPC tracker", ros::this_node::getName().c_str());
    using_speed_tracker = false;
  }
}
//}

/* publishLeaderRawOdom //{ */
void publishLeaderRawOdom() {
  nav_msgs::Odometry leader_raw_odom;
  leader_raw_odom.header.frame_id      = uav_frame;
  leader_raw_odom.header.stamp         = ros::Time::now();
  leader_raw_odom.pose.pose.position.x = leader_raw_pos.x();
  leader_raw_odom.pose.pose.position.y = leader_raw_pos.y();
  leader_raw_odom.pose.pose.position.z = leader_raw_pos.z();
  leader_raw_odom_publisher.publish(leader_raw_odom);
}
//}

/* publishLeaderFilteredOdom //{ */
void publishLeaderFilteredOdom() {
  auto leader_estim_odom            = fc.getCurrentEstimate();
  leader_estim_odom.header.frame_id = uav_frame;
  leader_estim_odom.header.stamp    = ros::Time::now();
  leader_estim_odom_publisher.publish(leader_estim_odom);
}
//}

/* scoreTimer //{ */
void scoreTimer(const ros::TimerEvent /* &event */) {

  if ((left_blinkers + right_blinkers) >= 2) {
    last_contact_time = ros::Time::now();
  }

  auto now = ros::Time::now().toSec();

  // check if visual contact is broken
  double visual_dt = now - last_contact_time.toSec();
  if (visual_dt > visual_contact_timeout) {
    ROS_ERROR("[%s]: Visual contact broken for longer than %.2f sec! Following terminated!", ros::this_node::getName().c_str(), visual_contact_timeout);
    contact_broken = true;
  }

  // check if control commands are comming in
  double command_dt = now - last_command_time.toSec();
  if (command_dt > command_timeout) {
    ROS_ERROR("[%s]: Valid control command was not provided for longer than %.2f sec! Following terminated!", ros::this_node::getName().c_str(),
              command_timeout);
  }

  if (erroneous_commands_count > MAX_ERRONEOUS_COMMANDS_COUNT) {
    ROS_ERROR("[%s]: Too many erroneous commands recieved! Following terminated!", ros::this_node::getName().c_str());
    contact_broken = true;
  }

  if (counting_score) {
    score++;
    std_msgs::Int64 score_msg;
    score_msg.data = score;
    score_publisher.publish(score_msg);
  }
}
//}

/* controlAction //{ */
void controlAction(const ros::TimerEvent /* &event */) {

  publishLeaderRawOdom();
  publishLeaderFilteredOdom();

  // call student's createSpeedCommand
  auto speed_command_request = fc.createSpeedCommand();
  auto speed_command         = buildSpeedTrackerCommand(speed_command_request.velocity, speed_command_request.heading, speed_command_request.height, uav_frame);
  speed_tracker_command_publisher.publish(speed_command);

  if (speed_command_request.use_for_control) {

    /* speed command sanity checks //{ */
    if (speed_command_request.height < MIN_COMMAND_HEIGHT) {
      ROS_WARN("[%s]: Reference point set too low! The command will be discarded", ros::this_node::getName().c_str());
      erroneous_commands_count++;
      return;
    }
    if (speed_command_request.height > MAX_COMMAND_HEIGHT) {
      ROS_WARN("[%s]: Reference point set too high! The command will be discarded", ros::this_node::getName().c_str());
      erroneous_commands_count++;
      return;
    }
    if (speed_command_request.velocity.norm() > MAX_VELOCITY_MAGNITUDE) {
      ROS_WARN("[%s]: Requested velocity is too high! The command will be discarded", ros::this_node::getName().c_str());
      erroneous_commands_count++;
      return;
    }
    //}
    erroneous_commands_count = 0;

    switchToSpeedTracker();
    last_command_time = speed_command.header.stamp;
  } else {
    switchToMpcTracker();
    // call student's createReferencePoint
    auto reference_point_request = fc.createReferencePoint();

    /* reference command sanity checks //{ */
    if (reference_point_request.position.z() < MIN_COMMAND_HEIGHT) {
      ROS_WARN("[%s]: Reference point set too low! The command will be discarded", ros::this_node::getName().c_str());
      erroneous_commands_count++;
      return;
    }

    if (reference_point_request.position.z() > MAX_COMMAND_HEIGHT) {
      ROS_WARN("[%s]: Reference point set too high! The command will be discarded", ros::this_node::getName().c_str());
      erroneous_commands_count++;
      return;
    }

    if ((reference_point_request.position - follower_position).norm() > MAX_COMMAND_DISTANCE_THRESHOLD) {
      ROS_WARN("[%s]: Reference point [%.2f, %.2f, %.2f] set too far from the UAV position [%.2f, %.2f, %.2f]. The command will be discarded",
               ros::this_node::getName().c_str(), reference_point_request.position.x(), reference_point_request.position.y(),
               reference_point_request.position.z(), follower_position.x(), follower_position.y(), follower_position.z());
      erroneous_commands_count++;
      return;
    }
    //} reference command sanity check

    auto reference_point = buildMpcReference(reference_point_request.position, reference_point_request.heading, uav_frame);
    mpc_reference_publisher.publish(reference_point);
    last_command_time = reference_point.header.stamp;
  }
}
//}

/* leftBlinkersCallback //{ */
void leftBlinkersCallback(const mrs_msgs::ImagePointsWithFloatStamped& msg) {
  left_blinkers = msg.points.size();
}
//}

/* rightBlinkersCallback //{ */
void rightBlinkersCallback(const mrs_msgs::ImagePointsWithFloatStamped& msg) {
  right_blinkers = msg.points.size();
}
//}

/* uvdarCallback //{ */
void uvdarCallback(const mrs_msgs::PoseWithCovarianceArrayStamped& uvdar_msg) {
  if (!initialized) {
    return;
  }
  if (uvdar_msg.poses.size() < 1) {
    return;
  }
  geometry_msgs::PoseWithCovarianceStamped msg;
  msg.header     = uvdar_msg.header;
  msg.pose.pose  = uvdar_msg.poses[0].pose;
  leader_raw_pos = Eigen::Vector3d(uvdar_msg.poses[0].pose.position.x, uvdar_msg.poses[0].pose.position.y, uvdar_msg.poses[0].pose.position.z);
  fc.receiveUvdar(msg);
}
//}

/* odometryCallback //{ */
void odometryCallback(const nav_msgs::Odometry& odometry_msg) {
  if (!initialized) {
    return;
  }
  follower_position = Eigen::Vector3d(odometry_msg.pose.pose.position.x, odometry_msg.pose.pose.position.y, odometry_msg.pose.pose.position.z);
  fc.receiveOdometry(odometry_msg);
}
//}

/* callbackStartScoreCounting //{ */
bool callbackStartScoreCounting([[maybe_unused]] std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
  if (counting_score) {
    res.success = false;
    res.message = "Already counting score!";
    return false;
  }
  counting_score = true;
  res.success    = true;
  return true;
}
//}

/* main //{ */
int main(int argc, char** argv) {

  ros::init(argc, argv, "summer_school_supervisor");
  ros::NodeHandle nh = ros::NodeHandle("~");

  std::string       reference_frame;
  std::stringstream ss;
  ss << std::getenv("UAV_NAME") << "/gps_origin";
  uav_frame = ss.str();

  odometry_subscriber       = nh.subscribe("odometry_in", 10, &odometryCallback);
  uvdar_subscriber          = nh.subscribe("uvdar_in", 10, &uvdarCallback);
  left_blinkers_subscriber  = nh.subscribe("left_blinkers_in", 10, &leftBlinkersCallback);
  right_blinkers_subscriber = nh.subscribe("right_blinkers_in", 10, &rightBlinkersCallback);
  score_publisher           = nh.advertise<std_msgs::Int64>("score_out", 1);

  mpc_reference_publisher         = nh.advertise<mrs_msgs::ReferenceStamped>("reference_point_out", 1);
  speed_tracker_command_publisher = nh.advertise<mrs_msgs::SpeedTrackerCommand>("speed_tracker_command_out", 1);
  switch_tracker_client           = nh.serviceClient<mrs_msgs::String>("switch_tracker_srv_out");
  start_score_counting_server     = nh.advertiseService("start_score_counting_in", &callbackStartScoreCounting);

  leader_raw_odom_publisher   = nh.advertise<nav_msgs::Odometry>("leader_raw_odom_out", 1);
  leader_estim_odom_publisher = nh.advertise<nav_msgs::Odometry>("leader_estim_odom_out", 1);

  dynamic_reconfigure::Server<uvdar_leader_follower::FollowerConfig>               dynamic_reconfigure_server;
  dynamic_reconfigure::Server<uvdar_leader_follower::FollowerConfig>::CallbackType dynamic_reconfigure_callback_t;

  ROS_INFO("[%s]: Waiting for position from UVDAR...", ros::this_node::getName().c_str());
  while (ros::ok()) {
    if (left_blinkers + right_blinkers > 1) {
      last_contact_time = ros::Time::now();
      break;
    }
    ros::Duration(0.01).sleep();
    ros::spinOnce();
  }
  mrs_lib::ParamLoader param_loader(nh, "follower");
  initialized    = true;
  auto dr_config = fc.initialize(param_loader);
  dynamic_reconfigure_server.updateConfig(dr_config);
  dynamic_reconfigure_callback_t = boost::bind(&FollowerController::dynamicReconfigureCallback, fc, _1, _2);
  dynamic_reconfigure_server.setCallback(dynamic_reconfigure_callback_t);
  ROS_INFO("[%s]: Initialization complete", ros::this_node::getName().c_str());

  control_action_interval = fc.getControlActionInterval();

  if (control_action_interval < 0.01) {
    ROS_WARN("[%s]: Cannot set shorter control interval than 0.01 seconds! Value will be truncated to 0.01 seconds.", ros::this_node::getName().c_str());
  }

  ROS_INFO("[%s]: Starting control action loop at %.3f Hz", ros::this_node::getName().c_str(), (1.0 / control_action_interval));
  last_command_time = ros::Time::now();

  score_timer          = nh.createTimer(ros::Duration(score_timer_interval), &scoreTimer);
  control_action_timer = nh.createTimer(ros::Duration(control_action_interval), &controlAction);

  while (ros::ok() && !contact_broken) {
    ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(0.01));
  }

  control_action_timer.stop();
  score_timer.stop();
  switchToMpcTracker();
  ROS_INFO("[%s]: Final score: %d", ros::this_node::getName().c_str(), score);

  return 0;
}
//}