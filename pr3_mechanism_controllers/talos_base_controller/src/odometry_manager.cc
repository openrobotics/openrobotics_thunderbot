#include "talos_base_controller/odometry_manager.h"

OdometryManager::OdometryManager() :
x_global_(0.0), x_current_(0.0), x_vel_(0.0),
y_global_(0.0), y_current_(0.0), y_vel_(0.0),
th_global_(0.0), th_current_(0.0), th_vel_(0.0),
wheel_width_(0.455), meters_per_tick_((6 * 2 * M_PI * 0.0525)/4680), previous_motor_ts_(0.0),
current_timestamp_(0.0)
{}

OdometryManager::~OdometryManager() {}

void OdometryManager::UpdateOdometry(ArduinoMessage message)
{
  /* update odometry state */
  double left_delta = this->meters_per_tick_ * message.l_feedback;
  double right_delta = this->meters_per_tick_ * message.r_feedback;
  double distance = ( left_delta + right_delta ) / 2;
  double theta_delta = ( left_delta  - right_delta ) / this->wheel_width_;
  double x_delta = distance * cosf(theta_delta + this->th_global_);
  double y_delta = distance * sinf(theta_delta + this->th_global_);

  this->x_global_  += x_delta;
  this->y_global_  += y_delta;

  this->th_global_ = fmod((theta_delta + th_global_),(2*M_PI));

  this->x_vel_ = x_delta / ToSeconds(message.time_stamp - this->previous_motor_ts_);
  this->y_vel_ = y_delta / ToSeconds(message.time_stamp - this->previous_motor_ts_);
  this->th_vel_ = theta_delta / ToSeconds(message.time_stamp - this->previous_motor_ts_);

  this->current_timestamp_ = CalcNewTime(message.time_stamp);

  this->previous_motor_ts_ = message.time_stamp;

}

/* presuppose: duration is in milliseconds */
double OdometryManager::ToSeconds(unsigned long duration) {
  return duration * (1/1000.0);
}

geometry_msgs::TransformStamped OdometryManager::GetCurrentTransform()
{
  geometry_msgs::TransformStamped odom_trans;
  odom_trans.header.stamp = this->current_timestamp_;
  odom_trans.header.frame_id = "odom";
  odom_trans.child_frame_id = "base_footprint";
  odom_trans.transform.translation.x = this->x_global_;
  odom_trans.transform.translation.y = this->y_global_;
  odom_trans.transform.translation.z = 0.0;
  odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(this->th_global_);

  return odom_trans;
}

ros::Time OdometryManager::CalcNewTime(const unsigned long& new_ts)
{ 
  uint32_t difference = (uint32_t)(new_ts - this->previous_motor_ts_);
  /* convert difference into nano seconds */
  difference = difference * 1000000;
  ros::Duration elapsed(0, difference);
  return (this->current_timestamp_ + elapsed);
}

nav_msgs::Odometry OdometryManager::GetCurrentOdom()
{
  nav_msgs::Odometry odom;
  odom.header.stamp = this->current_timestamp();
  odom.header.frame_id = "odom";
  odom.child_frame_id = "base_footprint";

  /* position */
  odom.pose.pose.position.x = this->x_global_;
  odom.pose.pose.position.y = this->y_global_;
  odom.pose.pose.position.z = 0.0;
  odom.pose.pose.orientation = 
    tf::createQuaternionMsgFromRollPitchYaw(0, 0, this->th_global_);

  /* velocity */
  odom.twist.twist.linear.x = this->x_vel_;
  odom.twist.twist.linear.y = this->y_vel_;
  odom.twist.twist.linear.z = 0.0;
  odom.twist.twist.angular.x = 0.0;
  odom.twist.twist.angular.y = 0.0;
  odom.twist.twist.angular.z = this->th_vel_;
  return odom;
}

