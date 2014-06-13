#ifndef ODOMETRY_MANAGER_H

#include <string>
#include <math.h>
#include <stdint.h>

#include "nav_msgs/Odometry.h"
#include "tf/transform_broadcaster.h"
#include "sensor_msgs/JointState.h"
#include "talos_base_controller/ArduinoMessage.h"
#include "ros/ros.h"

#define ODOMETRY_MANAGER_H
class OdometryManager
{
  public :
    OdometryManager();
    virtual void UpdateOdometry(ArduinoMessage message);
    virtual geometry_msgs::TransformStamped GetCurrentTransform();
    virtual nav_msgs::Odometry GetCurrentOdom();
    virtual double ToSeconds(unsigned long duration);
    virtual ~OdometryManager();

    /* getters */
    double x_global() const                  { return this->x_global_; }
    double x_vel() const                     { return this->x_vel_; }
    double y_global() const                  { return this->y_global_; }
    double y_vel() const                     { return this->y_vel_; }
    double th_global() const                 { return this->th_global_; }
    double th_vel() const                    { return this->th_vel_; }
    double wheel_width() const               { return this->wheel_width_; }
    double meters_per_tick() const           { return this->meters_per_tick_; }
    unsigned long previous_motor_ts() const { return this->previous_motor_ts_; }
    ros::Time current_timestamp() const      { return this->current_timestamp_; }

    /* setters */
    void x_global          (const double& x)         { this->x_global_ = x; }
    void x_current         (const double& x)         { this->x_current_ = x; }
    void y_global          (const double& y)         { this->y_global_ = y; }
    void y_current         (const double& y)         { this->y_current_ = y; }
    void th_global         (const double& th)        { this->th_global_ = th; }
    void th_current        (const double& th)        { this->th_current_ = th; }
    void wheel_width       (const double& ww)        { this->th_current_ = ww; }
    void meters_per_tick   (const double& mpt)       { this->meters_per_tick_ = mpt; }
    void previous_motor_ts(const unsigned long& ts) { this->previous_motor_ts_ = ts; }

  private :
    double x_global_;
    double x_current_;
    double x_vel_;
    double y_global_;
    double y_current_;
    double y_vel_;
    double th_global_;
    double th_current_;
    double th_vel_;
    double wheel_width_;
    double meters_per_tick_;
    unsigned long previous_motor_ts_;
    ros::Time current_timestamp_;
    /* Assumption: time elapsed since last observation is < 4 seconds */
    ros::Time CalcNewTime(const unsigned long& motor_ts);

};

#endif
