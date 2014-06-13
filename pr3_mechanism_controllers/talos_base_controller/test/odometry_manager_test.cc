#include <gtest/gtest.h>
#include "talos_base_controller/odometry_manager.h"

class OdometryManagerTest : public ::testing::Test 
{
  protected:
    OdometryManager odom_manager;
};

TEST_F (OdometryManagerTest, TestDefaultConstructor) {
  EXPECT_EQ (odom_manager.x_global(), 0.0);
  EXPECT_EQ (odom_manager.y_global(), 0.0);
  EXPECT_EQ (odom_manager.th_global(), 0.0);
  EXPECT_EQ (odom_manager.previous_motor_ts(), 0.0);
}

TEST_F (OdometryManagerTest, TestGetCurrentTransform) {
  ArduinoMessage message;
  message.time_stamp = 100;
  message.l_feedback = 300;
  message.r_feedback = 300;

  const double expected_left_delta = odom_manager.meters_per_tick() * message.l_feedback;
  const double expected_right_delta = odom_manager.meters_per_tick() * message.r_feedback;
  const double expected_distance = ( expected_left_delta + expected_right_delta ) / 2;
  const double expected_theta_delta = ( expected_left_delta  - expected_right_delta ) / odom_manager.wheel_width();

  const double expected_x_delta = expected_distance * cosf(expected_theta_delta + odom_manager.th_global());
  const double expected_y_delta = expected_distance * sinf(expected_theta_delta + odom_manager.th_global());

  const double expected_x = expected_x_delta + odom_manager.x_global();

  const double expected_y = expected_y_delta + odom_manager.y_global(); 
  const double expected_th = expected_theta_delta + odom_manager.th_global();

  const double expected_z = 0.0;
  const geometry_msgs::Quaternion expected_rotation = tf::createQuaternionMsgFromYaw(odom_manager.th_global());

  odom_manager.UpdateOdometry(message);
  geometry_msgs::TransformStamped actual = odom_manager.GetCurrentTransform();

  /* test headers */
  EXPECT_EQ ( actual.header.stamp, odom_manager.current_timestamp() );
  EXPECT_EQ ( actual.header.frame_id, "odom" );
  EXPECT_EQ ( actual.child_frame_id, "base_footprint" );
  /* translation */
  EXPECT_EQ ( actual.transform.translation.x , expected_x );
  EXPECT_EQ ( actual.transform.translation.y , expected_y );
  EXPECT_EQ ( actual.transform.translation.z , expected_z );
  /* test quaternion */
  EXPECT_EQ ( actual.transform.rotation.x, expected_rotation.x );
  EXPECT_EQ ( actual.transform.rotation.y, expected_rotation.y );
  EXPECT_EQ ( actual.transform.rotation.z, expected_rotation.z );
  EXPECT_EQ ( actual.transform.rotation.w, expected_rotation.w );
}

TEST_F (OdometryManagerTest, TestGetCurrentOdom) {

  ArduinoMessage message;
  message.time_stamp = 100;
  message.l_feedback = 300;
  message.r_feedback = 300;

  const double expected_left_delta = odom_manager.meters_per_tick() * message.l_feedback;
  const double expected_right_delta = odom_manager.meters_per_tick() * message.r_feedback;
  const double expected_distance = ( expected_left_delta + expected_right_delta ) / 2;
  const double expected_theta_delta = ( expected_left_delta  - expected_right_delta ) / odom_manager.wheel_width();

  const double expected_x_delta = expected_distance * cosf(expected_theta_delta + odom_manager.th_global());
  const double expected_y_delta = expected_distance * sinf(expected_theta_delta + odom_manager.th_global());

  const double expected_x = expected_x_delta + odom_manager.x_global();

  const double expected_y = expected_y_delta + odom_manager.y_global(); 
  const double expected_th = expected_theta_delta + odom_manager.th_global();

  const double expected_z = 0.0;
  const geometry_msgs::Quaternion expected_rotation = tf::createQuaternionMsgFromYaw(odom_manager.th_global());

  // ToSeconds(delta_t)
  const double expected_x_vel = expected_x_delta / odom_manager.ToSeconds(message.time_stamp - odom_manager.previous_motor_ts()); 
  const double expected_y_vel = expected_y_delta / odom_manager.ToSeconds(message.time_stamp - odom_manager.previous_motor_ts());
  const double expected_th_vel = expected_theta_delta / odom_manager.ToSeconds(message.time_stamp - odom_manager.previous_motor_ts());
  const double expected_z_vel = 0.0;
  const double expected_angular_x_vel = 0.0;
  const double expected_angular_y_vel = 0.0;

  /* run methods */
  odom_manager.UpdateOdometry(message);
  nav_msgs::Odometry actual = odom_manager.GetCurrentOdom();

  /* test headers */
  EXPECT_EQ ( actual.header.stamp, odom_manager.current_timestamp() );
  EXPECT_EQ ( actual.header.frame_id, "odom" );
  EXPECT_EQ ( actual.child_frame_id, "base_footprint" );

  /* test positions */
  EXPECT_EQ ( actual.pose.pose.position.x , expected_x );
  EXPECT_EQ ( actual.pose.pose.position.y , expected_y );
  EXPECT_EQ ( actual.pose.pose.position.z , expected_z );
  EXPECT_EQ ( actual.pose.pose.orientation.x, expected_rotation.x );
  EXPECT_EQ ( actual.pose.pose.orientation.y, expected_rotation.y );
  EXPECT_EQ ( actual.pose.pose.orientation.z, expected_rotation.z );
  EXPECT_EQ ( actual.pose.pose.orientation.w, expected_rotation.w );

  /* test velocities */
  EXPECT_EQ ( actual.twist.twist.linear.x, expected_x_vel );
  EXPECT_EQ ( actual.twist.twist.linear.y, expected_y_vel );
  EXPECT_EQ ( actual.twist.twist.linear.z, expected_z_vel );
  EXPECT_EQ ( actual.twist.twist.angular.x, expected_angular_x_vel );
  EXPECT_EQ ( actual.twist.twist.angular.y, expected_angular_y_vel );
  EXPECT_EQ ( actual.twist.twist.angular.z, expected_th_vel );

}

TEST_F (OdometryManagerTest, TestToSecond){
  unsigned long t1 = 0.0;
  unsigned long t2 = 167;
  const double ToSec = odom_manager.ToSeconds(t2 - t1);
  EXPECT_EQ (odom_manager.ToSeconds(t2 - t1), (t2-t1)*(1.0/1000));
}

TEST_F (OdometryManagerTest, TestForward) {
  ArduinoMessage message;
  message.time_stamp = 167;
  message.l_feedback = 300;
  message.r_feedback = 300;

  const double expected_left_delta = odom_manager.meters_per_tick() * message.l_feedback;
  const double expected_right_delta = odom_manager.meters_per_tick() * message.r_feedback;
  const double expected_distance = ( expected_left_delta + expected_right_delta ) / 2;
  const double expected_theta_delta = ( expected_left_delta  - expected_right_delta ) / odom_manager.wheel_width();

  const double expected_x_delta = expected_distance * cosf(expected_theta_delta + odom_manager.th_global());
  const double expected_y_delta = expected_distance * sinf(expected_theta_delta + odom_manager.th_global());

  const double expected_x = expected_x_delta + odom_manager.x_global();
  const double expected_y = expected_y_delta + odom_manager.y_global(); 
  const double expected_th = expected_theta_delta + odom_manager.th_global();

  // ToSeconds(delta_t)
  const double expected_x_vel = expected_x_delta / odom_manager.ToSeconds(message.time_stamp - odom_manager.previous_motor_ts()); 
  const double expected_y_vel = expected_y_delta / odom_manager.ToSeconds(message.time_stamp - odom_manager.previous_motor_ts());
  const double expected_th_vel = expected_theta_delta / odom_manager.ToSeconds(message.time_stamp - odom_manager.previous_motor_ts());

  odom_manager.UpdateOdometry(message);

  EXPECT_EQ ( odom_manager.x_global(), expected_x );
  EXPECT_EQ ( odom_manager.y_global(), expected_y );
  EXPECT_EQ ( odom_manager.th_global(), expected_th );
  EXPECT_EQ ( odom_manager.x_vel(), expected_x_vel );
  EXPECT_EQ ( odom_manager.y_vel(), expected_y_vel );
  EXPECT_EQ ( odom_manager.th_vel(), expected_th_vel );
}

TEST_F (OdometryManagerTest, TestForwardLeft) {

  ArduinoMessage message;
  message.time_stamp = 200;
  message.l_feedback = 300;
  message.r_feedback = 400;

  const double expected_left_delta = odom_manager.meters_per_tick() * message.l_feedback;
  const double expected_right_delta = odom_manager.meters_per_tick() * message.r_feedback;
  const double expected_distance = ( expected_left_delta + expected_right_delta ) / 2;
  const double expected_theta_delta = ( expected_left_delta  - expected_right_delta ) / odom_manager.wheel_width();

  const double expected_x_delta = expected_distance * cosf(expected_theta_delta + odom_manager.th_global());
  const double expected_y_delta = expected_distance * sinf(expected_theta_delta + odom_manager.th_global());

  const double expected_x = expected_x_delta + odom_manager.x_global();
  const double expected_y = expected_y_delta + odom_manager.y_global(); 
  const double expected_th = expected_theta_delta + odom_manager.th_global();

  // ToSeconds(delta_t)
  const double expected_x_vel = expected_x_delta / odom_manager.ToSeconds(message.time_stamp - odom_manager.previous_motor_ts()); 
  const double expected_y_vel = expected_y_delta / odom_manager.ToSeconds(message.time_stamp - odom_manager.previous_motor_ts());
  const double expected_th_vel = expected_theta_delta / odom_manager.ToSeconds(message.time_stamp - odom_manager.previous_motor_ts());

  odom_manager.UpdateOdometry(message);

  EXPECT_EQ ( odom_manager.x_global(), expected_x );
  EXPECT_EQ ( odom_manager.y_global(), expected_y );
  EXPECT_EQ ( odom_manager.th_global(), expected_th );
  EXPECT_EQ ( odom_manager.x_vel(), expected_x_vel );
  EXPECT_EQ ( odom_manager.y_vel(), expected_y_vel );
  EXPECT_EQ ( odom_manager.th_vel(), expected_th_vel );
}

TEST_F (OdometryManagerTest, TestRotateRightLeft) {

  ArduinoMessage message;
  message.time_stamp = 180;
  message.l_feedback = 300;
  message.r_feedback = -300;
  const double expected_left_delta = odom_manager.meters_per_tick() * message.l_feedback;
  const double expected_right_delta = odom_manager.meters_per_tick() * message.r_feedback;
  const double expected_distance = ( expected_left_delta + expected_right_delta ) / 2;
  const double expected_theta_delta = ( expected_left_delta  - expected_right_delta ) / odom_manager.wheel_width();

  const double expected_x_delta = expected_distance * cosf(expected_theta_delta + odom_manager.th_global());
  const double expected_y_delta = expected_distance * sinf(expected_theta_delta + odom_manager.th_global());

  const double expected_x = expected_x_delta + odom_manager.x_global();
  const double expected_y = expected_y_delta + odom_manager.y_global(); 
  const double expected_th = expected_theta_delta + odom_manager.th_global();

  const unsigned long expected_previous_motor_ts = 180;
  const ros::Time expected_current_ts(0, 180 * 1000000);

  // ToSeconds(delta_t)
  const double expected_x_vel = expected_x_delta / odom_manager.ToSeconds(message.time_stamp - odom_manager.previous_motor_ts()); 
  const double expected_y_vel = expected_y_delta / odom_manager.ToSeconds(message.time_stamp - odom_manager.previous_motor_ts());
  const double expected_th_vel = expected_theta_delta / odom_manager.ToSeconds(message.time_stamp - odom_manager.previous_motor_ts());

  odom_manager.UpdateOdometry(message);
  EXPECT_EQ ( odom_manager.x_global(), expected_x );
  EXPECT_EQ ( odom_manager.y_global(), expected_y );
  EXPECT_EQ ( odom_manager.th_global(), expected_th );
  EXPECT_EQ ( odom_manager.x_vel(), expected_x_vel );
  EXPECT_EQ ( odom_manager.y_vel(), expected_y_vel );
  EXPECT_EQ ( odom_manager.th_vel(), expected_th_vel );
  EXPECT_EQ ( odom_manager.previous_motor_ts(), expected_previous_motor_ts );
  EXPECT_EQ ( odom_manager.current_timestamp(), expected_current_ts );

  ArduinoMessage message2;
  message2.time_stamp = 344;
  message2.l_feedback = -120;
  message2.r_feedback = 120;

  const double expected_left_delta2 = odom_manager.meters_per_tick() * message2.l_feedback;
  const double expected_right_delta2 = odom_manager.meters_per_tick() * message2.r_feedback;
  const double expected_distance2 = ( expected_left_delta2 + expected_right_delta2 ) / 2;
  const double expected_theta_delta2 = ( expected_left_delta2  - expected_right_delta2 ) / odom_manager.wheel_width();

  const double expected_x2_delta = expected_distance2 * cosf(expected_theta_delta2 + odom_manager.th_global());
  const double expected_y2_delta = expected_distance2 * sinf(expected_theta_delta2 + odom_manager.th_global());

  const double expected_x2 = expected_x2_delta + odom_manager.x_global();
  const double expected_y2 = expected_y2_delta + odom_manager.y_global();
  const double expected_th2 = expected_theta_delta2 + odom_manager.th_global();

  const unsigned long expected_previous_motor_ts2 = 344;
  const ros::Time expected_current_ts2(0, 344 * 1000000);

  // ToSeconds(delta_t)
  const double expected_x_vel2 = expected_x2_delta / odom_manager.ToSeconds(message2.time_stamp - odom_manager.previous_motor_ts()); 
  const double expected_y_vel2 = expected_y2_delta / odom_manager.ToSeconds(message2.time_stamp - odom_manager.previous_motor_ts());
  const double expected_th_vel2 = expected_theta_delta2 / odom_manager.ToSeconds(message2.time_stamp - odom_manager.previous_motor_ts());

  odom_manager.UpdateOdometry(message2);

  EXPECT_EQ ( odom_manager.x_global(), expected_x2 );
  EXPECT_EQ ( odom_manager.y_global(), expected_y2 );
  EXPECT_EQ ( odom_manager.th_global(), expected_th2 );
  EXPECT_EQ ( odom_manager.x_vel(), expected_x_vel2 );
  EXPECT_EQ ( odom_manager.y_vel(), expected_y_vel2 );
  EXPECT_EQ ( odom_manager.th_vel(), expected_th_vel2 );
  EXPECT_EQ ( odom_manager.previous_motor_ts(), expected_previous_motor_ts2 );
  EXPECT_EQ ( odom_manager.current_timestamp(), expected_current_ts2 );
}


TEST_F (OdometryManagerTest, TestFromNonZeroStart) {

  ArduinoMessage message;
  message.time_stamp = 133;
  message.r_feedback = +70;
  message.l_feedback = +121;
  
  odom_manager.x_global(5.4);
  odom_manager.y_global(-3.2);
  odom_manager.th_global(-2*M_PI+.01) ;

  const double expected_left_delta = odom_manager.meters_per_tick() * message.l_feedback;
  const double expected_right_delta = odom_manager.meters_per_tick() * message.r_feedback;
  const double expected_distance = ( expected_left_delta + expected_right_delta ) / 2;
  const double expected_theta_delta = ( expected_left_delta  - expected_right_delta ) / odom_manager.wheel_width();

  const double expected_x_delta = expected_distance * cosf(expected_theta_delta + odom_manager.th_global());
  const double expected_y_delta = expected_distance * sinf(expected_theta_delta + odom_manager.th_global());


  const double expected_x = expected_x_delta + odom_manager.x_global();
  const double expected_y = expected_y_delta + odom_manager.y_global();
  const double expected_th = fmod((expected_theta_delta + odom_manager.th_global()),(2*M_PI));

  // ToSeconds(delta_t)
  const double expected_x_vel = expected_x_delta / odom_manager.ToSeconds(message.time_stamp - odom_manager.previous_motor_ts()); 
  const double expected_y_vel = expected_y_delta / odom_manager.ToSeconds(message.time_stamp - odom_manager.previous_motor_ts());
  const double expected_th_vel = expected_theta_delta / odom_manager.ToSeconds(message.time_stamp - odom_manager.previous_motor_ts());

  odom_manager.UpdateOdometry(message);

  EXPECT_EQ ( odom_manager.x_global(), expected_x );
  EXPECT_EQ ( odom_manager.y_global(), expected_y );
  EXPECT_EQ ( odom_manager.th_global(), expected_th );
  EXPECT_EQ ( odom_manager.x_vel(), expected_x_vel );
  EXPECT_EQ ( odom_manager.y_vel(), expected_y_vel );
  EXPECT_EQ ( odom_manager.th_vel(), expected_th_vel ); 
}
TEST_F (OdometryManagerTest, TestBackRight) {
  ArduinoMessage message;
  message.time_stamp = 133;
  message.l_feedback = -150;
  message.r_feedback = -89;

  const double expected_left_delta = odom_manager.meters_per_tick() * message.l_feedback;
  const double expected_right_delta = odom_manager.meters_per_tick() * message.r_feedback;
  const double expected_distance = ( expected_left_delta + expected_right_delta ) / 2;
  const double expected_theta_delta = ( expected_left_delta  - expected_right_delta ) / odom_manager.wheel_width();

  const double expected_x_delta = expected_distance * cosf(expected_theta_delta + odom_manager.th_global());
  const double expected_y_delta = expected_distance * sinf(expected_theta_delta + odom_manager.th_global());


  const double expected_x = expected_x_delta + odom_manager.x_global();
  const double expected_y = expected_y_delta + odom_manager.y_global();
  const double expected_th = expected_theta_delta + odom_manager.th_global();

  // ToSeconds(delta_t)
  const double expected_x_vel = expected_x_delta / odom_manager.ToSeconds(message.time_stamp - odom_manager.previous_motor_ts()); 
  const double expected_y_vel = expected_y_delta / odom_manager.ToSeconds(message.time_stamp - odom_manager.previous_motor_ts());
  const double expected_th_vel = expected_theta_delta / odom_manager.ToSeconds(message.time_stamp - odom_manager.previous_motor_ts());

  odom_manager.UpdateOdometry(message);

  EXPECT_EQ ( odom_manager.x_global(), expected_x );
  EXPECT_EQ ( odom_manager.y_global(), expected_y );
  EXPECT_EQ ( odom_manager.th_global(), expected_th );
  EXPECT_EQ ( odom_manager.x_vel(), expected_x_vel );
  EXPECT_EQ ( odom_manager.y_vel(), expected_y_vel );
  EXPECT_EQ ( odom_manager.th_vel(), expected_th_vel ); 
}

TEST_F (OdometryManagerTest, TestUpdateTime) {
  ArduinoMessage message;
  message.time_stamp = 1000;
  message.l_feedback = 0;
  message.r_feedback = 0;

  const double expected_x = 0.0;
  const double expected_y = 0.0;
  const double expected_th = 0.0;
  const double expected_x_vel = 0.0;
  const double expected_y_vel = 0.0;
  const double expected_th_vel = 0.0;
  const double expected_previous_motor_ts = 1000;
  const ros::Time expected_current_ts(1);

  odom_manager.UpdateOdometry(message);

  EXPECT_EQ ( odom_manager.x_global(), expected_x );
  EXPECT_EQ ( odom_manager.y_global(), expected_y );
  EXPECT_EQ ( odom_manager.th_global(), expected_th );
  EXPECT_EQ ( odom_manager.x_vel(), expected_x_vel );
  EXPECT_EQ ( odom_manager.y_vel(), expected_y_vel );
  EXPECT_EQ ( odom_manager.th_vel(), expected_th_vel ); 

  EXPECT_EQ ( odom_manager.previous_motor_ts(), expected_previous_motor_ts );
  EXPECT_EQ ( odom_manager.current_timestamp(), expected_current_ts );
}

int main (int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
