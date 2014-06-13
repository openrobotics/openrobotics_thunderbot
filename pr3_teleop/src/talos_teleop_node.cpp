#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "geometry_msgs/Twist.h"

class TeleopTalos
{
  public:
  TeleopTalos();

  private:
  bool killCommandDetected(const sensor_msgs::Joy::ConstPtr& joy);
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
  void killAllNodes(void);

  ros::NodeHandle nh_;

  int linear_, angular_, LB, RB;
  double l_scale_, a_scale_;
  ros::Publisher vel_pub_;
  ros::Subscriber joy_sub_;
};

TeleopTalos::TeleopTalos():
  linear_(1),
  angular_(2),
  LB(10),
  RB(11)
{
  nh_.param("axis_linear", linear_, linear_);
  nh_.param("axis_angular", angular_, angular_);
  nh_.param("scale_linear", l_scale_, l_scale_);
  nh_.param("scale_angular", a_scale_, a_scale_);

  vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 100);
  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &TeleopTalos::joyCallback, this);
}

// Author: Devon Ash
bool TeleopTalos::killCommandDetected(const sensor_msgs::Joy::ConstPtr& joy)
{
  int lb = joy->buttons[LB];
  int rb = joy->buttons[RB];

  return (lb && rb);  
}

// Author: Devon Ash
void TeleopTalos::killAllNodes()
{

   // Stop motors
   geometry_msgs::Twist twist;
   twist.linear.x = 0;
   twist.angular.z = 0;

   int THRESHOLD = 10;
   
   // Send lots of 0s for "safety"
   for (int i = 0; i < THRESHOLD; i++)
   {
       vel_pub_.publish(twist);
   }

   // code that brings down talos nodes. 
   std::vector<std::string> node_list;

   // Kills all
   system("rosnode kill -a");

   // Check what is alive   
   ros::master::getNodes(node_list);

   for (int i = 0; i < node_list.size(); i++)
   {
	ROS_WARN("Node %s was not killed. Not sure why.", node_list[i].c_str());
   }

}


void TeleopTalos::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
    // 0 = 2: left stick f/b
    // 1: left stick l/r
    // 3: right stick f/b
  geometry_msgs::Twist twist;
  twist.linear.x = l_scale_*joy->axes[linear_];
  twist.angular.z = a_scale_*joy->axes[angular_];

  // Process kill code if received both LB and RB as true
  if (killCommandDetected(joy))
  {
     ROS_WARN("KILL Command Detected. Initializing shutdown of vital nodes");
     killAllNodes();
  }

  vel_pub_.publish(twist);
}





int main(int argc, char** argv)
{
  ros::init(argc, argv, "teleop_talos");
  TeleopTalos teleop_talos;

  ros::spin();
}
