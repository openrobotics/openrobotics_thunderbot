#include <signal.h>

#include <boost/thread.hpp>

/* ROS libraries */
#include "ros/ros.h"
#include "ros/xmlrpc_manager.h"
#include "ros/console.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/String.h"
#include "tf/transform_broadcaster.h"

#include "BufferedAsyncSerial.h"
#include "talos_base_controller/ArduinoMessage.h"
#include "talos_base_controller/parser.h"
#include "talos_base_controller/odometry_manager.h"

namespace base_controller {
    BufferedAsyncSerial * async_serial;
    bool kill;

    /* signal-safe flag for whether shutdown is requested */
    sig_atomic_t volatile g_request_shutdown = 0;
}

/* Replacement SIGINT handler */
void mySigIntHandler(int sig)
{
    base_controller::g_request_shutdown = 1;
    ROS_WARN( "Received SIGINT, shutting down" );
}

/* Replacement "shutdown" XMLRPC callback */
void shutdown_callback (XmlRpc::XmlRpcValue& params, XmlRpc::XmlRpcValue& result)
{
    int num_params = 0;
    if ( params.getType() == XmlRpc::XmlRpcValue::TypeArray )
        num_params = params.size();
    if ( num_params > 1 )
    {
        std::string reason = params[1];
        ROS_WARN("Shutdown request received. Reason: %s]", reason.c_str());
        base_controller::g_request_shutdown = 1; // Set flag
    }
    ROS_WARN("custom shutdown functionality goes here");

    result = ros::xmlrpc::responseInt(1, "", 0);
}

void killCallBack(const std_msgs::String::ConstPtr& msg) {
  ROS_INFO ("KILL KILL KILL KILL KILL KILL KILL: %s", msg->data.c_str());
  if (msg->data.compare("kill") == 0) {
    base_controller::kill = true;
    ROS_INFO( "activating  software kill" );
  } else if (msg->data.compare("run") == 0) {
    base_controller::kill = false;
    ROS_INFO( "deactivating activating software kill" );
  } else {
    /* do nothing */
  }
}

void cmdCallBack(const geometry_msgs::Twist::ConstPtr& msg) {

    // If the time of the message compared to the now time is too great
    // Then dispose of the message, only process recent commands to give
    // The user a realistic feedback and control
 

    if ( ros::ok() && !base_controller::kill ) {
        std::string linear_x = boost::lexical_cast<std::string>(msg->linear.x);
        std::string linear_y = boost::lexical_cast<std::string>(msg->linear.y);
        std::string angular_z = boost::lexical_cast<std::string>(msg->angular.z);

        std::string usb_msg = linear_x + ", " + linear_y + ", " + angular_z + "\n";

        ROS_INFO( "sending: %s", usb_msg.c_str() );

        try {
          base_controller::async_serial->writeString( usb_msg );
        } catch ( boost::system::system_error& e ) {
          ROS_ERROR( "Error: %s", e.what() );
        }
    } else if ( base_controller::kill ) {
      std::string usb_kill_msg = "0,0,0\n";
      ROS_INFO( "sending kill message to motor controller, state of var: %d", base_controller::kill );
      try {
        base_controller::async_serial->writeString( usb_kill_msg );
      } catch ( boost::system::system_error& e ) {
        ROS_ERROR( "Error: %s", e.what() );
      }
    }
}

int main( int argc, char **argv ) {
    ROS_INFO( "initializing base_controller" );
    //ros::init( argc, argv, "talos_base_controller" );
    ros::init( argc, argv, "base_controller", ros::init_options::NoSigintHandler );
    ros::NodeHandle n;

    /* Override SIGINT handler */
    signal( SIGINT, mySigIntHandler );

    /* Override XMLRPC shutdown */
    ros::XMLRPCManager::instance()->unbind( "shutdown" );
    ros::XMLRPCManager::instance()->bind( "shutdown", shutdown_callback );

    std::string cmd_vel_topic;
    n.param<std::string>( "cmd_vel_topic", cmd_vel_topic, "cmd_vel" );
    /* create publisher and subscribers */
    ROS_INFO( "subscribing to cmd_vel" );
    ros::Subscriber sub = n.subscribe(cmd_vel_topic, 1, cmdCallBack );
    ROS_INFO( "subscribing to kill topic" );
    ros::Subscriber sub_kill = n.subscribe("kill", 1, killCallBack );
    ROS_INFO( "publishing to odom" );
    ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 10);
    ROS_INFO( "publishing tfs" );
    tf::TransformBroadcaster broadcaster;
    ros::Rate loop_rate(20);

    /* adds odometry manager and parser */
    Parser parser;
    OdometryManager odom_manager;

    base_controller::kill = false;

    try {

        ROS_INFO( "initializing asynchronous serial communication" );
        std::string device_file;
        n.param<std::string>("device_file", device_file, "/dev/ttyACM0");
        ROS_INFO( "device_file: %s", device_file.c_str() );

        base_controller::async_serial = new BufferedAsyncSerial( device_file , 9600 );
        ROS_INFO( "reading serial comm buffer" );

	

         	
        //while ( ros::ok() ) {
        while ( !(base_controller::g_request_shutdown) && n.ok() ) {
         std::string readValue;
            readValue = base_controller::async_serial->readStringUntil("\n");
            
            if (readValue != "") {
                ROS_INFO( "received message: %s", readValue.c_str() );
                odom_manager.UpdateOdometry(parser.parse( readValue.c_str() ));
                odom_pub.publish( odom_manager.GetCurrentOdom() );
                broadcaster.sendTransform( odom_manager.GetCurrentTransform() );
            }

             ros::spinOnce();
             loop_rate.sleep();  

        }
        
    } catch ( boost::system::system_error& e ) {
        ROS_ERROR ( "Error: %s", e.what() );
        ROS_ERROR ( "closing serial connection" );
        base_controller::async_serial->close();
        delete base_controller::async_serial;
        ros::shutdown();
    }    

    ROS_INFO ( "closing serial connection" );
    base_controller::async_serial->close();
    delete base_controller::async_serial;
    ros::shutdown();
}
