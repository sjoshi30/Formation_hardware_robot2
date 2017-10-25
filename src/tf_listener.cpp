#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int64.h"
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <math.h>

// Global variables
double vd ;
double wd ;

// Subscriber for twist message from arduino
void twist_message_callback(geometry_msgs::Twist::ConstPtr& vel)
{
   vd = vel->linear.x ;
   wd = vel->angular.z ;
}

int main(int argc, char** argv)
{
   ros::init(argc, argv, "tf_listener");
   ros::NodeHandle node ;

   ros::Publisher odom_pub = node.advertise<nav_msgs::Odometry>("odom",50);
   ros::Subscriber odom_sub = node.subscribe("arduino_vel",10, twist_message_callback); 
 
   tf::TransformListener listener ;

   

   ros::Rate rate(10.0);
   
   while (node.ok())
   {
      tf::StampedTransform transform ;
      try 
      {
         listener.lookupTransform("base_link","odom",ros::Time(0),transform);
      }
      catch (tf::TransformException ex)
      {
         ROS_ERROR("%s",ex.what());
         ros::Duration(1.0).sleep();
      }

      ros::Time now = ros::Time::now() ;

      nav_msgs::Odometry odom ;
      odom.header.stamp = now ;
      odom.header.frame_id = "odom" ;
      odom.child_frame_id = "base_footprint" ;
      odom.pose.pose.position.x = transform.getOrigin().x() ;
      odom.pose.pose.position.y = transform.getOrigin().y() ;
      odom.pose.pose.position.z = 0.0 ;
      odom.pose.pose.orientation = 0 ; //transform.getrotation() ;
      odom.twist.twist.linear.x = vd ;
      odom.twist.twist.linear.y = 0 ;
      odom.twist.twist.angular.z = wd ;
      odom_pub.publish(odom);    
   }

   return 0 ;
}


