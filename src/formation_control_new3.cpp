#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include "std_msgs/String.h"
#include <actionlib/client/simple_action_client.h>
#include <algorithm>

#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "geometry_msgs/QuaternionStamped.h"
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "tf/LinearMath/Transform.h"
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <string>
#include "collvoid_msgs/PoseTwistWithCovariance.h"

#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/MapMetaData.h"
#include "std_msgs/Header.h"
#include "nav_msgs/MapMetaData.h"

using namespace std ;

/*
#include "IPlanarGridMap.h"
#include "PlanarGridBinaryMap.h"
#include "PlanarGridContainer.h"
#include "PlanarGridIndex.h"
#include "PlanarGridOccupancyMap.h"private: 
*/

#define MAX_LINEAR_VEL 0.5
#define MAX_ANGULAR_VEL 0.5 

class formation_control
{
public:
	formation_control()    ;
	void spin()            ;
	int signum(double val) ;
private: 
	ros::NodeHandle nh ;
    ros::Publisher cmd_vel_pub ;
    ros::Publisher debug_pub ;
    ros::Subscriber position_share_sub ;
	void formation_callback(const collvoid_msgs::PoseTwistWithCovariance::ConstPtr& data) ;
	void init_variables()  ;
	void update()          ; 

	geometry_msgs::Twist debug_msg      ;
	geometry_msgs::Twist vel_msg        ; 

	double SWARM_DIST ;
	double SWARM_ANG ;
	double FORMATION_ANG ;

	double robot0_x ;
    double robot0_y ;
    double robot0_xdot  ;
    double robot0_ydot  ;
    double robot0_xdot_prev ;
    double robot0_ydot_prev ;
    double robot0_xdotdot ;
    double robot0_ydotdot ;
    double robot0_yaw ;
    double robot0_vx  ;
    double robot0_omega ;

    double robot2_x ;
    double robot2_y ;
    double robot2_xdot ;
    double robot2_ydot ;
    double robot2_xdotdot ;
    double robot2_ydotdot ;
    double robot2_yaw ;
    double robot2_vx ;
    double robot2_wx ;

    double error_dist_prev ;
    double error_dist ;
    double error_dist_rate ;
    double error_yaw ;
    double error_yaw_rate ;

    double goal_x ;
    double goal_y ;
    double goal_yaw ;

    double g ;
    double omega_n ;
    double k1,k2,k3 ;
    double e1,e2,e3 ;
    double vt1,vt2 ;

    double k_s ;
    double k_theta ;
    double k_orientation ; 
    double e_s ;
    double e_theta ;
    double e_orientation ;
    double theta ;
    double delta_l ;

    double rate ;
    ros::Time t_next ;
    ros::Duration t_delta ;
    double elapsed ;
    ros::Time then;
};

formation_control::formation_control()
{
	init_variables();

    // Publish
    cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel",20);
    debug_pub   = nh.advertise<geometry_msgs::Twist>("debug",20);

    // Subscribe
    position_share_sub = nh.subscribe("position_share", 50, &formation_control::formation_callback, this);
}

void formation_control::init_variables()
{
	rate = 20 ;

	SWARM_DIST = 1 ;
	SWARM_ANG = 0.785 ;
	FORMATION_ANG = 0 ;

	robot0_x = 0 ;
    robot0_y = 0 ;
    robot0_xdot = 0 ;
    robot0_ydot = 0 ;
    robot0_xdot_prev = 0 ;
    robot0_ydot_prev = 0 ;
    robot0_xdotdot = 0;
    robot0_ydotdot = 0;
    robot0_yaw = 0;
    robot0_vx  = 0.05;
    robot0_omega = 0 ; // angular velocity curvature

    robot2_x = 0 ;
    robot2_y = 0 ;
    robot2_xdot = 0 ;
    robot2_ydot = 0 ;
    robot2_xdotdot = 0 ;
    robot2_ydotdot = 0 ;
    robot2_yaw = 0 ;
    robot2_vx = 0 ;
    robot2_wx = 0 ;

    error_dist_prev = 0;
    error_dist = 0 ;
    error_dist_rate = 0 ;
    error_yaw = 0 ;
    error_yaw_rate = 0 ;

    goal_x = 0 ;
    goal_y = 0 ;
    goal_yaw = 0 ;

    g = 5 ;
    omega_n = 0 ;
    k1 = 1 ;
    k2 = 1 ;
    k3 = 0.5 ;
    e1 = 0 ;
    e2 = 0 ;
    e3 = 0 ;
    vt1 = 0 ;
    vt2 = 0 ;

    k_s = 1 ;
    k_theta = 1 ;
    k_orientation = 1 ;
    e_s = 0 ;
    e_theta = 0 ;
    e_orientation = 0;
    theta = 0 ;

    t_delta = ros::Duration(1.0 / rate) ;
    t_next = ros::Time::now() + t_delta ; 
    then = ros::Time::now();
}

int formation_control::signum(double val)
{
    if (val > 0.0) return  1 ;
    if (val < 0.0) return -1 ;
    return 0;
}

void formation_control::formation_callback(const collvoid_msgs::PoseTwistWithCovariance::ConstPtr& data)
{
	if (data->robot_id == "robot_0")
	{
		robot0_x  = data->pose.pose.position.x ;
        robot0_y  = data->pose.pose.position.y ;
        tf::Quaternion q_robot0(data->pose.pose.orientation.x, data->pose.pose.orientation.y, data->pose.pose.orientation.z, data->pose.pose.orientation.w);
        tf::Matrix3x3 m_robot0(q_robot0);
        double roll, pitch, yaw;
        m_robot0.getRPY(roll, pitch, yaw);
        robot0_yaw = yaw ;
        robot0_vx = data->twist.twist.linear.x ;
        //robot0_wx = data->twist.twist.angular.z ;
        robot0_xdot = robot0_vx*cos(robot0_yaw) ;
        robot0_ydot = robot0_vx*sin(robot0_yaw) ;
	}

	if (data->robot_id == "robot_2")
	{
		robot2_x = data->pose.pose.position.x ;
        robot2_y = data->pose.pose.position.y ;
        tf::Quaternion q_robot2(data->pose.pose.orientation.x, data->pose.pose.orientation.y, data->pose.pose.orientation.z, data->pose.pose.orientation.w);
        tf::Matrix3x3 m_robot2(q_robot2);
        double roll, pitch, yaw;
        m_robot2.getRPY(roll, pitch, yaw);
        robot2_yaw = yaw ;
        robot2_vx = data->twist.twist.linear.x ;
        //robot1_wx = data->twist.twist.angular.z ;
        robot2_xdot = robot2_vx*cos(robot2_yaw) ;
        robot2_ydot = robot2_vx*sin(robot2_yaw) ;
	}
}

void formation_control::update()
{
	ros::Time now = ros::Time::now();

	if ( now > t_next) 
	{
		elapsed = now.toSec() - then.toSec(); 
		//ROS_INFO_STREAM("elapsed =" << elapsed);
		//ROS_INFO_STREAM("now =" << now.toSec());
		//ROS_INFO_STREAM("then =" << then.toSec());

		FORMATION_ANG = SWARM_ANG - robot0_yaw ;

		if (FORMATION_ANG >= 2*M_PI)
			FORMATION_ANG = FORMATION_ANG - 2*M_PI ;
		if (FORMATION_ANG <= 2*M_PI)
			FORMATION_ANG = FORMATION_ANG + 2*M_PI ;

	    goal_x = robot0_x + SWARM_DIST*cos(FORMATION_ANG) ;
        goal_y = robot0_y + SWARM_DIST*sin(FORMATION_ANG) ;

        robot0_xdotdot = (robot0_xdot - robot0_xdot_prev)/elapsed ;
        robot0_ydotdot = (robot0_ydot - robot0_ydot_prev)/elapsed ;

        if (abs(robot0_vx) >= 0.05) 
        {
        	robot0_omega   = (robot0_xdot*robot0_ydotdot - robot0_xdotdot*robot0_ydot)/pow(robot0_vx,2) ;

            omega_n           =  sqrt(pow(robot0_omega,2) + g*pow(robot0_vx,2)) ;

            //k1                =  k1_gain*omega_n ;
            //k2                =  k2_gain*omega_n ;
            //k3                =  g*abs(robot0_vx);

            e1                =  cos(robot2_yaw)*(goal_x-robot2_x) + sin(robot2_yaw)*(goal_y-robot2_y) ; // ok
            e2                = -sin(robot2_yaw)*(goal_x-robot2_x) + cos(robot2_yaw)*(goal_y-robot2_y) ; // ok
            e3                =  robot0_yaw - robot2_yaw ; //very small

            vt1               = -k1*e1 ;
            vt2               = -k2*signum(robot0_vx)*e2 - k3*e3 ;

            robot0_xdot_prev = robot0_xdot      ;
            robot0_ydot_prev = robot0_ydot      ;
            
            vel_msg.linear.x  = robot0_vx*cos(e3) - vt1 ;
            vel_msg.angular.z = robot0_omega - vt2      ; 
            cmd_vel_pub.publish(vel_msg) ;
            
            debug_msg.linear.x =  robot0_yaw                ;
            debug_msg.linear.y =  robot2_yaw               ;
            debug_msg.linear.z =  robot0_omega                      ;
            debug_msg.angular.x = robot2_y                              ;
            debug_msg.angular.y = goal_x                             ;
            debug_msg.angular.z = goal_y                               ;
            debug_pub.publish(debug_msg)                          ;
        }
        else 
        {
        	delta_l = sqrt(pow(goal_y-robot2_y,2) + pow(goal_x-robot2_x,2))   ;
        	theta = atan2(goal_y-robot2_y,goal_x-robot2_x)                    ;
        	e_theta = theta - robot2_yaw ;
        	e_orientation = robot0_yaw - robot2_yaw ;
        	e_s = delta_l*cos(e_theta) ;
        	vel_msg.linear.x  = k_s*e_s ;
        	if (delta_l > 0.5)
        	{
        		vel_msg.angular.z = k_theta*e_theta ;
        	}
        	else
        	{
        		vel_msg.angular.z = k_orientation*e_orientation ;
        	}
            cmd_vel_pub.publish(vel_msg) ;
        }
        

        then = now;

        ros::spinOnce();
	}
	else 
	{
		ROS_INFO_STREAM("LOOP MISSED");
	} 
    
}

void formation_control::spin()
{
    ros::Rate loop_rate(rate);
    while (ros::ok())
	{
	   update();
	   loop_rate.sleep();
	}
}  
  

int main(int argc, char **argv)
{ 
  //Initiate ROS
  ros::init(argc, argv, "formation_control");

  // Create an object of class formation_control that will take care of everything
  formation_control formation_one ;

  formation_one.spin() ;

  return 0 ;
}
