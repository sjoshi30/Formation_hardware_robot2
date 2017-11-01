#include "Eigen/Dense"
#include <vector>
#include <iostream>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int64.h"
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <nav_msgs/Odometry.h>
#include <math.h>
#include <apriltags_ros/AprilTagDetectionArray.h>
#include <apriltags_ros/AprilTagDetection.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

class outerloop_calc
{

public:
	outerloop_calc();
	void spin();
        
private:

	ros::NodeHandle n ;

        // Subscriber of apriltag poses
        ros::Subscriber apriltag_pose_sub ;

        // Subscriber of odom
        ros::Subscriber odom_sub ;

        // Publisher of velocityh
        ros::Publisher cmd_vel_pub ;

        // Publisher debug messages
        ros::Publisher debug_pub ;

        double id, x, y, z, theta, roll, pitch, yaw ;

        double kp_v ;
        double kd_v ;
        double ki_v ;
        double kp_w ; 
        double kd_w ;
        double ki_w ;

        // Prefilter
        double h ;
        double z_p ;
        double pref_out ;
        double pref_out_p ;
   
        // Rolloff
        double alpha ;
        
        double vd_ctrl, vd_ctrl_p, vd_ctrl_pp ;
        double wd_ctrl, wd_ctrl_p, wd_ctrl_pp ;
        double dist_error, dist_error_p, dist_error_pp ;
        double omega_error, omega_error_p, omega_error_pp ;

        double x_odom, y_odom, theta_odom, vd_odom,wd_odom;

        double rate, td ;
        ros::Duration t_delta;
	ros::Time t_next;
	ros::Time then;
	ros::Time current_time, last_time;

        // Subscriber callback odom
        void odom_pose_callback(const nav_msgs::Odometry::ConstPtr& data);

	void init_variables();
	void update();
};

outerloop_calc::outerloop_calc()
{
	init_variables();

	ROS_INFO("Started odometry computing node");

        // Publish in /cmd_vel
        cmd_vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 50) ; 

        // Publish in /debug
        debug_pub   = n.advertise<std_msgs::String>("/debug",10) ;

        // subscribe to odom topic
        odom_sub = n.subscribe("/odom",10, &outerloop_calc::odom_pose_callback,this);
}

void outerloop_calc::init_variables()
{
        x = 0 ; y = 0 ; z = 0 ; theta = 0; id = 0;
        x_odom = 0 ; y_odom = 0; theta_odom = 0 ; vd_odom =0; wd_odom =0;

        if (!n.getParam("kp_v", kp_v)) 
           kp_v = 0 ;
        if (!n.getParam("ki_v", ki_v))
           ki_v = 0 ;
        if (!n.getParam("kd_v", kd_v))
           kd_v = 0 ;
        if (!n.getParam("kp_w", kp_w)) 
           kp_v = 0 ;
        if (!n.getParam("ki_w", ki_w))
           ki_v = 0 ;
        if (!n.getParam("kd_w", kd_w))
           kd_v = 0 ;
        
        roll = 0; pitch = 0 ; yaw = 0;
 
        vd_ctrl = 0;vd_ctrl_p = 0 ; vd_ctrl_pp = 0 ;
        wd_ctrl = 0;wd_ctrl_p = 0 ; wd_ctrl_pp = 0 ;

        dist_error = 0; dist_error_p  = 0 ; dist_error_pp  = 0 ;
        omega_error = 0;omega_error_p = 0 ; omega_error_pp = 0 ;

	rate = 10 ; 
        td   = 1/rate ;
        alpha = 600 ;

	t_delta = ros::Duration(1.0 / rate);
	t_next  = ros::Time::now() + t_delta;
	then    = ros::Time::now();

	current_time = ros::Time::now();
  	last_time    = ros::Time::now();
}

//Spin function
void outerloop_calc::spin()
{
     ros::Rate loop_rate(rate);

     // delay 
     //ros::Rate delay(1) ;
     //delay.sleep(); 

     while (ros::ok())
	{
		update();
		loop_rate.sleep();
	}    
}

//Update function
void outerloop_calc::update()
{
	ros::Time now = ros::Time::now();
	
        // ros::Time elapsed;

	double elapsed;

        double dtheta ;
        double dxy ;
        double dx ;
        double dy ;  

	if ( now > t_next) 
        {
	    elapsed = now.toSec() - then.toSec();

            //pref_out = ( (td*h)*0.3 + (td*h)*0.3 - (td*h - 2)*pref_out_p )/(2 + td*h);
            //z_p = z ;
            //pref_out_p = pref_out ;

            dist_error = 1 - x_odom ;
            omega_error = 0 - y_odom ;

            // PD (normal controller)
            vd_ctrl = kp_v*dist_error + kd_v*(dist_error - dist_error_p)/td    ;
            wd_ctrl = kp_w*omega_error + kd_w*(omega_error - omega_error_p)/td ;

            geometry_msgs::Twist cmd_data ;
            cmd_data.linear.x = vd_ctrl ; // "-ve" to make control signal positive
            cmd_data.linear.y = 0 ;
            cmd_data.linear.z = 0 ;
            cmd_data.angular.x = 0   ;      // roll
            cmd_data.angular.y = 0   ;     // pitch
            cmd_data.angular.z = wd_ctrl   ;     // yaw
            cmd_vel_pub.publish(cmd_data);

            vd_ctrl_pp = vd_ctrl_p ; 
            vd_ctrl_p  = vd_ctrl ;

            wd_ctrl_pp = wd_ctrl_p ; 
            wd_ctrl_p  = wd_ctrl ;

            dist_error_pp = dist_error_p ;
            dist_error_p = dist_error ;
                
            omega_error_pp = omega_error_p ; 
            omega_error_p = omega_error ;
                
            then = now;
            ros::spinOnce();
        }
}
            
void outerloop_calc::odom_pose_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
  x_odom = msg->pose.pose.position.x ;
  y_odom = msg->pose.pose.position.y ;
  vd_odom = msg->twist.twist.linear.x ;
  wd_odom = msg->twist.twist.angular.z ;
}
  
int main(int argc, char **argv)
{
	ros::init(argc, argv,"outerloop2");
	outerloop_calc obj;
	obj.spin();
	return 0;
}
