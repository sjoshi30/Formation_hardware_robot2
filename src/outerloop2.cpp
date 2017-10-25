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

        double rate, td ;
        ros::Duration t_delta;
	ros::Time t_next;
	ros::Time then;
	ros::Time current_time, last_time;

        // Subscriber callback 
        void apriltag_pose_callack(const apriltags_ros::AprilTagDetectionArray::ConstPtr& taginfo);
        // void apriltag_pose_callback(const geometry_msgs::PoseArray::ConstPtr& msg) ;

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

        // subscribe to apriltag topic
        apriltag_pose_sub = n.subscribe("camera/tag_detections",10,&outerloop_calc::apriltag_pose_callack, this);
        // apriltag_pose_sub = n.subscribe("camera/tag_detections_pose", 10, &outerloop_calc::apriltag_pose_callback,this);  
}

void outerloop_calc::init_variables()
{
        x = 0 ; y = 0 ; z = 0 ; theta = 0; id = 0;

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
        

        //kp_v = 10 ; 
        //kd_v = 10 ;
        //ki_v = 0  ; //0.05

        //kp_w = 10 ; 
        //kd_w = 10 ;
        //ki_w = 0  ; //0.05 

        roll = 0; pitch = 0 ; yaw = 0;
 
        //h = ki/kp ;
        //z_p = 0 ;
        //pref_out_p = 0 ;
        //pref_out = 0 ; 

        vd_ctrl = 0;vd_ctrl_p = 0 ; vd_ctrl_pp = 0 ;
        wd_ctrl = 0;wd_ctrl_p = 0 ; wd_ctrl_pp = 0 ;

        dist_error = 0; dist_error_p  = 0 ; dist_error_pp  = 0 ;
        omega_error = 0;omega_error_p = 0 ; omega_error_pp = 0 ;

	rate = 20 ; 
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

            dist_error = 0.3 - z ;
            omega_error = 0 - x ;

                 
	    //CL = ((alpha*td*td*ki+2*alpha*td*kp)*Lerror + (2*alpha*td*td*ki)*Lerror_p + (alpha*td*td*ki-2*alpha*td*kp)*Lerror_pp + 8*CL_p - (4-2*alpha*td)*CL_pp)/(2*alpha*td + 4);

            // vd_ctrl = vd_ctrl_p + 0.5*kp*(2+(ki/kp)*td)*dist_error + 0.5*((ki/kp)*td - 2)*dist_error_p ;
            //  //
            // vd_ctrl = kp*(dist_error + dist_error_p) + 2*kd*(dist_error - dist_error_p)/td - vd_ctrl_p ;

            // PI no rolloff
            //vd_ctrl = vd_ctrl_p + (kp_v + (td/2)*ki_v)*dist_error_p + ((td/2)*ki_v - kp_v)*dist_error ; // kp = 10, kd = 10 
            //wd_ctrl = wd_ctrl_p + (kp_w + (td/2)*ki_w)*omega_error_p + ((td/2)*ki_w - kp_w)*omega_error ;

            // PD (normal controller)
            vd_ctrl = kp_v*dist_error + kd_v*(dist_error - dist_error_p)/td    ;
            wd_ctrl = kp_w*omega_error + kd_w*(omega_error - omega_error_p)/td ;

            // PID
            //vd_ctrl = dist_error*(2*kp*td+ki*td*td+4*kd)/(2*td) + dist_error_p*(2*ki*td*td-8*kd)/(2*td) + dist_error_pp*(4*kd-2*kp*td+ki*td*td)/(2*td) +2*vd_ctrl_p - vd_ctrl_pp ;

            // PI * rolloff
            // vd_ctrl = ((alpha*td*td*ki+2*alpha*td*kp)*dist_error  + (2*alpha*td*td*ki)*dist_error_p  + (alpha*td*td*ki-2*alpha*td*kp)*dist_error_pp  + 8*vd_ctrl_p - (4-2*alpha*td)*vd_ctrl_pp)/(2*alpha*td + 4); 
            //wd_ctrl = ((alpha*td*td*ki+2*alpha*td*kp)*omega_error + (2*alpha*td*td*ki)*omega_error_p + (alpha*td*td*ki-2*alpha*td*kp)*omega_error_pp + 8*wd_ctrl_p - (4-2*alpha*td)*wd_ctrl_pp)/(2*alpha*td + 4);

            geometry_msgs::Twist cmd_data ;
            cmd_data.linear.x = -0.1   -vd_ctrl ; // "-ve" to make control signal positive
            cmd_data.linear.y = 0 ;
            cmd_data.linear.z = 0 ;
            cmd_data.angular.x = 0   ;      // roll
            cmd_data.angular.y = 0   ;     // pitch
            cmd_data.angular.z =  0 ;wd_ctrl   ;     // yaw
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
            
/*void outerloop_calc::apriltag_pose_callback(const geometry_msgs::PoseArray::ConstPtr& msg)
{ 
  if (msg->poses.size() > 0)
  {
   x = msg->poses[0].position.x ;
   y = msg->poses[0].position.y ;
   z = msg->poses[0].position.z ;
  }
}*/

void outerloop_calc::apriltag_pose_callack(const apriltags_ros::AprilTagDetectionArray::ConstPtr& msg)
{
      
   /*for(std::vector<apriltags_ros::AprilTagDetection>::const_iterator tag = tagarray->detections.begin(); tag != tagarray->detections.end(); tag++)
   {
      x = (*tag).pose.pose.position.x ;
      y = (*tag).pose.pose.position.y ;
      z = (*tag).pose.pose.position.z ;

      
      std_msgs::String msg;
      std::stringstream debug;
      debug <<"Tag ID is : "<< (*tag).id ; 
      msg.data = debug.str();
      ros::Publisher p = nh.advertise<std_msgs::String>("/debug_cb", 100);
      p.publish(msg);  
   }*/ 

   if (msg->detections.size() > 0)
   {    
     ROS_DEBUG("%d tag detected", (int)msg->detections.size()); 
      
     for (int i = 0 ; i <msg->detections.size() ; i++)
     {
      // AprilTagDetection tag_detection;

      id = msg->detections[i].id ;
      geometry_msgs::PoseStamped apriltag = msg->detections[i].pose ;
      x = apriltag.pose.position.x ;
      y = apriltag.pose.position.y ;
      z = apriltag.pose.position.z ;
      // T_tag2cam << x, y, z, 0 ;

      tf::Quaternion q(apriltag.pose.orientation.x, apriltag.pose.orientation.y, apriltag.pose.orientation.z, apriltag.pose.orientation.w) ;
      tf::Matrix3x3 m(q);
      //double roll, pitch, yaw;
      m.getRPY(roll, pitch, yaw);     
     } 
   }
   else 
   {
     z = 0.3 ;
     x = 0 ;
   } 
}

int main(int argc, char **argv)
{
	ros::init(argc, argv,"outerloop2");
	outerloop_calc obj;
	obj.spin();
	return 0;
}
