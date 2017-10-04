//  ///////////////////////////////////////////////////////////
//
// turtlebot_example.cpp
// This file contains example code for use with ME 597 lab 1
// It outlines the basic setup of a ros node and the various 
// inputs and outputs.
// 
// Author: James Servos 
//
// //////////////////////////////////////////////////////////

#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_datatypes.h>
#include <math.h>

double desired_x[1] = 0.0;
double desired_y[1] = 0.0;
double desired_yaw = 0.0
int stage = 0, pos = 0;

//Callback function for the Position topic 
//void pose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
//{
	//This function is called when a new position message is received

//	double X = msg->pose.pose.position.x; // Robot X psotition
//	double Y = msg->pose.pose.position.y; // Robot Y psotition
//	double Yaw = tf::getYaw(msg->pose.pose.orientation); // Robot Yaw

//}

void pose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
	//This function is called when a new position message is received

	double X = msg->pose.pose.position.x; // Robot X psotition
	double Y = msg->pose.pose.position.y; // Robot Y psotition
 	double Yaw = tf::getYaw(msg->pose.pose.orientation); // Robot Yaw
	ROS_INFO("X = %f",X);
	ROS_INFO("Y = %f",Y);
	ROS_INFO("YAW =  %f", Yaw);

}

void calculateYaw()
{
    tan
}


int main(int argc, char **argv)
{
	//Initialize the ROS framework
    ros::init(argc,argv,"main_control");
    ros::NodeHandle n;

    //Subscribe to the desired topics and assign callbacks
    ros::Subscriber pose_sub = n.subscribe("/amcl_pose", 1, pose_callback);

    //Setup topics to Publish from this node
    ros::Publisher velocity_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/teleop", 1);
    
    //Velocity control variable
    geometry_msgs::Twist vel;

    //Set the loop rate
    ros::Rate loop_rate(20);    //20Hz update rate

    while (ros::ok())
    {
    	loop_rate.sleep(); //Maintain the loop rate
    	ros::spinOnce();   //Check for new messages
    
    	//Main loop code goes here:


        if (fabs(desired_x[pos] - X) > 0.1 || fabs(desired_y[pos] - Y) > 0.1)
        {
            ROS_INFO("not yet");
            calculateYaw();
        }

    	vel.linear.x = 0.0; // set linear speed
    	vel.angular.z = 0.0; // set angular speed

    	velocity_publisher.publish(vel); // Publish the command velocity
    }

    return 0;
}
