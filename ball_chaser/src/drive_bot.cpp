#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
//the ball_chaser "DriveToTarget" header file
#include "ball_chaser/DriveToTarget.h"

//ROS::Publisher motor commands
ros::Publisher motor_command_publisher;

//handle_drive_request callback function that executes whenever a drive_bot service is requested
bool handle_drive_request(ball_chaser::DriveToTarget::Request& req, ball_chaser::DriveToTarget::Response& res){

	ROS_INFO("DriveToTarget Request received - linear_x:%1.2f, angular_z:%1.2f", (float)req.linear_x, (float)req.angular_z);

	//Publish the motor command from the service request
	geometry_msgs::Twist motor_commands;
	motor_commands.linear.x = req.linear_x;
	motor_commands.angular.z = req.angular_z;
	
	motor_command_publisher.publish(motor_commands);

	//Wait 3 seconds for the robot to settle 
	//ros::Duration(3).sleep();

	//Return a response message
	res.msg_feedback = "Robot move - linear_x: " + std::to_string(motor_commands.linear.x) + ", angular_z: " + std::to_string(motor_commands.angular.z);
	ROS_INFO_STREAM(res.msg_feedback);

	return true;
}

int main(int argc, char** argv){

	//Initialize a ROS node 
	ros::init(argc, argv, "drive_bot");
	
	//Create a ROS NodeHandle object
	ros::NodeHandle n;

	//Inform ROS master that we will be publishing a message of type geometry_msgs::Twist on the robot actuation topic with a publishing size of 10 
	motor_command_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel",10);

	//Define a drive /ball_chaser/command_robot service with a handle_drive_request callback function
	ros::ServiceServer service = n.advertiseService("/ball_chaser/command_robot", handle_drive_request);
	ROS_INFO("Ready to send motor commands");

	//Handle ROS communcation events
	ros::spin();

	return 0;
	
}
