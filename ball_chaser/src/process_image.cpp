#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include "sensor_msgs/Image.h"

//Define a global client that can request services
ros::ServiceClient client;

//Function of calling the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z){
    //Request a service and pass the velocites to it to drive the robot
}

//Callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img){

    int white_pixel = 255;
    //Loop through each pixel in the image and check if there's a bright white one
    //Then, identify if this pixel falls in the left, mid, or right side of the image
    //Depending on the white ball position, call the drive_bot function and pass velocities to it 
    //Request a stop when there's no white ball seen by the camera
}

int main(int argc, char**argv){

    //
}