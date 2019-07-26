#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include "sensor_msgs/Image.h"

//Define a global client that can request services
ros::ServiceClient client;

//Function of calling the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z){

    ROS_INFO("Drive the robot: ");
    
    //Request a service and pass the velocites to it to drive the robot
    ball_chaser::DriveToTarget srv;
    srv.request.linear_x = lin_x;
    srv.request.angular_z = ang_z;

}

//Callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img){

    int white_pixel = 255;
    int left_bound = int(img.width / 3);
    int right_bound = 2 * int(img.width / 3);
    int right_count = 0; int mid_count = 0; int left_count = 0;
    int max_count = 0;
    //Loop through each pixel in the image and check if there's a bright white one
    for (int i = 0; i < img.height; i++){
        for (int j = 0; j < img.step; j++){
            int index = j + (i * img.step);
            if (img.data[index] == white_pixel){
                //Then, identify if this pixel falls in the left, mid, or right side of the image
                if (i <= left_bound){
                    left_count += 1;
                } else if (i >= right_bound){
                    right_count += 1;
                } else {
                    mid_count += 1;
                }
            } else{
                continue;
            }
        }
    }
    //Depending on the white ball position, call the drive_bot function and pass velocities to it 
    //Request a stop when there's no white ball seen by the camera
    max_count = std::max(std::max(left_count, mid_count), right_count);
    if (max_count == 0){
        drive_robot(0.0, 0.0);
    } else if (max_count == left_count){
        drive_robot(0.5, 0.5);
    } else if (max_count == right_count){
        drive_robot(0.5, -0.5);
    } else{
        drive_robot(0.5, 0.0);
    }
    
}

int main(int argc, char**argv){

    //Initialize the process_image node and create a handle to it
    ros::init(argc, argv, "process_image");
    ros::NodeHandle n;

    //Define a client service capable of requesting services from command _robot
    client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

    //Subscribe to /camera/rgb/image_raw topic to read the image data inside the process_image_callback function
    ros::Subscriber sub1 = n.subscribe("/camera/rgb/image_raw", 10, process_image_callback);

    //Handle ROS communication events
    ros::spin();
    
    return 0;
}