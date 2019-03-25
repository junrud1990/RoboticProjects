#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

// Define a global client that can request services
ros::ServiceClient client;

// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z)
{
    ball_chaser::DriveToTarget srv;
    srv.request.linear_x = lin_x;
    srv.request.angular_z = ang_z;
    if (client.call(srv)) {
        //ROS_INFO("[Process_image]Service return: %s\n", srv.response.msg_feedback.c_str());
    }
    else {
        ROS_ERROR("[Process_image]Failed to call service DriveToTarget");
    }
}

// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img)
{

    const int white_pixel = 255;
    /* TODOs:
        1. Find center point of the ball (Done)
        2. Simple P controller (Done)
        3. Count how many white pixels (Done)
        4. Detect ball based on shape. Maybe gradient?
     */
    int cnt = 0;
    int leftIdxMin = img.width;
    int rightIdxMax = -1;
    int upIdxMin = img.height;
    int downIdxMax = -1;
    bool found = false;
    for (int h = 0; h < img.height; h++) {
        for (int w = 0; w < img.width; w++) {
            if(img.data[h*img.step+w*3] == white_pixel && img.data[h*img.step+w*3+1] == white_pixel && img.data[h*img.step+w*3+2] == white_pixel) {
                found = true;
                if (leftIdxMin > w) {
                    leftIdxMin = w;
                }
                if (rightIdxMax < w) {
                    rightIdxMax = w;
                }
                if (upIdxMin > h) {
                    upIdxMin = h;
                }
                if (downIdxMax < h) {
                    downIdxMax = h;
                }
                cnt++;
            }
            //ROS_INFO("[Process_image]Image: %d\n", img.data[h*img.step+w]);
        }
    }
    //ROS_INFO("[Process_image] encoding: %s\n", img.encoding.c_str());
    if(!found) {
        drive_robot(0,0);
    }
    else {
        const int ballRow = (upIdxMin + downIdxMax) / 2;
        const int ballCol = (leftIdxMin + rightIdxMax) / 2;
        const float lin_base = 0.3;
        const float ang_base = 1;
        //ROS_INFO("[Process_image]  %d, %d, %d, %d, (%d,%d)\n", leftIdxMin, rightIdxMax, upIdxMin, downIdxMax, img.height, img.width);
        //Check the position of the while pixel position
        float k_ang=0, k_lin=0, lin_x=0, ang_z = 0;
        if (ballCol < img.width * 5 / 11) {
            //Left
            k_ang = 1 - 1.0 * ballCol / (img.width * 5 / 11);
            k_lin = 1 - 1.0 * cnt / img.width / img.height;
            lin_x = k_lin * lin_base;
            ang_z = k_ang * ang_base;
            drive_robot(lin_x, ang_z);
        }
        else if (ballCol > 6 * img.width / 11) {
            //Right
            k_ang = 1 - 1.0 * ballCol / (img.width * 6 / 11);
            k_lin = 1 - 1.0 * cnt / img.width / img.height;
            lin_x = k_lin * lin_base;
            ang_z = k_ang * ang_base;
            drive_robot(lin_x, ang_z);
        } else {
            //Middle
            k_lin = 1 - 1.0 * cnt / img.width / img.height;
            if (k_lin < 0.05)
                k_lin = 0;
            lin_x = k_lin * lin_base;
            ang_z = 0;
            drive_robot(lin_x, ang_z);
        }
        //ROS_INFO("[Process_image] row:%d, col:%d \n %f, %f, %f, %f \n", ballRow, ballCol, k_ang, k_lin, lin_x, ang_z);
    }
}

int main(int argc, char** argv)
{
    // Initialize the process_image node and create a handle to it
    ros::init(argc, argv, "process_image");
    ros::NodeHandle n;

    // Define a client service capable of requesting services from command_robot
    client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

    // Subscribe to /camera/rgb/image_raw topic to read the image data inside the process_image_callback function
    ros::Subscriber sub1 = n.subscribe("/camera/rgb/image_raw", 10, process_image_callback);

    // Handle ROS communication events
    ros::spin();

    return 0;
}
