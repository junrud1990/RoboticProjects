#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <std_msgs/Bool.h>

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
std_msgs::Bool Reached;

int main(int argc, char** argv){
  // Initialize the simple_navigation_goals node
  ros::init(argc, argv, "simple_navigation_goals");

  ros::NodeHandle n;
  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);
  ros::Publisher poseChecker  = n.advertise<std_msgs::Bool>("PoseReached",1000);
  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;
  double pick_xyw[3] = {-4.0, 0.5, 1};
  double drop_xyw[3] = {3.0, -2.5, 1};
  while(ros::ok()) {
    {
      // set up the frame parameters
      goal.target_pose.header.frame_id = "map";
      goal.target_pose.header.stamp = ros::Time::now();

      // Define a position and orientation for the robot to reach
      goal.target_pose.pose.position.x = pick_xyw[0];
      goal.target_pose.pose.position.y = pick_xyw[1];
      goal.target_pose.pose.orientation.w = pick_xyw[2];

      // Send the goal position and orientation for the robot to reach
      ROS_INFO("Sending pickup zone goal");
      ac.sendGoal(goal);

      // Wait an infinite time for the results
      ac.waitForResult();

      // Check if the robot reached its goal
      if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_INFO("Reach pickup zone");
        poseChecker.publish(Reached);
      }
      else
        ROS_INFO("Fail to reach pickup zone");
    }
    ros::Duration(2).sleep();
    {
      goal.target_pose.header.frame_id = "map";
      goal.target_pose.header.stamp = ros::Time::now();

      // Define a position and orientation for the robot to reach
      goal.target_pose.pose.position.x = drop_xyw[0];
      goal.target_pose.pose.position.y = drop_xyw[1];
      goal.target_pose.pose.orientation.w = drop_xyw[2];

      // Send the goal position and orientation for the robot to reach
      ROS_INFO("Sending drop off zone goal");
      ac.sendGoal(goal);

      // Wait an infinite time for the results
      ac.waitForResult();

      // Check if the robot reached its goal
      if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_INFO("Reach drop off zone");
        poseChecker.publish(Reached);
      }
      else
        ROS_INFO("Fail to reach drop off zone");
    }
  }
  return 0;
}
