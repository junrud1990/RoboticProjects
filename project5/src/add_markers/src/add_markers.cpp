/*
 * Copyright (c) 2010, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

// %Tag(FULLTEXT)%
// %Tag(INCLUDES)%
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>

const double pick_xyw[3] = {-4.0, 0.5, 1};
const double drop_xyw[3] = {3.0, -2.5, 1};
bool pickup = true;
// Set our initial shape type to be a cube
const uint32_t shape = visualization_msgs::Marker::CUBE;
visualization_msgs::Marker marker;
ros::Publisher marker_pub;
const float red[3] = {1.0f, 0.0f, 0.0f};
const float green[3] = {0.0f, 1.0f, 0.0f};

void delete_marker(visualization_msgs::Marker *marker)
{
  marker->action = visualization_msgs::Marker::DELETE;
}

void edit_marker(visualization_msgs::Marker *marker, const double goal[2], const float color[3])
{
  // Set the frame ID and timestamp.  See the TF tutorials for information on these.
  marker->header.frame_id = "map";
  marker->header.stamp = ros::Time::now();

  // Set the namespace and id for this marker.  This serves to create a unique ID
  // Any marker sent with the same namespace and id will overwrite the old one
  marker->ns = "basic_shapes";
  marker->id = 0;

  // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
  marker->type = shape;

  // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
  marker->action = visualization_msgs::Marker::ADD;

  // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
  marker->pose.position.x = goal[0];
  marker->pose.position.y = goal[1];
  marker->pose.position.z = 0;
  marker->pose.orientation.x = 0.0;
  marker->pose.orientation.y = 0.0;
  marker->pose.orientation.z = 0.0;
  marker->pose.orientation.w = 1.0;

  // Set the scale of the marker -- 1x1x1 here means 1m on a side
  marker->scale.x = 0.2;
  marker->scale.y = 0.2;
  marker->scale.z = 0.2;

  // Set the color -- be sure to set alpha to something non-zero!
  marker->color.r = color[0];
  marker->color.g = color[1];
  marker->color.b = color[2];
  marker->color.a = 1.0;

  marker->lifetime = ros::Duration();

}
double sqrt_dist(const double x[2], const double y[2])
{
  return sqrt((x[0]-y[0])*(x[0]-y[0]) + (x[1]-y[1])*(x[1]-y[1]));
}

void poseCallback(const nav_msgs::Odometry::ConstPtr& msg){
  double xy[2] = {msg->pose.pose.position.x, msg->pose.pose.position.y};
  if(pickup) {
    if(sqrt_dist(xy, pick_xyw) < 0.01) {
      // pickup is done;
      pickup = false;
      // delete marker
      delete_marker(&marker);
      marker_pub.publish(marker);
      // add new drop off zone marker
      edit_marker(&marker, drop_xyw, red);
      marker_pub.publish(marker);
    }
  } else {
    if(sqrt_dist(xy, drop_xyw) < 0.01) {
      // dropoff is done;
      pickup = true;
      // delete marker
      delete_marker(&marker);
      marker_pub.publish(marker);
      // add new drop off zone marker
      edit_marker(&marker, pick_xyw, green);
      marker_pub.publish(marker);
    }
  }
	//ROS_INFO("Seq: [%d]", msg->header.seq);
	//ROS_INFO("Position-> x: [%f], y: [%f]", msg->pose.pose.position.x,msg->pose.pose.position.y);
}

int main( int argc, char** argv )
{
  ros::init(argc, argv, "basic_shapes");
  ros::NodeHandle n;
  marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  while (marker_pub.getNumSubscribers() < 1) {
    ROS_WARN_ONCE("Please create a subscriber to the marker");
    sleep(1);
  }
  edit_marker(&marker, pick_xyw, green);
  marker_pub.publish(marker);
  ROS_INFO("publish markers!");
  ros::Subscriber sub = n.subscribe("/odom",1000, poseCallback);
  ros::spin();
}
