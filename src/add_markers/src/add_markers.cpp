#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include "nav_msgs/Odometry.h"
#include <cmath>

double pickUpPos[2]  = {1.5, 1.5};
double dropOffPos[2] = {1.5, 0};
double pose[2] = {0, 0};  // current pose

void get_current_pose(const nav_msgs::Odometry::ConstPtr& msg)
{
  pose[0] = msg->pose.pose.position.x;
  pose[1] = msg->pose.pose.position.y;
}

double distToCurrentPos(double goalPos[2])
{
  double dx = goalPos[0] - pose[0];
  double dy = goalPos[1] - pose[1];
  return sqrt(dx*dx + dy*dy);
}

bool reach_pick_up()
{
  return distToCurrentPos(pickUpPos) < 0.8;
}

bool reach_drop_zone()
{
  return distToCurrentPos(dropOffPos) < 0.2;
}

enum State {
  PICKUP,  // going to pick up zon
  CARRY,   // carry to drop zone
  DROP,    // already drop
} state = PICKUP;

double beg_s;
bool first = true;
void callback(const ros::TimerEvent& event) {
    if(first) {
      beg_s = ros::Time::now().toSec();
      first = false;
      return;
    }
    double passed_s = ros::Time::now().toSec() - beg_s;
    ROS_INFO("Timer passed %f", passed_s);

    if (state == PICKUP && passed_s >= 5) {
        ROS_INFO("Hide ... ");
        state = CARRY;
    }
    if (state == CARRY && passed_s >= 10) {
        ROS_INFO("Show ... ");
        state = DROP;
    }
    else {

    }
}

int main( int argc, char** argv )
{
  ros::init(argc, argv, "add_markers");
  ros::NodeHandle nh;
  ros::Rate r(1);
  ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  ros::Subscriber pose_sub = nh.subscribe("odom", 10, get_current_pose);

  visualization_msgs::Marker marker;
  marker.header.frame_id = "odom";
  marker.header.stamp = ros::Time::now();
  marker.ns = "basic_shapes";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::CUBE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = pickUpPos[0];
  marker.pose.position.y = pickUpPos[1];
  marker.pose.position.z = 0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.5;
  marker.scale.y = 0.5;
  marker.scale.z = 0.5;
  marker.color.r = 0.0f;
  marker.color.g = 1.0f;
  marker.color.b = 1.0f;
  marker.color.a = 1.0;
  marker.lifetime = ros::Duration();

  ROS_INFO("Going to pick up zone ... ");
  ros::Timer timer = nh.createTimer(ros::Duration(1), callback);

  bool once_i = true;
  bool once_ii = true;
  bool once_iii = true;
  bool once_iv = true;
  bool once_v = true;

  while (ros::ok())
  {
    ros::spinOnce();
    marker_pub.publish(marker);

    // always hide if target reached
    if(reach_pick_up() && state == PICKUP) {
      marker.action = visualization_msgs::Marker::DELETE;
      if(once_i) { ROS_INFO("PICKUP(): Goal reached"); once_i = false; }
      continue;
    }

    if(reach_drop_zone() && state == DROP) {
      marker.action = visualization_msgs::Marker::DELETE;
      if(once_ii) { ROS_INFO("DROP(): Goal reached"); once_ii = false; }
      continue;
    }

    // timer behavior
    if (state == PICKUP) {
      marker.action = visualization_msgs::Marker::ADD;
      marker.pose.position.x = pickUpPos[0];
      marker.pose.position.y = pickUpPos[1];
      if(once_iii) { ROS_INFO("Show pick-up"); once_iii = false; }
    }
    else if (state == CARRY) {
      marker.action = visualization_msgs::Marker::DELETE;
      marker.pose.position.x = dropOffPos[0];
      marker.pose.position.y = dropOffPos[1];
      if(once_iv) { ROS_INFO("Removed pick-up location after 5s"); once_iv = false; }
    }
    else if (state == DROP) {
      marker.action = visualization_msgs::Marker::ADD;
      marker.pose.position.x = dropOffPos[0];
      marker.pose.position.y = dropOffPos[1];
      if(once_v) { ROS_INFO("Show drop location"); once_v = false; }
    }
    else {
      marker.action = visualization_msgs::Marker::DELETE;
    }
  }
}
