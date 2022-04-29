#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include "nav_msgs/Odometry.h"
#include <tf/tf.h>
#include <math.h>

float odom_x = 0.0, odom_y = 0.0;
float goals[2][3] = { {1.5, 1.5, 0}, {1.5, 0, 0}  };
float tolerance = 0.7;
bool have_thing = false;

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  ::odom_x = msg->pose.pose.position.x;
  ::odom_y = msg->pose.pose.position.y;
}

int main( int argc, char** argv )
{
  ros::init(argc, argv, "add_markers");
  ros::NodeHandle n;
  ros::Rate r(20);
  ros::Subscriber obom_sub = n.subscribe("/odom", 1000, odomCallback);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

  visualization_msgs::Marker marker;
  // Set the frame ID and timestamp.  See the TF tutorials for information on these.
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time::now();

  // Set the namespace and id for this marker.  This serves to create a unique ID
  // Any marker sent with the same namespace and id will overwrite the old one
  marker.ns = "add_markers";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::CUBE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = goals[0][0];
  marker.pose.position.y = goals[0][1];
  marker.pose.position.z = 0;
  marker.pose.orientation = tf::createQuaternionMsgFromYaw(goals[0][2]);

  marker.scale.x = 0.5;
  marker.scale.y = 0.5;
  marker.scale.z = 0.25;

  marker.color.r = 0.0f;
  marker.color.g = 0.0f;
  marker.color.b = 1.0f;
  marker.color.a = 1.0;

  marker.lifetime = ros::Duration();

  while (ros::ok()) {
	  float dis = sqrt(pow((odom_x - marker.pose.position.x), 2) + pow((odom_y - marker.pose.position.y), 2));
    ROS_INFO("cur dis: %f", dis);
	  if (!have_thing) {
		  if(dis < tolerance) {
        ros::Duration(5.0).sleep();
		    marker.action = visualization_msgs::Marker::DELETE;
		    have_thing = true;
		  }
	  }
	  else {
		  marker.action = visualization_msgs::Marker::ADD;
		  marker.pose.position.x = goals[1][0];
		  marker.pose.position.y = goals[1][1];
		  marker.pose.orientation = tf::createQuaternionMsgFromYaw(goals[1][2]);
		  have_thing = false;
	  }

    marker_pub.publish(marker);
    ros::spinOnce();
  }
  return 0;
}
