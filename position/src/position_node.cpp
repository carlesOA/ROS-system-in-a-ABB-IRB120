#include <ros/ros.h>
#include <moveit/move_group_interface/move_group.h>
#include "std_msgs/String.h"
#include <sstream>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>

#include <tf/transform_listener.h>

#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "/opt/ros/indigo/include/eigen_conversions/eigen_msg.h"

#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <iostream>

std::string cluster;

geometry_msgs::PoseStamped clusterPose;

//Function to transform from cluster to base_link

void ClusterCallback(const std_msgs::String::ConstPtr& msg)
{
  cluster = msg->data.c_str();

  tf::TransformListener listener;
  tf::StampedTransform transform;

  while (true)
  {
  try
    {
	ros::Time now = ros::Time::now();
	listener.waitForTransform("/base_link", cluster, now, ros::Duration(2.0)); //Modificar???
	listener.lookupTransform("/base_link", cluster, now, transform);

      break;
    }
    catch (tf::TransformException ex)
    {
      ROS_WARN("%s",ex.what());
    }
  }
  clusterPose.pose.position.x = transform.getOrigin().x();
  clusterPose.pose.position.y = transform.getOrigin().y();
  clusterPose.pose.position.z = transform.getOrigin().z();
  
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "position_cluster");

  ros::NodeHandle n;  

  ros::Subscriber sub_cluster = n.subscribe("/big_cluster", 1, ClusterCallback);

  ros::Publisher pub_pose_ = n.advertise<geometry_msgs::PoseStamped>("pose", 100);
  
  ros::Rate loop_rate(100); //Ajustar valor
 
   while(ros::ok())
  {
    //Publish the position of the cluster
    pub_pose_.publish(clusterPose);

    ros::spinOnce();
    loop_rate.sleep(); // El puc treure? 

  }

  ros::spin(); //Fa falta el Ros::spin o el puc treure?
}
