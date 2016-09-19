#include <ros/ros.h>
#include <moveit/move_group_interface/move_group.h>
#include "std_msgs/String.h"
#include <sstream>
#include <geometry_msgs/Pose.h>

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
geometry_msgs::Pose clusterPose;
geometry_msgs::Pose publish__;

ros::Publisher pub_pose_;

//Eigen variables
Eigen::Affine3d cluster_pose;


//Function to transform from cluster to base_link
geometry_msgs::Pose objectPose(std::string cluster)
{
  geometry_msgs::Pose object;

  tf::TransformListener listener;
  tf::StampedTransform transform;

  while (true)
  {
  try
    {
	//ros::Time now = ros::Time::now();
	//listener.waitForTransform("/base_link", cluster, now, ros::Duration(2.0));
	//listener.lookupTransform("/base_link", cluster, now, transform);

        listener.waitForTransform("/base_link", cluster, ros::Time(0), ros::Duration(2.0));
	listener.lookupTransform("/base_link", cluster, ros::Time(0), transform);
      break;
    }
    catch (tf::TransformException ex)
    {
      ROS_WARN("%s",ex.what());
    }
  }
  object.position.x = transform.getOrigin().x();
  object.position.y = transform.getOrigin().y();
  object.position.z = transform.getOrigin().z();

  return object;
}


void ClusterCallback(const std_msgs::String::ConstPtr& msg)
{
  cluster = msg->data.c_str();
  
  clusterPose = objectPose(cluster);

  cluster_pose = Eigen::Translation3d(clusterPose.position.x, clusterPose.position.y, clusterPose.position.z) * Eigen::Quaterniond(0.4915, 0.49153, 0.50833, 0.50836);
  
  //std::cout << publish__.position.x << "\n";
  //std::cout << publish__.position.y << "\n";
  //std::cout << publish__.position.z << "\n";

  tf::poseEigenToMsg(cluster_pose , publish__);

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "position_cluster");

  ros::NodeHandle n;  

  ros::Subscriber sub_cluster = n.subscribe("/big_cluster", 1, ClusterCallback);

  ros::Publisher pub_pose_ = n.advertise<geometry_msgs::Pose>("pose", 100);
  
  ros::Rate loop_rate(100); //Ajustar valor
 
   while(ros::ok())
  {
    //Publish the original point cloud
    pub_pose_.publish(publish__);

    std::cout << publish__.position.x << "\n";
    std::cout << publish__.position.y << "\n";
    std::cout << publish__.position.z << "\n";

    ros::spinOnce();
    loop_rate.sleep();

  }

  ros::spin();
}
