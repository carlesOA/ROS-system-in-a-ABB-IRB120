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

//Counter the number of clusters that we had move
int cluster_move = 0;

//Leave piece position. Falta modificar
Eigen::Affine3d leave = Eigen::Translation3d(0.0, 0.0, 0.2) *
                           Eigen::Quaterniond(0.495, 0.498, 0.515, 0.49);

//Eigen variables
Eigen::Affine3d leave_offset, aproach, pick, cluster_pose;


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
	ros::Time now = ros::Time::now();
	listener.waitForTransform("/base_link", cluster, now, ros::Duration(2.0));
	listener.lookupTransform("/base_link", cluster, now, transform);
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


//Function to move the robot to stimate pose of the cluster
void move(Eigen::Affine3d cluster_pose)
{  
  
  //ros::AsyncSpinner spinner(1);
  //spinner.start();
    std::cout << "Entra al move \n \n \n";
  //move_group_interface::MoveGroup group("manipulator");
  //group.setPlannerId("RRTConnectkConfigDefault");
  //group.setMaxVelocityScalingFactor(0.2);
  //Desprès del manipulator surt de la funció! WTF???
  group.setNamedTarget("pose1");

  group.move();
  std::cout << "A dormir \n \n \n";
  sleep(5);
  //Home position
  group.setNamedTarget("home");
  group.move();

  //Aproach position. S'ha de modificar el valor
  aproach = cluster_pose.translate(-0.2*Eigen::Vector3d::UnitZ());

  group.setPoseTarget(aproach);
  group.move();

  //Pick position

  pick = cluster_pose.translate(-0.1*Eigen::Vector3d::UnitZ());

  group.setPoseTarget(pick);
  group.move();
  // Activar ventosa
  //Verificar activació ventosa


  //Retrac position
  group.setPoseTarget(aproach);
  group.move();

  
  // Leave position
  if (cluster_move == 0){

	leave_offset = leave;
	group.setPoseTarget(leave);
  	group.move();
	
  }
  else{
	
	leave_offset = leave_offset.translate(0.4*Eigen::Vector3d::UnitY()); //Verificar que m'he de moure amb Y i no amb X. També la distancia
	group.setPoseTarget(leave_offset);
  	group.move();
  }

  cluster_move++;
}


void ClusterCallback(const std_msgs::String::ConstPtr& msg)
{
  cluster = msg->data.c_str();
  
  clusterPose = objectPose(cluster);

  cluster_pose = Eigen::Translation3d(clusterPose.position.x, clusterPose.position.y, clusterPose.position.z) * Eigen::Quaterniond(0.4915, 0.49153, 0.50833, 0.50836);

  move(cluster_pose);

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "position_cluster");

  ros::NodeHandle n;  

  ros::Subscriber sub_cluster = n.subscribe("/big_cluster", 1, ClusterCallback);

  ros::spin();
}
