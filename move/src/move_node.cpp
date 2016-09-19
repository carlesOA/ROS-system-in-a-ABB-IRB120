#include <ros/ros.h>
#include <moveit/move_group_interface/move_group.h>
#include "/opt/ros/indigo/include/eigen_conversions/eigen_msg.h"

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include "std_msgs/String.h"
#include <boost/bind.hpp>

//Counter the number of clusters that we had move
int cluster_move = 0;

//Eigen variables. Leave piece position. Falta modificar
Eigen::Affine3d leave = Eigen::Translation3d(0.0, -0.3, 0.2) *
                           Eigen::Quaterniond(0.0, 0.0, 0.0, 0.0);

Eigen::Affine3d leave_offset, aproach, pick, pose_move, pose_move_mod;

// Callback function and move function
void ClusterCallback_move(const geometry_msgs::PoseStamped& sub_pose)
{

  ros::NodeHandle n;
  
  //Translation the position passed by the Position package
  pose_move = Eigen::Translation3d(sub_pose.pose.position.x, sub_pose.pose.position.y, sub_pose.pose.position.z) * Eigen::Quaterniond(0.0, 0.0, 0.0, 0.0);
  
  //duplicate this pose to translate
  pose_move_mod = pose_move;

  std::cout << sub_pose.pose.position.x << "\n";
  std::cout << sub_pose.pose.position.y << "\n";
  std::cout << sub_pose.pose.position.z << "\n";

  //Initiating the group to move the robot
  ros::AsyncSpinner spinner(1);
  spinner.start();  
  move_group_interface::MoveGroup group("manipulator");
  group.setPlannerId("RRTConnectkConfigDefault");
  //group.setMaxVelocityScalingFactor(0.2);

  //Start the sequence
  //Home position
  group.setNamedTarget("home");
  group.move();
  sleep(3);

  //Aproach position.
  aproach = pose_move_mod.translate(0.2*Eigen::Vector3d::UnitZ()); // Z = Amunt
  aproach = pose_move_mod.translate(-0.1*Eigen::Vector3d::UnitX()); // X = Endavant

  group.setPoseTarget(aproach);
  group.move();
  sleep(3);

  //Pick position
  pick = pose_move.translate(0.1*Eigen::Vector3d::UnitZ()); // Z = Amunt
  pick = pose_move.translate(-0.1*Eigen::Vector3d::UnitX()); // X = Endavant

  group.setPoseTarget(pick);
  group.move();
  sleep(3);

  //Retrac position
  group.setPoseTarget(aproach);
  group.move();
  sleep(3);
  
  // Leave position
  if (cluster_move == 0){

	leave_offset = leave;
	group.setPoseTarget(leave);
  	group.move();
	
  }
  else{
	
	leave_offset = leave_offset.translate(0.1*Eigen::Vector3d::UnitY()); //Desplaçar lateralment la peça
	group.setPoseTarget(leave_offset);
  	group.move();
  }

  cluster_move++;

}

int main(int argc, char **argv)
  {
	ros::init(argc, argv, "move_group_package");

	ros::NodeHandle n;

	ros::Subscriber sub_cluster = n.subscribe("/pose", 1000, ClusterCallback_move);

        ros::spin();

  }
