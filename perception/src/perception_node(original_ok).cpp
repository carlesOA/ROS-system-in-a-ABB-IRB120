#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>
#include <pcl/visualization/cloud_viewer.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <pcl/filters/crop_box.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>

#include <tf/transform_listener.h>

#include "std_msgs/String.h"
#include <sstream>

int n_cluster, big_cluster, cluster_size;

std::vector<ros::Publisher> pub_vec;

std::string cluster_name;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

sensor_msgs::PointCloud2::Ptr original_cloud (new sensor_msgs::PointCloud2);
sensor_msgs::PointCloud2::Ptr voxel_cloud (new sensor_msgs::PointCloud2);
sensor_msgs::PointCloud2::Ptr passthrough_cloud (new sensor_msgs::PointCloud2);
sensor_msgs::PointCloud2::Ptr segmentation_cloud (new sensor_msgs::PointCloud2);
sensor_msgs::PointCloud2::Ptr cluster_cloud (new sensor_msgs::PointCloud2);
sensor_msgs::PointCloud2::Ptr outlier_cloud (new sensor_msgs::PointCloud2);
std::vector<sensor_msgs::PointCloud2::Ptr> pc2_clusters;

void callback(const PointCloud::ConstPtr& msg)
{

  pc2_clusters.clear();

  std::string world_frame="camera_depth_optical_frame";
  ROS_INFO_STREAM("Original cloud had " << msg->size() << " points");

  //1. voxel grid filter
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>(*msg));
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_voxel_filtered(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
  voxel_filter.setInputCloud(cloud_ptr);
  voxel_filter.setLeafSize(float(0.005), float(0.005), float(0.005)); //In Meters
  voxel_filter.filter(*cloud_voxel_filtered);
  ROS_INFO_STREAM("DownSampled cloud with " <<cloud_voxel_filtered->size()<< "points");


  //2. passthrough filter
  pcl::PointCloud<pcl::PointXYZ>xf_cloud, yf_cloud, zf_cloud;

  //filter in x
  pcl::PassThrough<pcl::PointXYZ> pass_x;
  pass_x.setInputCloud(cloud_voxel_filtered);
  pass_x.setFilterFieldName("x");
  pass_x.setFilterLimits(-0.5,0.5); //-0.25 i 1.5 //Lateral
  pass_x.filter(xf_cloud);

  //filter in y
  pcl::PointCloud<pcl::PointXYZ>::Ptr xf_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>(xf_cloud));
  pcl::PassThrough<pcl::PointXYZ> pass_y;
  pass_y.setInputCloud(xf_cloud_ptr);
  pass_y.setFilterFieldName("y");
  pass_y.setFilterLimits(-0.5,0.3); //-0.5 i 0.3 //Amunt
  pass_y.filter(yf_cloud);

    //filter in z
  pcl::PointCloud<pcl::PointXYZ>::Ptr yf_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>(yf_cloud));
  pcl::PassThrough<pcl::PointXYZ> pass_z;
  pass_z.setInputCloud(yf_cloud_ptr);
  pass_z.setFilterFieldName("z");
  pass_z.setFilterLimits(0.3,1.3); //0.6 i 1.5 //Endavant
  pass_z.filter(zf_cloud);

  ROS_INFO_STREAM("DownSampled Passthrough filter with "<<zf_cloud.size()<<" points");

  //3. plane segmentation
  pcl::PointCloud<pcl::PointXYZ>::Ptr cropped_cloud(new pcl::PointCloud<pcl::PointXYZ>(zf_cloud));
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ>);

  //Create the segmentation object for the planar model and set all the parameters
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations(400);
  seg.setDistanceThreshold(0.008);

  //Segment the largest planar component from the cropped cloud
  seg.setInputCloud(cropped_cloud);
  seg.segment(*inliers, *coefficients);
  if (inliers->indices.size() == 0) {
    ROS_WARN_STREAM("Could not estimate a planar model for the given dataset.");
  }

  //Extract the planar inliers from the input cloud
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  extract.setInputCloud(cropped_cloud);
  extract.setIndices(inliers);
  extract.setNegative(false);

  //Get the points associated with the planar surface
  extract.filter(*cloud_plane);
  ROS_INFO_STREAM("PointCloud representing the planar component: " << cloud_plane->points.size() << "data points.");

  //Remove the planar inliers, extract the rest
  extract.setNegative(true);
  extract.filter(*cloud_f);


  //4. euclidean cluster extraction 
  pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_cloud(new pcl::PointCloud<pcl::PointXYZ>);

  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
  *cloud_filtered = *cloud_f;
  tree->setInputCloud(cloud_filtered);
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance(0.02); //S'ha de tocar ja que per això a l'ho millor tinc 2 cluster en compte 1. Està en metres
  ec.setMinClusterSize(100);
  ec.setMaxClusterSize(10000);
  ec.setSearchMethod(tree);
  ec.setInputCloud(cloud_filtered);
  ec.extract(cluster_indices);

  ros::NodeHandle nh;

  //Create a publisher for each cluster
  for (int i=0; i<cluster_indices.size(); ++i) {
    std::string topicName = "/cluster_" + boost::lexical_cast<std::string>(i);
    ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2> (topicName, 1);
    pub_vec.push_back(pub);
    }

  //Calculate de biggest cluster
  n_cluster = 0;
  big_cluster = 0;
  cluster_size = 0;

 for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
    
    for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); pit++)
      cloud_cluster->points.push_back(cloud_filtered->points[*pit]);

    if (cloud_cluster->points.size() > cluster_size){
	
	cluster_size = cloud_cluster->points.size();
	big_cluster = n_cluster;

    }

    n_cluster++;  
  }

  std::cout << "El cluster mes gran te" << cluster_size << " punts \n";
  std::cout << "I es el cluster_" << big_cluster << "\n";
  cluster_name = "/cluster_" + boost::lexical_cast<std::string>(big_cluster);
  std::cout << cluster_name << "\n";


  std::vector<pcl::PointIndices>::const_iterator it;
  std::vector<int>::const_iterator pit;

  int j = 0;
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters;

  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
  
    for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); pit++)
      cloud_cluster->points.push_back(cloud_filtered->points[*pit]);

  cloud_cluster->width = cloud_cluster->points.size();
  cloud_cluster->height = 1;
  cloud_cluster->is_dense = true;
  std::cout << "Cluster has " << cloud_cluster->points.size() << " points.\n";
  clusters.push_back(cloud_cluster);
  
  sensor_msgs::PointCloud2::Ptr tempROSMsg(new sensor_msgs::PointCloud2);
  pcl::toROSMsg(*cloud_cluster, *tempROSMsg);
  pc2_clusters.push_back(tempROSMsg);
  tempROSMsg->header.frame_id = world_frame;
  tempROSMsg-> header.stamp = ros::Time::now();
  pub_vec[j].publish(tempROSMsg);
  ++j;

  }


  //5. statical outlier removal
  pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
  *cluster_cloud_ptr = *cloud_f;
  pcl::PointCloud<pcl::PointXYZ>::Ptr sor_cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

  pcl::StatisticalOutlierRemoval<pcl::PointXYZ>sor;
  sor.setInputCloud(cluster_cloud_ptr);
  sor.setMeanK(100);
  sor.setStddevMulThresh(1.0);
  sor.filter(*sor_cloud_filtered);

  ROS_INFO_STREAM("Removal statical outlier: "<<sor_cloud_filtered->points.size()<<"data points.");

  //6. broadcast transform
  for(int i=0; i<j; i++){
    pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_vec= clusters.at(i);

    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*cluster_vec, centroid);
    Eigen::Vector3f vec3 = centroid.head<3>();

    //Broadcast transform
    static tf::TransformBroadcaster br;
    tf::Transform part_transform;

    part_transform.setOrigin(tf::Vector3(vec3(0),vec3(1),vec3(2)));
    tf::Quaternion q;
    q.setRPY(0 ,0, 0);
    part_transform.setRotation(q);

    std::stringstream ss;
    ss << "cluster_" << i;

    ROS_INFO_STREAM(vec3);

    br.sendTransform(tf::StampedTransform(part_transform, ros::Time::now(), world_frame, ss.str()));
    
  }

  //Convert point cloud PCL->ROS
  pcl::toROSMsg(*msg, *original_cloud);
  original_cloud->header.frame_id=world_frame;
  original_cloud->header.stamp=ros::Time::now();

  //voxel 
  pcl::toROSMsg(*cloud_voxel_filtered, *voxel_cloud);
  voxel_cloud->header.frame_id=world_frame;
  voxel_cloud->header.stamp=ros::Time::now();

  //passthrough 
  pcl::toROSMsg(zf_cloud, *passthrough_cloud);
  passthrough_cloud->header.frame_id=world_frame;
  passthrough_cloud->header.stamp=ros::Time::now();

  //segmentation 
  pcl::toROSMsg(*cloud_f,*segmentation_cloud);
  segmentation_cloud->header.frame_id = world_frame;
  segmentation_cloud->header.stamp = ros::Time::now();

  //outlier
  pcl::toROSMsg(*sor_cloud_filtered,*outlier_cloud);
  outlier_cloud->header.frame_id = world_frame;
  outlier_cloud->header.stamp = ros::Time::now();

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "perception_cluster");
  ros::NodeHandle nh;

  //Subscribe to the kinect point cloud topic
  ros::Subscriber sub = nh.subscribe<PointCloud>("/camera/depth/points", 1, callback);

  //Create a ROS publisher for the output point cloud
  ros::Publisher pub_original = nh.advertise<PointCloud>("original_cloud", 1);
  //voxel publisher
  ros::Publisher pub_voxel = nh.advertise<PointCloud>("voxel_filter", 1);
  //passthrough publisher
  ros::Publisher pub_passthrough = nh.advertise<PointCloud>("passthrough_filter", 1);
  //segmentation publisher
  ros::Publisher pub_segmentation = nh.advertise<PointCloud>("segmentation_filter", 1);
  //outlier removal publisher
  ros::Publisher pub_outlier = nh.advertise<PointCloud>("outlier_removal", 1);

  //Biggest cluster publisher
  ros::Publisher pub_big_cluster = nh.advertise<std_msgs::String>("big_cluster", 100);

  ros::Rate loop_rate(10);

  while(ros::ok())
  {
    //Publish the original point cloud
    pub_original.publish(original_cloud);

    //Publish the processed point cloud by the voxel filter
    pub_voxel.publish(voxel_cloud);

    //Publish the processed point cloud by the passthrough filter
    pub_passthrough.publish(passthrough_cloud);
    
    //Publish the processed point cloud by the plane segmentation
    pub_segmentation.publish(segmentation_cloud);

    //Publish the processed point cloud by the statical outlier removal
    pub_outlier.publish(outlier_cloud);

    //Publish the biggest cluster
    std_msgs::String msg;
    std::stringstream ss;
    ss << cluster_name;
    msg.data = ss.str();
    pub_big_cluster.publish(msg);

    ros::spinOnce();
    loop_rate.sleep();
  }

  ros::spin();

return 0;
}
