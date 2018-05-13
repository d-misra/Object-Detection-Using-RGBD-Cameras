

#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/PointCloud2.h>

// PCL specific includes
#include <pcl_conversions/pcl_conversions.h>
#include "pcl_ros/transforms.h"

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/statistical_outlier_removal.h>

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "perception_node");
  ros::NodeHandle nh;
  ros::NodeHandle priv_nh_("~");

  std::string cloud_topic, world_frame, camera_frame;

  /*If want to test with a static scene, load the PCD file in rviz

  roscore
  rosrun tf2_ros static_transform_publisher 0 0 0 0 0 0 world_frame kinect_link
  rosrun pcl_ros pcd_to_pointcloud pcd_file.pcd 0.1 _frame_id:=kinect_link cloud_pcd:=kinect/depth_registered/points
  rosrun rviz rviz

  And uncomment:

  world_frame="kinect_link";
  camera_frame="kinect_link";
  cloud_topic="kinect/depth_registered/points";

  */

  world_frame="camera_link";
  camera_frame="camera_link";
  cloud_topic="camera/depth_registered/points";

  ros::Publisher object_pub, object_pub2, object_pub3, cluster_pub, pose_pub;
  object_pub = nh.advertise<sensor_msgs::PointCloud2>("voxel_filtered", 1);
  object_pub2 = nh.advertise<sensor_msgs::PointCloud2>("pass_filtered", 1);
  object_pub3 = nh.advertise<sensor_msgs::PointCloud2>("plane_segmented", 1);
  cluster_pub = nh.advertise<sensor_msgs::PointCloud2>("euclidean_cluster", 1);

 while (ros::ok())
 {

  std::string topic = nh.resolveName(cloud_topic);
  ROS_INFO_STREAM("Cloud service called; waiting for a PointCloud2 on topic "<< topic);
  sensor_msgs::PointCloud2::ConstPtr recent_cloud =
               ros::topic::waitForMessage<sensor_msgs::PointCloud2>(topic, nh);

  tf::TransformListener listener;
  tf::StampedTransform stransform;
  try
  {
    listener.waitForTransform(world_frame, recent_cloud->header.frame_id,  ros::Time::now(), ros::Duration(6.0));
    listener.lookupTransform(world_frame, recent_cloud->header.frame_id,  ros::Time(0), stransform);
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("%s",ex.what());
  }
  sensor_msgs::PointCloud2 transformed_cloud;
//  sensor_msgs::PointCloud2::ConstPtr recent_cloud =
//               ros::topic::waitForMessage<sensor_msgs::PointCloud2>(topic, nh);
  pcl_ros::transformPointCloud(world_frame, stransform, *recent_cloud, transformed_cloud);


  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromROSMsg (transformed_cloud, cloud);

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr (new pcl::PointCloud<pcl::PointXYZ> (cloud));
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_voxel_filtered (new pcl::PointCloud<pcl::PointXYZ> ());
	pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
	voxel_filter.setInputCloud (cloud_ptr);
	//voxel_filter.setLeafSize (float(0.01), float(0.01), float(0.01));
        voxel_filter.setLeafSize (float(0.01), float(0.01), float(0.01));
	voxel_filter.filter (*cloud_voxel_filtered);

  ROS_INFO_STREAM("Original cloud  had " << cloud_ptr->size() << " points");
  ROS_INFO_STREAM("Downsampled cloud  with " << cloud_voxel_filtered->size() << " points");


  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());

  // Create the segmentation object for the planar model and set all the parameters
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (200);
  seg.setDistanceThreshold (0.004);

  // Segment the largest planar component from the cropped cloud
  // seg.setInputCloud (cropped_cloud);
  seg.setInputCloud (cloud_voxel_filtered); //cloud_voxel_filtered is alread the downsampled cloud ptr
  seg.segment (*inliers, *coefficients);
  if (inliers->indices.size () == 0)
  {
    ROS_WARN_STREAM ("Could not estimate a planar model for the given dataset.") ;
    //break;
  }

  // Extract the planar inliers from the input cloud
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  //extract.setInputCloud (cropped_cloud);
  extract.setInputCloud (cloud_voxel_filtered);
  extract.setIndices(inliers);
  extract.setNegative (false);

  // Get the points associated with the planar surface
  extract.filter (*cloud_plane);
  ROS_INFO_STREAM("PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." );

  // Remove the planar inliers, extract the rest
  extract.setNegative (true);
  extract.filter (*cloud_f);

  //Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  *cloud_filtered = *cloud_f;
  tree->setInputCloud (cloud_filtered); //remains after plane segmentation done

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance (0.01); // 2cm
  // ec.setMinClusterSize (300);
  ec.setMinClusterSize (5);

  ec.setMaxClusterSize (50000); //10000 was default
  ec.setSearchMethod (tree);
  ec.setInputCloud (cloud_filtered);
  ec.extract (cluster_indices);

  std::vector<sensor_msgs::PointCloud2::Ptr> pc2_clusters;
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr > clusters;
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
      cloud_cluster->points.push_back(cloud_filtered->points[*pit]);
    cloud_cluster->width = cloud_cluster->points.size ();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;
    std::cout << "Cluster has " << cloud_cluster->points.size() << " points.\n";
    clusters.push_back(cloud_cluster);
    sensor_msgs::PointCloud2::Ptr tempROSMsg(new sensor_msgs::PointCloud2);
    pcl::toROSMsg(*cloud_cluster, *tempROSMsg);
    pc2_clusters.push_back(tempROSMsg);
  }

   //Centroid computation of clustered objects
   Eigen::Vector4f centroid;
   pcl::compute3DCentroid(*(clusters.at(0)),centroid);

   double x, y, z;
   x = centroid[0];
   y = centroid[1];
   z = centroid[2];

   double distance = sqrt(x*x + y*y + z*z);

   std::cout << "most dense cluster has ::" << clusters.at(0)->points.size() << "points\n";
   std::cout << "centroid of most dense object is::" << x << "," << y << "," << z << "\n";
   std::cout << "distance from object::" << distance << "\n";


  sensor_msgs::PointCloud2::Ptr pc2_cloud (new sensor_msgs::PointCloud2);
  sensor_msgs::PointCloud2::Ptr pc3_cloud (new sensor_msgs::PointCloud2);
  sensor_msgs::PointCloud2::Ptr pc4_cloud (new sensor_msgs::PointCloud2);
  sensor_msgs::PointCloud2::Ptr pc5_cloud (new sensor_msgs::PointCloud2);

  pcl::toROSMsg(*cloud_voxel_filtered, *pc2_cloud); //voxel filtered
  pc2_cloud->header.frame_id=world_frame;
  pc2_cloud->header.stamp=ros::Time::now();
  object_pub.publish(pc2_cloud);

  pcl::toROSMsg(*cloud_f, *pc4_cloud); //plane segmentation
  pc4_cloud->header.frame_id=world_frame;
  pc4_cloud->header.stamp=ros::Time::now();
  object_pub3.publish(pc4_cloud);

  pcl::toROSMsg(*(clusters.at(0)), *pc5_cloud); //euclidean clustering
  pc5_cloud->header.frame_id=world_frame;
  pc5_cloud->header.stamp=ros::Time::now();
  cluster_pub.publish(pc5_cloud);

  pcl::toROSMsg(*(clusters.at(1)), *pc5_cloud); //euclidean clustering
  pc5_cloud->header.frame_id=world_frame;
  pc5_cloud->header.stamp=ros::Time::now();
  cluster_pub.publish(pc5_cloud);

  pcl::toROSMsg(*(clusters.at(2)), *pc5_cloud); //euclidean clustering
  pc5_cloud->header.frame_id=world_frame;
  pc5_cloud->header.stamp=ros::Time::now();
  cluster_pub.publish(pc5_cloud);

   }
  return 0;
}
