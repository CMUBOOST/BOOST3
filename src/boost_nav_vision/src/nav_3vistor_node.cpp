#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/shadowpoints.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>

#include "pcl_ros/point_cloud.h"

#include <pcl/filters/statistical_outlier_removal.h>

#include <boost_nav_vision/Heading.h>

ros::Publisher pub1;
ros::Publisher pub2;
ros::Publisher pub3;
ros::Publisher marker_pub;
ros::Publisher twist_pub;
ros::Publisher heading_pub;

geometry_msgs::Twist twist_msg;

boost_nav_vision::Heading headingMsg;

typedef pcl::PointXYZ PointT;

const double forwardVel = 0.5;
const double turnGain = 4.0;

void driveStraight()
{
  twist_msg.linear.x = forwardVel;
  twist_msg.angular.z = 0.0;
  twist_pub.publish(twist_msg);
  ROS_INFO_STREAM("Driving straight!");
}

void driveAdjust(float crosstrackAdj, float headingAdj)
{
  // Just use crosstrack error for now, include heading error at some point
  float zAdj = crosstrackAdj * turnGain;
  twist_msg.linear.x = forwardVel;
  twist_msg.angular.z = zAdj;
  twist_pub.publish(twist_msg);
  ROS_INFO_STREAM("Adjusting direction!");
}

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{

  // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
  pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
  pcl::fromROSMsg (*cloud_msg, *cloud);
  // pcl::PointCloud<pcl::PointXYZ> output_cloud;  // Used to output inliers  

  // All the objects needed
  pcl::PassThrough<PointT> pass;
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  
  pcl::NormalEstimation<PointT, pcl::Normal> ne;
  pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg , seg_row1, seg_row2; 
  
  pcl::ExtractIndices<PointT> extract, extract_row1, extract_row2;
  pcl::ExtractIndices<pcl::Normal> extract_normals, extract_normals_row1, extract_normals_row2;
  
  pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());

  // // Datasets
  pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>), ground_filtered (new pcl::PointCloud<PointT>), row_filtered1 (new pcl::PointCloud<PointT>), row_filtered2 (new pcl::PointCloud<PointT>);
  
  pcl::PointCloud<pcl::Normal>::Ptr temp_normals (new pcl::PointCloud<pcl::Normal>), normals_row1 (new pcl::PointCloud<pcl::Normal>), normals_row2 (new pcl::PointCloud<pcl::Normal>);

  pcl::ModelCoefficients::Ptr coefficients_ground_plane (new pcl::ModelCoefficients), coefficients_row1_plane (new pcl::ModelCoefficients), coefficients_row2_plane (new pcl::ModelCoefficients);

  pcl::PointIndices::Ptr inliers_ground (new pcl::PointIndices), inliers_row1 (new pcl::PointIndices), inliers_row2 (new pcl::PointIndices);

  // Read in the cloud data
  ROS_DEBUG_STREAM("PointCloud has: " << cloud->points.size () << " data points.");

  // Build a passthrough filter
  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0.4, 5.0);
  pass.filter (*cloud_filtered);
  ROS_DEBUG_STREAM("PointCloud after Z filtering has: " << cloud_filtered->points.size () << " data points.");

  // Build a passthrough filter
  pass.setInputCloud (cloud_filtered);
  pass.setFilterFieldName ("y");
  pass.setFilterLimits (-0.05, 0.5);
  pass.filter (*cloud_filtered);
  ROS_DEBUG_STREAM("PointCloud after Y filtering has: " << cloud_filtered->points.size () << " data points.");

  // Build a passthrough filter
  pass.setInputCloud (cloud_filtered);
  pass.setFilterFieldName ("x");
  pass.setFilterLimits (-0.75, 0.75);
  pass.filter (*cloud_filtered);
  ROS_DEBUG_STREAM("PointCloud after filtering has: " << cloud_filtered->points.size () << " data points.");

  // Make sure points in cloud after filtering
  if (cloud_filtered->points.empty())
    {
    ROS_WARN_STREAM("No points after initial filtering!");
    driveStraight();
    return;
    }

  // Build statistical outlier removal filter
  sor.setInputCloud (cloud_filtered);
  sor.setMeanK (20);  // number of neighbors to calculate
  sor.setStddevMulThresh (1.0);  // standard deviation multiplier outside which a point is  considered an outlier 
  sor.filter (*cloud_filtered);

  // Estimate point normals
  ne.setSearchMethod (tree);
  ne.setInputCloud (cloud_filtered);
  ne.setKSearch (50);
  ne.compute (*temp_normals);

  // Ground Plane //
  // Create the segmentation object for the planar model and set all the parameters
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_NORMAL_PLANE);
  seg.setNormalDistanceWeight (0.1);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (100);
  seg.setDistanceThreshold (0.15);
  seg.setInputCloud (cloud_filtered);
  seg.setInputNormals (temp_normals);
  // Obtain the plane inliers and coefficients
  seg.segment (*inliers_ground, *coefficients_ground_plane);
  // ROS_INFO_STREAM("Ground plane coefficients: " << *coefficients_ground_plane);

  // Extract the planar inliers from the input cloud
  extract.setInputCloud (cloud_filtered);
  extract.setIndices (inliers_ground);
  extract.setNegative (false);

  // Write the planar inliers to other cloud
  extract.filter (*ground_filtered);
  ROS_DEBUG_STREAM("PointCloud representing the ground plane: " << ground_filtered->points.size () << " data points.");

  if (ground_filtered->points.empty())
    {
    ROS_WARN_STREAM("No points in ground plane!");
    driveStraight();
    return;
    }

  // Remove the planar inliers, extract the rest
  extract.setNegative (true);
  extract.filter (*row_filtered1);  // remove ground plane from filtered point cloud
  extract_normals.setNegative (true);
  extract_normals.setInputCloud (temp_normals);
  extract_normals.setIndices (inliers_ground);
  extract_normals.filter (*temp_normals);

  // Filter for the right side
  pass.setInputCloud (row_filtered1);
  pass.setFilterFieldName ("x");
  pass.setFilterLimits (0, 0.75);
  pass.filter (*row_filtered2);

  if (row_filtered2->points.size() < 500)
  {
    ROS_WARN_STREAM("Not enough points in right row!");
    driveStraight();
    return;
  }

    // Filter for the left side
  pass.setInputCloud (row_filtered1);
  pass.setFilterFieldName ("x");
  pass.setFilterLimits (-0.75, 0);
  pass.filter (*row_filtered1);

  if (row_filtered1->points.size() < 500)
  {
    ROS_WARN_STREAM("Not enough points in left row!");
    driveStraight();
    return;
  }

  // Row 1 //
  // Estimate point normals (INEFFICIENT, USE SINGLE ESTIMATION IN FUTURE)
  ne.setSearchMethod (tree);
  ne.setInputCloud (row_filtered1);
  ne.setKSearch (50);
  ne.compute (*normals_row1);

  // Create the segmentation object for the planar model and set all the parameters
  seg_row1.setOptimizeCoefficients (true);
  seg_row1.setModelType (pcl::SACMODEL_NORMAL_PLANE);
  seg_row1.setNormalDistanceWeight (0.1);
  seg_row1.setMethodType (pcl::SAC_RANSAC);
  seg_row1.setMaxIterations (100);
  seg_row1.setDistanceThreshold (0.05);
  seg_row1.setInputCloud (row_filtered1);
  seg_row1.setInputNormals (normals_row1);
  // Obtain the plane inliers and coefficients
  seg_row1.segment (*inliers_row1, *coefficients_row1_plane);
  // ROS_INFO_STREAM("Row 1 plane coefficients: " << *coefficients_row1_plane);

  // Extract the planar inliers from the input cloud
  extract_row1.setInputCloud (row_filtered1);
  extract_row1.setIndices (inliers_row1);
  extract_row1.setNegative (false);

  // Extract the planar inliers
  extract_row1.filter (*row_filtered1);
  ROS_DEBUG_STREAM("PointCloud representing the left planar component: " << row_filtered1->points.size () << " data points.");


  // Row 2 //
  // Estimate point normals (INEFFICIENT, USE SINGLE ESTIMATION IN FUTURE)
  ne.setSearchMethod (tree);
  ne.setInputCloud (row_filtered2);
  ne.setKSearch (50);
  ne.compute (*normals_row2);

  // Create the segmentation object for the planar model and set all the parameters
  seg_row2.setOptimizeCoefficients (true);
  seg_row2.setModelType (pcl::SACMODEL_NORMAL_PLANE);
  seg_row2.setNormalDistanceWeight (0.1);
  seg_row2.setMethodType (pcl::SAC_RANSAC);
  seg_row2.setMaxIterations (100);
  seg_row2.setDistanceThreshold (0.05);
  seg_row2.setInputCloud (row_filtered2);
  seg_row2.setInputNormals (normals_row2);
  // Obtain the plane inliers and coefficients
  seg_row2.segment (*inliers_row2, *coefficients_row2_plane);
  // ROS_INFO_STREAM("Row 1 plane coefficients: " << *coefficients_row2_plane);

  // Extract the planar inliers from the input cloud
  extract_row2.setInputCloud (row_filtered2);
  extract_row2.setIndices (inliers_row2);
  extract_row2.setNegative (false);

  // Extract the planar inliers
  extract_row2.filter (*row_filtered2);
  ROS_DEBUG_STREAM("PointCloud representing the right planar component: " << row_filtered2->points.size () << " data points.");


  // Calculate perpendicular distance to origin from each plane (plane coefficients are norm 1!)
  float dl = fabs(coefficients_row1_plane->values[3]);
  float dr = fabs(coefficients_row2_plane->values[3]);

  float deltaD = dl - dr;

  ROS_DEBUG_STREAM("Distance to left plane is: " << dl);
  ROS_DEBUG_STREAM("Distance to right plane is: " << dr);
  ROS_INFO_STREAM("Delta D is: " << deltaD);

  // Heading calculation
  float headingE = acosf((coefficients_row1_plane->values[0] + coefficients_row2_plane->values[0])/2);
  headingE = headingE - 0.1; // Adjusting for incorrect alignment of sensor, HACK!

  // Publish heading error
  headingMsg.crosstrackError = deltaD;
  headingMsg.headingError = headingE;  
  heading_pub.publish(headingMsg);

  // float normRow1 = sqrtf(
  //                 powf(coefficients_row1_plane->values[0], 2) + 
  //                 powf(coefficients_row1_plane->values[1], 2) + 
  //                 powf(coefficients_row1_plane->values[2], 2));
  // ROS_INFO_STREAM("Norm of left row: " << normRow1);

  // Check that detected rows are roughly parallel to x-axis
  if (coefficients_row1_plane->values[0] < 0.9 || coefficients_row2_plane->values[0] < 0.9)
  {
    ROS_WARN_STREAM("Detected rows are outside range for row-following!");
    driveStraight();
    return;
  }

  driveAdjust(deltaD, headingE);

  // Publish marker indicating direction robot should move
  visualization_msgs::Marker arrow;
  arrow.header.frame_id = "/camera_tof";
  arrow.header.stamp = ros::Time::now();
  arrow.ns = "camera_line";
  arrow.action = visualization_msgs::Marker::ADD;
  arrow.pose.orientation.w = 1.0;
  arrow.id = 0;
  arrow.type = visualization_msgs::Marker::ARROW;
  arrow.scale.x = 0.01;
  arrow.scale.y = 0.1;
  arrow.color.g = 1.0;
  arrow.color.a = 1.0;

  geometry_msgs::Point p;
  p.x = 0; p.y = 0; p.z = 0;
  arrow.points.push_back(p);
  p.x = deltaD*0.5; p.y = 0; p.z = 1.0;
  arrow.points.push_back(p);  
  marker_pub.publish(arrow);

  // // Create the segmentation object for cylinder segmentation and set all the parameters
  // seg.setOptimizeCoefficients (true);
  // seg.setModelType (pcl::SACMODEL_CYLINDER);
  // seg.setMethodType (pcl::SAC_RANSAC);
  // seg.setNormalDistanceWeight (0.1);
  // seg.setMaxIterations (10000);
  // seg.setDistanceThreshold (0.05);
  // seg.setRadiusLimits (0, 0.1);
  // seg.setInputCloud (row_filtered1);
  // seg.setInputNormals (cloud_normals2);

  // // Obtain the cylinder inliers and coefficients
  // seg.segment (*inliers_cylinder, *coefficients_cylinder);
  // std::cerr << "Cylinder coefficients: " << *coefficients_cylinder << std::endl;

  // // Write the cylinder inliers to disk
  // extract.setInputCloud (row_filtered1);
  // extract.setIndices (inliers_cylinder);
  // extract.setNegative (false);
  // pcl::PointCloud<PointT>::Ptr cloud_cylinder (new pcl::PointCloud<PointT> ());
  // extract.filter (*cloud_cylinder);
  // if (cloud_cylinder->points.empty ()) 
  //   std::cerr << "Can't find the cylindrical component." << std::endl;
  // else
  // {
  //   std::cerr << "PointCloud representing the cylindrical component: " << cloud_cylinder->points.size () << " data points." << std::endl;
  //   writer.write ("table_scene_mug_stereo_textured_cylinder.pcd", *cloud_cylinder, false);
  // }


  // Convert to ROS data type
  // sensor_msgs::PointCloud2 output; 
  // pcl_conversions::moveFromPCL(*cloud_filtered, output);

  // Print important information
  ROS_DEBUG_STREAM("Ground plane coefficients: \n" << *coefficients_ground_plane);
  ROS_DEBUG_STREAM("Row 1 plane coefficients: \n" << *coefficients_row1_plane);
  ROS_DEBUG_STREAM("Row 2 plane coefficients: \n" << *coefficients_row2_plane);

  // Publish the data
  pub1.publish (*ground_filtered);
  pub2.publish (*row_filtered1);
  pub3.publish (*row_filtered2);


  // // Create the segmentation object
  // pcl::SACSegmentation<pcl::PointXYZ> seg;
  // // Optional
  // seg.setOptimizeCoefficients (true);
  // // Mandatory
  // seg.setModelType (pcl::SACMODEL_PLANE);
  // seg.setMethodType (pcl::SAC_RANSAC);
  // seg.setDistanceThreshold (0.01);

  // seg.setInputCloud (cloud.makeShared ());
  // seg.segment (inliers, coefficients);


  // // Publish the model coefficients
  // pcl_msgs::ModelCoefficients ros_coefficients;
  // pcl_conversions::fromPCL(coefficients, ros_coefficients);
  // // pub.publish (ros_coefficients);



/*  Voxel grid
  // Convert PointCloud2 to PointXYZ
  pcl::PCLPointCloud2 pcl_pc2;
  pcl_conversions::toPCL(*cloud_msg, pcl_pc2);
  pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromPCLPointCloud2(pcl_pc2, *temp_cloud);

  // Container for original & filtered data
  pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2; 
  pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
  // pcl::PCLPointCloud2 cloud_filtered;

  // Convert to PCL data type
  pcl_conversions::toPCL(*cloud_msg, *cloud);

  // // Perform the actual filtering
  // pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  // sor.setInputCloud (cloudPtr);
  // sor.setLeafSize (0.01, 0.01, 0.01);
  // sor.filter (cloud_filtered);


  // Convert to ROS data type
  sensor_msgs::PointCloud2 output; 
  pcl_conversions::moveFromPCL(cloud_filtered, output);

  // Publish the data
  pub.publish (output);

  // Print info about the clouds to the screen
  ROS_INFO_STREAM("Input cloud: " << cloud->width * cloud->height 
       << " data points with (" << pcl::getFieldsList (*cloud) << ") attributes.");

  ROS_INFO_STREAM("Output cloud: " << cloud_filtered.width * cloud_filtered.height   
       << " data points with (" << pcl::getFieldsList (cloud_filtered) << ") attributes."); 
*/


/*  Old code
  // Create a container for the data.
  sensor_msgs::PointCloud2 output;

  // Do data processing here...
  output = *input;6

  // Publish the data.
  pub.publish (output);
*/
}

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "nav_3vistor_node");
  ros::NodeHandle nh;

  // Create a ROS publisher for the output point cloud
  marker_pub = nh.advertise<visualization_msgs::Marker>("/visualization_marker", 10);
  twist_pub = nh.advertise<geometry_msgs::Twist> ("/sickToF/nav_vel", 1);
  heading_pub = nh.advertise<boost_nav_vision::Heading> ("/headingError", 1);

  pub1 = nh.advertise<pcl::PointCloud<pcl::PointXYZ> > ("/sickToF/groundPlane", 1);
  pub2 = nh.advertise<pcl::PointCloud<pcl::PointXYZ> > ("/sickToF/leftRowPlane", 1);
  pub3 = nh.advertise<pcl::PointCloud<pcl::PointXYZ> > ("/sickToF/rightRowPlane", 1);

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("/sick_3vistort_driver/points", 1, cloud_cb);


  ros::Rate r(10);
  while (ros::ok())
  {

    r.sleep();

    // Spin
    ros::spinOnce ();
  }





}