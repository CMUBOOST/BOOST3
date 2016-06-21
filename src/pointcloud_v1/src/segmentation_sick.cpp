#include <ros/ros.h>
#include <iostream>
#include <string.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <sensor_msgs/PointCloud2.h>
#include <pcl/common/common.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

typedef pcl::PointXYZI PointT; 

class SegmentationSick
{
	ros::NodeHandle nh;
	ros::Subscriber sub_pointcloud;
	ros::Publisher pub_pointcloud, pub_pointcloud_plane, pub_pointcloud_notplane;

	public:
		SegmentationSick()
		{
			sub_pointcloud = nh.subscribe("/sick_3vistort_driver/points",1,&SegmentationSick::pointCloudCb, this);
			pub_pointcloud = nh.advertise<sensor_msgs::PointCloud2>("/sick_3vistort_driver/points_filtered", 1);

			pub_pointcloud_plane = nh.advertise<sensor_msgs::PointCloud2>("/sick_3vistort_driver/points_plane", 1);
			pub_pointcloud_notplane = nh.advertise<sensor_msgs::PointCloud2>("/sick_3vistort_driver/points_notplane", 1);
		}
		
		~SegmentationSick()
		{}

		void extractPlane(pcl::PointCloud<PointT>::Ptr cloud_filtered)
		{
			pcl::PointCloud<PointT>::Ptr cloud_plane(new pcl::PointCloud<PointT>), cloud_notplane(new pcl::PointCloud<PointT>);

			pcl::ExtractIndices<PointT> extract;
			pcl::ModelCoefficients::Ptr coefficients_plane (new pcl::ModelCoefficients);
			pcl::PointIndices::Ptr inliers_plane (new pcl::PointIndices ());
			pcl::SACSegmentation<PointT> seg; 

			seg.setOptimizeCoefficients (true);
			seg.setModelType (pcl::SACMODEL_PLANE);
			seg.setMethodType (pcl::SAC_RANSAC);
			seg.setMaxIterations (100000);
			seg.setDistanceThreshold (0.1);

			seg.setInputCloud (cloud_filtered);
			seg.segment (*inliers_plane, *coefficients_plane);

			extract.setInputCloud (cloud_filtered);
			extract.setIndices (inliers_plane);
			extract.setNegative (false);
			extract.filter (*cloud_plane);
			extract.setNegative (true);
			extract.filter (*cloud_notplane);

			// pcl::StatisticalOutlierRemoval<PointT> sor;
			// sor.setInputCloud (cloud_notplane);
			// sor.setMeanK (3);
			// sor.setStddevMulThresh (1);
			// sor.filter (*cloud_notplane);

			sensor_msgs::PointCloud2 msg_pub1, msg_pub2;
			pcl::toROSMsg(*cloud_plane, msg_pub1);
			pcl::toROSMsg(*cloud_notplane, msg_pub2);
			msg_pub1.header.frame_id = "/point_cloud_link";
			msg_pub2.header.frame_id = "/point_cloud_link";

			pub_pointcloud_plane.publish(msg_pub1);
			pub_pointcloud_notplane.publish(msg_pub2);

		}

		void pointCloudCb(const sensor_msgs::PointCloud2ConstPtr& msg_sub)
		{
			pcl::PointCloud<PointT>::Ptr cloud_in(new pcl::PointCloud<PointT>);
			pcl::fromROSMsg(*msg_sub, *cloud_in);

			std::vector<int> indices;
			pcl::removeNaNFromPointCloud(*cloud_in, *cloud_in, indices);
			// pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>);

			// SOR filter
			// pcl::StatisticalOutlierRemoval<PointT> sor;
			// sor.setInputCloud (cloud_in);
			// sor.setMeanK (3);
			// sor.setStddevMulThresh (1.0);
			// sor.filter (*cloud_filtered);

			// extractPlane(cloud_filtered);
			extractPlane(cloud_in);

			sensor_msgs::PointCloud2 msg_pub;
			// pcl::toROSMsg(*cloud_filtered, msg_pub);
			pcl::toROSMsg(*cloud_in, msg_pub);			
			msg_pub.header.frame_id = "/point_cloud_link";
			  	  		
			pub_pointcloud.publish(msg_pub);
		}
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "segmentation_sick");
	SegmentationSick segSick;
	ros::spin();
	return 0;
}
