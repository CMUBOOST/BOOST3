#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
//#include <geometry_msgs/PointStamped.h>
#include <tf2_ros/transform_broadcaster.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl/common/common.h>
#include <pcl_conversions/pcl_conversions.h>
//#include <pcl/filters/passthrough.h>
//#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/features/normal_3d.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
//#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/distances.h>
//#include <pcl/filters/statistical_outlier_removal.h>
//#include <pcl/filters/radius_outlier_removal.h>
#include <log4cxx/logger.h>

typedef pcl::PointXYZ PointT;

class PlaneSegmentation
{
    ros::NodeHandle nh;
    ros::Subscriber sub_pointcloud;
    ros::Publisher pub_pointcloud, pub_pointcloud_plane;

    tf2_ros::TransformBroadcaster tfb;
    geometry_msgs::TransformStamped transformStamped;

    public:
        PlaneSegmentation()
        {
            sub_pointcloud = nh.subscribe("/assembled_cloud2",1,&PlaneSegmentation::pointCloudCb, this);
            pub_pointcloud = nh.advertise<sensor_msgs::PointCloud2>("/assembled_cloud2/points_filtered", 1);
            pub_pointcloud_plane = nh.advertise<sensor_msgs::PointCloud2>("/assembled_cloud2/points_groundplane", 1);
        }

        ~PlaneSegmentation()
        {}


        void extractplane(pcl::PointCloud<PointT>::Ptr cloud_filtered)
        {
            Eigen::Vector3f axis = Eigen::Vector3f(0.0,1.0,0.0);  //this sets an axis that the plane is normal to

            pcl::PointCloud<PointT>::Ptr cloud_plane(new pcl::PointCloud<PointT>);

            pcl::NormalEstimation<PointT, pcl::Normal> ne;
            pcl::ExtractIndices<PointT> extract;
            pcl::ModelCoefficients::Ptr coefficients_plane (new pcl::ModelCoefficients);
            pcl::PointIndices::Ptr inliers_plane (new pcl::PointIndices ());
            pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg;
            pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());
            pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

            ne.setSearchMethod (tree);
            ne.setInputCloud (cloud_filtered);
            ne.setKSearch (50);  //This sets the number of neighbors that inform the point normal
            ne.compute (*cloud_normals);

            seg.setOptimizeCoefficients (true);
            seg.setModelType (pcl::SACMODEL_NORMAL_PLANE);
            seg.setMethodType (pcl::SAC_RANSAC);
            seg.setAxis(axis);
            seg.setEpsAngle(5.0f * (M_PI/180.0f));  //This sets a threshold on the angle of the plane. You might want to make this tighter.
            seg.setNormalDistanceWeight (0.1);  //This corresponds to how much the point normals are weighted versus their location when defining a plane
            seg.setMaxIterations (1000);    //You might want to play around with the number of iterations
            seg.setDistanceThreshold (.2);  //This is a threshold of 0.2 meters - I'm not sure if it's in one direction or both
            //seg.setRadiusLimits (.010, .025);
            //seg.setRadiusLimits (10, 40);
            seg.setInputCloud (cloud_filtered);
            seg.setInputNormals (cloud_normals);
            seg.segment (*inliers_plane, *coefficients_plane);

            extract.setInputCloud (cloud_filtered);
            extract.setIndices (inliers_plane);
            extract.setNegative (true);  //This sets the inliers_plane cloud to consist of points NOT on the plane (ie just the plants)
            extract.filter (*cloud_plane);

            int numpoints_inliers = inliers_plane->indices.size();
            ROS_DEBUG_STREAM("inliers_plane is: " << numpoints_inliers);

            sensor_msgs::PointCloud2 msg_pub1;

            pcl::toROSMsg(*cloud_plane, msg_pub1);

            pub_pointcloud_plane.publish(msg_pub1);
        }


        void pointCloudCb(const sensor_msgs::PointCloud2ConstPtr& msg_sub)
        {
            pcl::PointCloud<PointT>::Ptr cloud_in(new pcl::PointCloud<PointT>);
            pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);
            pcl::fromROSMsg(*msg_sub, *cloud_in);

            std::vector<int> indices;
            pcl::removeNaNFromPointCloud(*cloud_in, *cloud_in, indices);

            int numpoints_original = cloud_in->points.size();
            ROS_DEBUG_STREAM("Original: "  << numpoints_original);

	    
            //*********PASSTHROUGH FILTER***************//
            // Create the filtering object
            /*
 	    pcl::PassThrough<PointT> pass;
            pass.setInputCloud (cloud_in);
            pass.setFilterFieldName ("x");
            pass.setFilterLimits (-35, -31);  //You will need to change these limits in x once you adjust the TF
            pass.filter (*cloud_filtered);
            int numpoints_passthrough = cloud_filtered->points.size();
            ROS_DEBUG_STREAM("Passthrough: "  << numpoints_passthrough);
	    */	

	    //*********VOXELGRID FILTER**************//
            const float voxel_grid_size = 0.1f;
            pcl::VoxelGrid<PointT> vox_grid;
            vox_grid.setInputCloud (cloud_in);
            vox_grid.setLeafSize (voxel_grid_size, voxel_grid_size, voxel_grid_size);
            //vox_grid.filter (*cloud); // Please see this http://www.pcl-developers.org/Possible-problem-in-new-VoxelGrid-implementation-from-PCL-1-5-0-td5490361.html
            pcl::PointCloud<PointT>::Ptr tempCloud (new pcl::PointCloud<PointT>);
            vox_grid.filter (*tempCloud);
            cloud_filtered = tempCloud;

            extractplane(cloud_filtered);

            sensor_msgs::PointCloud2 msg_pub;
            pcl::toROSMsg(*cloud_filtered, msg_pub);
            //msg_pub.header.frame_id = "/odom/ukf_dead_reckoning";

            pub_pointcloud.publish(msg_pub);
        }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "plane_segmentation");
    log4cxx::Logger::getLogger(ROSCONSOLE_DEFAULT_NAME)->setLevel(ros::console::g_level_lookup[ros::console::levels::Debug]);
    PlaneSegmentation plnSeg;
    ros::spin();
    return 0;
}
