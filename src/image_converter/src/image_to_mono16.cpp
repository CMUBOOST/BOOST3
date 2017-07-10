#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Image.h"
#include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"

cv::Mat depth_mono8_img;

void depthCallback(const sensor_msgs::Image::ConstPtr& original_image)
{
    cv_bridge::CvImagePtr cv_ptr;
    //Convert from the ROS image message to a CvImage suitable for working with OpenCV for processing
    try
    {
        //Always copy, returning a mutable CvImage
        //OpenCV expects color images to use BGR channel order.
        cv_ptr = cv_bridge::toCvCopy(original_image);
    }
    catch (cv_bridge::Exception& e)
    {
        //if there is an error during conversion, display it
        ROS_ERROR("tutorialROSOpenCV::main.cpp::cv_bridge exception: %s", e.what());
        return;
    }

    //Copy the image.data to imageBuf.
    cv::Mat depth_float_img = cv_ptr->image;
    depthToCV8UC1(depth_float_img, depth_mono8_img);
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "image_to_mono8_node");
    
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);     
    image_transport::Subscriber depth_sub = it.subscribe("/sick_3vistort_driver/depth", 1, depthCallback);
    image_transport::Publisher image_pub = it.advertise("mono8_image", 1);
    
    ros::spinOnce();

    ros::Rate r(60.0);
    while (ros::ok()) {
        sensor_msgs::Image ros_img;
        depth_mono8_img.toImageMsg(ros_img);        
        image_pub.publish(ros_img);
        
        
        
        ros::spinOnce();
        r.sleep();
    }
    
    return 0;
}
