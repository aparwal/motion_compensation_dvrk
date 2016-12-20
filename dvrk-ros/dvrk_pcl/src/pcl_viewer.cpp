#include "iostream"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "pcl_ros/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/visualization/cloud_viewer.h"
#include "pcl/visualization/pcl_visualizer.h"
#include "boost/thread/thread.hpp"
#include "time.h"

#define VIEWER
ros::Publisher pub;

#ifdef VIEWER
pcl::visualization::CloudViewer viewer("Cloud Viewer");
#endif

#ifdef VISUALIZER
boost::shared_ptr<pcl::visualization::PCLVisualizer> visualizer (new pcl::visualization::PCLVisualizer ("PLViewer"));
#endif

#ifdef VIEWER
void cloudViewer(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    //clock_t t1,t2;
    //t1 = clock();
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg(*msg, *cloud);

    viewer.showCloud(cloud, "point cloud");

    //t2 = clock();
    //float diff = (float)t2 - (float)t1;
    //float seconds = diff / CLOCKS_PER_SEC;
    //std::cout << seconds << std::endl;
}
#endif

#ifdef VISUALIZER
void cloudVisualizer(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    //clock_t t1,t2;
    //t1 = clock();
    visualizer->removePointCloud("point cloud");
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg(*msg, *cloud);
    visualizer->addPointCloud(cloud, "point cloud");
    //t2 = clock();
    //float diff = (float)t2 - (float)t1;
    //float seconds = diff / CLOCKS_PER_SEC;
    //std::cout << seconds << std::endl;
    visualizer->spinOnce();
}
#endif

void paraSettings(ros::NodeHandle nh)
{
#ifdef VISUALIZER
    visualizer->addCoordinateSystem(50,0);
#endif
    nh.setParam("/stereo/stereo_image_proc/prefilter_size", 5);
    nh.setParam("/stereo/stereo_image_proc/prefilter_cap", 63);
    nh.setParam("/stereo/stereo_image_proc/correlation_windo", 23);
    nh.setParam("/stereo/stereo_image_proc/min_disparity", -96);
    nh.setParam("/stereo/stereo_image_proc/disparity_range", 48);
    nh.setParam("/stereo/stereo_image_proc/uniqueness_ratio", 0.0);
    nh.setParam("/stereo/stereo_image_proc/texture_threshold", 0);
    nh.setParam("/stereo/stereo_image_proc/speckle_size", 1000);
    nh.setParam("/stereo/stereo_image_proc/speckle_range", 29);
}

/*
void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
  // ... do data processing

  sensor_msgs::PointCloud2 output;
  // Publish the data
  pub.publish (output);
}
*/

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pcdViewer");

    ros::NodeHandle nh;
    paraSettings(nh);
    ros::Subscriber pcd_sub;
#ifdef VIEWER
    pcd_sub = nh.subscribe("input", 1, cloudViewer);
#endif

#ifdef VISUALIZER
    pcd_sub = nh.subscribe("input", 1, cloudVisualizer);
#endif
    ros::spin();
}
