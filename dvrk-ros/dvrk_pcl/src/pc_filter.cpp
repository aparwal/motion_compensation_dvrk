#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
// PCL specific includes
//#include <pcl/ros/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>


#include <pcl/filters/voxel_grid.h>

ros::Publisher pub;

/*void 
cloud_cb (const pcl::PCLPointCloud2ConstPtr& input)
{
  // ... do data processing
	pcl::PCLPointCloud2 cloud_filtered;
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud (input);
  sor.setLeafSize (0.01, 0.01, 0.01);
  sor.filter (cloud_filtered);

  // sensor_msgs::PointCloud2 output;
  // Publish the data
  pub.publish (cloud_filtered);
}
*/
void 
cloud_cb (const pcl::PCLPointCloud2ConstPtr& input)
{
  pcl::PCLPointCloud2 cloud_filtered;
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud (input);
  sor.setLeafSize (0.01, 0.01, 0.01);
  sor.filter (cloud_filtered);

  // Publish the dataSize 
  pub.publish (cloud_filtered);
}
int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "pcl_tutorial");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("input", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);

  // Spin
  ros::spin ();
}
