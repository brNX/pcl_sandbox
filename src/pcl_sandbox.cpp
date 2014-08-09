#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/filters/radius_outlier_removal.h>

ros::Publisher pub;

double radius;
double min_neighbors;

void
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  // Container for original & filtered data
  pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
  pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
  pcl::PCLPointCloud2 cloud_filtered;

  // Convert to PCL data type
  pcl_conversions::toPCL(*cloud_msg, *cloud);

  // Perform the actual filtering
  pcl::RadiusOutlierRemoval<pcl::PCLPointCloud2> outrem;
  // build the filter
  outrem.setInputCloud(cloudPtr);
  outrem.setRadiusSearch(radius);
  outrem.setMinNeighborsInRadius(min_neighbors);
  outrem.filter(cloud_filtered);

  // Convert to ROS data type
  sensor_msgs::PointCloud2 output;
  pcl_conversions::moveFromPCL(cloud_filtered, output);

  // Publish the data
  pub.publish (output);
}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "pcl_sandbox");
  ros::NodeHandle nh;
  ros::NodeHandle pn("~");

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> ("input", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);

  pn.param<double>("radius", radius ,0.8);
  pn.param<double>("min_neighbors",min_neighbors,2);


  // Spin
  ros::spin ();
}
