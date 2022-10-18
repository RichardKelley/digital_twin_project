#include <iostream>

#include <ros/ros.h>

#define PCL_NO_PRECOMPILE

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/random_sample.h>
#include <pcl_conversions/pcl_conversions.h>

#include <dynamic_reconfigure/server.h>
#include <lidar_downsample/DownsampleConfig.h>

#include <velodyne_pcl/point_types.h>

double keep_fraction = 1.0;

using PointCloud = pcl::PointCloud<velodyne_pcl::PointXYZIRT>;

ros::Publisher downsampled_pub;

pcl::RandomSample <velodyne_pcl::PointXYZIRT> random_sampler;

void reconfigure_callback(lidar_downsample::DownsampleConfig &config, uint32_t level) {
  ROS_INFO("New keep_fraction = %f", config.keep_fraction);
  keep_fraction = config.keep_fraction;  
}

void cloud_callback(const PointCloud::ConstPtr& msg) {
  PointCloud::Ptr downsampled_cloud {new PointCloud};
  downsampled_cloud->header.frame_id = msg->header.frame_id;

  random_sampler.setInputCloud(msg);
  random_sampler.setSample((unsigned int) (keep_fraction * msg->width * msg->height));
  random_sampler.filter(*downsampled_cloud);
  
  pcl_conversions::toPCL(ros::Time::now(), downsampled_cloud->header.stamp);
  downsampled_pub.publish(*downsampled_cloud);
}

int main(int argc, char** argv) {

  ros::init(argc, argv, "lidar_downsample_node");

  ros::NodeHandle nh;

  ros::Subscriber cloud_sub = nh.subscribe<PointCloud>("velodyne_points", 1, cloud_callback);

  downsampled_pub = nh.advertise<PointCloud>("downsampled_cloud", 1);
  
  dynamic_reconfigure::Server<lidar_downsample::DownsampleConfig> server;
  dynamic_reconfigure::Server<lidar_downsample::DownsampleConfig>::CallbackType f;
  f = boost::bind(&reconfigure_callback, _1, _2);
  server.setCallback(f);

  ros::spin();
  return 0;
}
