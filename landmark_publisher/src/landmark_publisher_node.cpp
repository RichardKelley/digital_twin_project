#include <iostream>
#include <ros/ros.h>
#include <ros/package.h>

#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>

#include <map_manager/Atlas.h>
#include <map_manager/Landmark.h>
#include <map_manager/GetLandmarkCloud.h>

#include <landmark_publisher/SetCloudToPublish.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

bool publishing = false;
ros::Publisher cloud_pub;
ros::Publisher status_pub;
pcl::PointCloud<pcl::PointXYZI> current_cloud;
map_manager::Atlas atlas;
ros::ServiceClient landmark_client;

void atlas_cb(const map_manager::AtlasConstPtr& a) {
  atlas = *a;
}

bool set_active_pointcloud(landmark_publisher::SetCloudToPublish::Request& req,
			   landmark_publisher::SetCloudToPublish::Response& res) {

  if (req.cloud_id == -1) {
    publishing = false;
    ROS_INFO("Ceasing publish.");
    return true;
  }
  
  if (req.cloud_id >= atlas.landmarks.size()) {
    ROS_WARN("Request id out of bounds.");
    publishing = false;
    return false;
  }

  map_manager::GetLandmarkCloud srv;
  srv.request.cloud_id = req.cloud_id;

  if (landmark_client.call(srv)) {
    pcl::fromROSMsg(srv.response.landmark_cloud, current_cloud);
    publishing = true;
    return true;
  } else {
    ROS_WARN("Failed to retrieve valid cloud.");
    publishing = false;
    return false;
  }

  //  return true;
}

void timer_cb(const ros::TimerEvent& e) {
  std_msgs::Bool currently_publishing;
  sensor_msgs::PointCloud2 cloud_out;
  //cloud_out.header.frame_id = "map";
  //cloud_out.header.stamp = ros::Time::now();
  
  if (publishing) {
    pcl::toROSMsg(current_cloud, cloud_out);
    cloud_out.header.frame_id = "map";
    cloud_out.header.stamp = ros::Time::now();

    cloud_pub.publish(cloud_out);
    currently_publishing.data = true;
  } else {
    pcl::PointCloud<pcl::PointXYZI> dummy_cloud;
    pcl::PointXYZI dummy_pt;
    dummy_pt.x = 0.0;
    dummy_pt.y = 0.0;
    dummy_pt.z = 0.0;
    dummy_pt.intensity = 0;
    dummy_cloud.points.push_back(dummy_pt);
    currently_publishing.data = false;
    pcl::toROSMsg(dummy_cloud, cloud_out);
    cloud_out.header.frame_id = "map";
    cloud_out.header.stamp = ros::Time::now();
  
    cloud_pub.publish(cloud_out);
  }
  status_pub.publish(currently_publishing);
}

int main(int argc, char** argv) {

  ros::init(argc, argv, "landmark_publisher");
  ros::NodeHandle nh;

  ros::Timer timer = nh.createTimer(ros::Duration(1), timer_cb);
  cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("current_landmark", 10);
  status_pub = nh.advertise<std_msgs::Bool>("landmark_publisher/publishing", 10);
  ros::Subscriber atlas_sub = nh.subscribe("digital_twin/atlas", 1, atlas_cb);
  landmark_client = nh.serviceClient<map_manager::GetLandmarkCloud>("digital_twin/subcloud");
  ros::ServiceServer cloud_service = nh.advertiseService("landmark_publisher/set_active_cloud", set_active_pointcloud);
  
  ros::spin();
  return 0;

}
