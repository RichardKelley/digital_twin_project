#include <ros/ros.h>
#include <ros/package.h>

#include <sensor_msgs/PointCloud2.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
//#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include <map_manager/Landmark.h>
#include <map_manager/Atlas.h>
#include <map_manager/GetLandmarkCloud.h>

#include <yaml-cpp/yaml.h>

#include <iostream>
#include <string>
#include <vector>

#include <fmt/core.h>

std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> landmarks;
map_manager::Atlas atlas;
ros::Publisher atlas_pub;

bool get_landmark_cloud(map_manager::GetLandmarkCloud::Request& req,
			map_manager::GetLandmarkCloud::Response& res) {


  //int idx = req.cloud_id;
  if (req.cloud_id >= atlas.landmarks.size()) {
    return false;
  }

  sensor_msgs::PointCloud2 out;
  pcl::toROSMsg(*landmarks[req.cloud_id], out);
  //res.landmark_cloud = landmarks[req.cloud_id];
  out.header.stamp = ros::Time::now();
  out.header.frame_id = "map";
  res.landmark_cloud = out;
  
  return true;

}

void timer_cb(const ros::TimerEvent& evt) {
  atlas_pub.publish(atlas);
}

int main(int argc, char **argv) {

  ros::init(argc, argv, "map_manager");

  ros::NodeHandle nh;
  
  std::string pkg_path = ros::package::getPath("map_manager");
  
  // load landmarks
  YAML::Node landmarks_file = YAML::LoadFile(pkg_path + "/atlas/landmarks.yaml");

  for (int i = 0; i < landmarks_file.size(); ++i) {
    // load the landmark data
    map_manager::Landmark l;
    l.index = landmarks_file[i]["index"].as<int>();
    l.min_x = landmarks_file[i]["bounds"]["min_x"].as<double>();
    l.min_y = landmarks_file[i]["bounds"]["min_y"].as<double>();
    l.max_x = landmarks_file[i]["bounds"]["max_x"].as<double>();
    l.max_y = landmarks_file[i]["bounds"]["max_y"].as<double>();
    atlas.landmarks.push_back(l);

    // load the point cloud
    pcl::PointCloud<pcl::PointXYZI>::Ptr landmark_cloud(new pcl::PointCloud<pcl::PointXYZI>);

    if (pcl::io::loadPCDFile<pcl::PointXYZI>(pkg_path+"/atlas/"+fmt::format("subcloud_{}.pcd", l.index), *landmark_cloud) == -1) {
      std::cerr << "Failed to load cloud " << l.index << std::endl;
      return 1;
    } else {
      landmarks.push_back(landmark_cloud);
    } 
  }

  atlas_pub = nh.advertise<map_manager::Atlas>("digital_twin/atlas", 1);

  ros::ServiceServer cloud_service = nh.advertiseService("digital_twin/subcloud", get_landmark_cloud);
  ros::Timer timer = nh.createTimer(ros::Duration(1.0), timer_cb);
  
  ros::spin();
  
  return 0;
}
