#include <iostream>
#include <ros/ros.h>
#include <ros/package.h>

#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>

#include <map_manager/Atlas.h>
#include <map_manager/Landmark.h>
#include <map_manager/GetLandmarkCloud.h>

#include <behaviortree_cpp_v3/bt_factory.h>

#include "landmark_proximity.hpp"
#include "cloud_anomaly_detected.hpp"
#include "image_anomaly_detected.hpp"
#include "send_email.hpp"
#include "mark_as_seen.hpp"

#include "behaviortree_cpp_v3/loggers/bt_zmq_publisher.h"

BT::Tree tree;
LandmarkProximity lm_proximity;

void timer_cb(const ros::TimerEvent& e) {
  tree.tickRoot();
}

void localization_cb(const nav_msgs::OdometryConstPtr& loc) {
  lm_proximity.set_localization(loc->pose.pose.position.x, loc->pose.pose.position.y);
}

void atlas_cb(const map_manager::AtlasConstPtr& atlas) {
  lm_proximity.set_atlas(*atlas);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "ai_inspector");

  ros::NodeHandle nh;

  BT::BehaviorTreeFactory factory;

  SendEmail mailer;
  MarkAsSeen marker;
  
  factory.registerSimpleCondition("LandmarkProximity", std::bind(&LandmarkProximity::get_landmarks, &lm_proximity));
  factory.registerSimpleCondition("CloudAnomalyDetected", std::bind(CloudAnomalyDetected));
  factory.registerSimpleCondition("ImageAnomalyDetected", std::bind(ImageAnomalyDetected));
  factory.registerSimpleAction("SendEmail", std::bind(&SendEmail::send_email, &mailer));
  factory.registerSimpleAction("MarkAsSeen", std::bind(&MarkAsSeen::mark_as_seen, &marker));
  
  std::string tree_path_base = ros::package::getPath("ai_inspector");
  
  tree = factory.createTreeFromFile(tree_path_base + "/digital_twin.xml");

  ros::Timer timer = nh.createTimer(ros::Duration(1.0), timer_cb);
  ros::Subscriber localization_sub = nh.subscribe("/localization", 1, localization_cb);
  ros::Subscriber atlas_sub = nh.subscribe("digital_twin/atlas", 1, atlas_cb);

  lm_proximity.init(nh);
  
  BT::PublisherZMQ publisher_zmq(tree);
  
  ros::spin();

  return 0;

}
