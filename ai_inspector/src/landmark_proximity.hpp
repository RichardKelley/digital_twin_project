#ifndef LANDMARK_PROXIMITY_HPP
#define LANDMARK_PROXIMITY_HPP

#include <iostream>

#include <thread>
#include <chrono>
#include <cmath>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include "map_manager/Atlas.h"
#include "map_manager/Landmark.h"

#include <landmark_publisher/SetCloudToPublish.h>

#include <std_msgs/Bool.h>

inline double dist(double x1, double y1, double x2, double y2) {
  return std::sqrt((x1-x2)*(x1-x2) + (y1-y2)*(y1-y2));
}

inline bool intersect(double loc_x, double loc_y, double radius, map_manager::Landmark lm) {
  return (dist(loc_x, loc_y, lm.min_x, lm.min_y) < radius ||
	  dist(loc_x, loc_y, lm.min_x, lm.max_y) < radius ||
	  dist(loc_x, loc_y, lm.max_x, lm.min_y) < radius ||
	  dist(loc_x, loc_y, lm.max_x, lm.max_y) < radius);
}

class LandmarkProximity {
public:

  double loc_x;
  double loc_y;

  map_manager::Atlas atlas;

  ros::ServiceClient current_landmark_client;
  ros::Subscriber landmark_status_sub;

  bool publishing = false;
  
  double radius = 50.0; // meters, used for landmark-lidar
			// intersection test.

  void landmark_status_cb(const std_msgs::Bool msg) {
    publishing = msg.data;
  }
  
  void init(ros::NodeHandle& nh) {
    current_landmark_client = nh.serviceClient<landmark_publisher::SetCloudToPublish>("landmark_publisher/set_active_cloud");
    landmark_status_sub = nh.subscribe<std_msgs::Bool>("landmark_publisher/publishing", 10, &LandmarkProximity::landmark_status_cb, this);
  }
  
  BT::NodeStatus get_landmarks() {
    int current_cloud_idx = -1;

    for (const auto& landmark : atlas.landmarks) {
      if (intersect(loc_x, loc_y, radius, landmark)) {
	landmark_publisher::SetCloudToPublish srv;
	srv.request.cloud_id = landmark.index;

	if (current_landmark_client.call(srv)) {
	  return BT::NodeStatus::SUCCESS;
	} else {	  
	  return BT::NodeStatus::FAILURE;
	}
		
	//return BT::NodeStatus::SUCCESS;
      }
    }

    if (current_cloud_idx == -1) {
      landmark_publisher::SetCloudToPublish srv;
      srv.request.cloud_id = -1;
      if (current_landmark_client.call(srv)) {

      } else {
	ROS_WARN("Couldn't deactivate landmark publisher.");
      }
      
    }
    
    return BT::NodeStatus::FAILURE;
  }

  void set_localization(double x, double y) {
    loc_x = x;
    loc_y = y;
  }

  void set_atlas(const map_manager::Atlas& a) {
    atlas = a;
  }

};

/*
inline BT::NodeStatus LandmarkProximity() {
  // TODO
  using namespace std::chrono_literals;
  std::this_thread::sleep_for(500ms);
  std::cerr << "landmark proximity" << std::endl;
  return BT::NodeStatus::SUCCESS;
}
*/



#endif // LANDMARK_PROXIMITY_HPP
