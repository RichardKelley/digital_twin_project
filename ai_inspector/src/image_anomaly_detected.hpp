#ifndef IMAGE_ANOMALY_DETECTED_HPP
#define IMAGE_ANOMALY_DETECTED_HPP

#include <iostream>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

inline BT::NodeStatus ImageAnomalyDetected() {

  std::cerr << "image anomaly detected." << std::endl;
  
  return BT::NodeStatus::SUCCESS;
}


#endif // IMAGE_ANOMALY_DETECTED_HPP
