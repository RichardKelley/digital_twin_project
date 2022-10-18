#ifndef CLOUD_ANOMALY_DETECTED_HPP
#define CLOUD_ANOMALY_DETECTED_HPP

#include <iostream>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

inline BT::NodeStatus CloudAnomalyDetected() {

  std::cerr << "cloud anomaly detected." << std::endl;

  return BT::NodeStatus::SUCCESS;
}


#endif // CLOUD_ANOMALY_DETECTED_HPP
