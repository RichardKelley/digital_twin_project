#ifndef MARK_AS_SEEN_HPP
#define MARK_AS_SEEN_HPP

#include <iostream>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

class MarkAsSeen {
public:
  BT::NodeStatus mark_as_seen() {
    std::cerr << "mark as seen" << std::endl;
    return BT::NodeStatus::SUCCESS;
  }

};


#endif // MARK_AS_SEEN_HPP
