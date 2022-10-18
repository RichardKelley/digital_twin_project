#ifndef SEND_EMAIL_HPP
#define SEND_EMAIL_HPP

#include <iostream>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

class SendEmail {
public:
  BT::NodeStatus send_email() {

    std::cerr << "send email" << std::endl;
    
    return BT::NodeStatus::SUCCESS;
  }
  
};


#endif // SEND_EMAIL_HPP
