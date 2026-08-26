#ifndef RM_DECISION_MODE_PUBLISHER_HPP_
#define RM_DECISION_MODE_PUBLISHER_HPP_

#include "behaviortree_cpp/bt_factory.h"
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int8.hpp>
#include "rm_interfaces/msg/robot_mode.hpp"
#include <rclcpp/qos.hpp>

using namespace BT;
namespace rm_decision
{

class ModePublisher : public SyncActionNode
{
public:
  ModePublisher(const std::string& name, 
                const NodeConfig& config,
                std::shared_ptr<rclcpp::Node> node);
  
  ~ModePublisher() override = default;

  NodeStatus tick() override;
  
  static PortsList providedPorts();

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<rm_interfaces::msg::RobotMode>::SharedPtr mode_pub_;
   rm_interfaces::msg::RobotMode robot_mode_;
};

} // namespace rm_decision

#endif 