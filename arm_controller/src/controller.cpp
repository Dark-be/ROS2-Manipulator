#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_group");

class ControllerNode : public rclcpp::Node
{
public:
  ControllerNode(std::string name, std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group): Node(name){
    arm_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
      "arm_target", 10, std::bind(&ControllerNode::ArmTargetCallback, this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "Start to Received arm target message");
    arm_move_group_ = move_group;
    // rclcpp::NodeOptions node_options;
    // node_options.automatically_declare_parameters_from_overrides(true);
    // move_group_node = rclcpp::Node::make_shared("move_group_controller", node_options);
    
    // rclcpp::executors::SingleThreadedExecutor executor;
    // executor.add_node(move_group_node);
    // std::thread([&executor]() { executor.spin(); }).detach();

    //move_group = new moveit::planning_interface::MoveGroupInterface(move_group_node, "sgr_arm");

    RCLCPP_INFO(LOGGER, "Planning frame: %s", arm_move_group_->getPlanningFrame().c_str());

    RCLCPP_INFO(LOGGER, "End effector link: %s", arm_move_group_->getEndEffectorLink().c_str());

    RCLCPP_INFO(LOGGER, "Available Planning Groups:");
    std::copy(arm_move_group_->getJointModelGroupNames().begin(), arm_move_group_->getJointModelGroupNames().end(),
      std::ostream_iterator<std::string>(std::cout, ", "));
    arm_target_pose.orientation.w = 1.0;
    arm_target_pose.position.x = 0.2;
    arm_target_pose.position.y = -0.1;
    arm_target_pose.position.z = 0.4;
    arm_move_group_->setPoseTarget(arm_target_pose);

    bool success = (arm_move_group_->plan(arm_plan) == moveit::core::MoveItErrorCode::SUCCESS);

    RCLCPP_INFO(LOGGER, "Initial pose (pose goal) %s", success ? "" : "FAILED");//set in

    arm_move_group_->execute(arm_plan);

    
  }
private:
  void ArmTargetCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg){
    RCLCPP_INFO(this->get_logger(), "Received arm target message:");
    if(msg->data.size() != 6){
      RCLCPP_ERROR(this->get_logger(), "Invalid arm target message");
      return;
    }

    arm_target_pose.position.x = msg->data[0];
    arm_target_pose.position.y = msg->data[1];
    arm_target_pose.position.z = msg->data[2];
    Eigen::Quaterniond q = Eigen::AngleAxisd(msg->data[5], Eigen::Vector3d::UnitZ())
      * Eigen::AngleAxisd(msg->data[4], Eigen::Vector3d::UnitY())
      * Eigen::AngleAxisd(msg->data[3], Eigen::Vector3d::UnitX());
    arm_target_pose.orientation.w = 1;
    arm_target_pose.orientation.x = 0;
    arm_target_pose.orientation.y = 0;
    arm_target_pose.orientation.z = 0;
    RCLCPP_INFO(this->get_logger(), "%f %f %f %f %f %f",
      arm_target_pose.position.x, arm_target_pose.position.y, arm_target_pose.position.z,
      arm_target_pose.orientation.x, arm_target_pose.orientation.y, arm_target_pose.orientation.z);
    arm_move_group_->setPoseTarget(arm_target_pose);
    bool success = (arm_move_group_->plan(arm_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    RCLCPP_INFO(this->get_logger(), "Plan (pose goal) %s", success ? "" : "FAILED");
    if(success){
      arm_move_group_->execute(arm_plan);
    }
  }

  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr arm_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr hand_sub_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> arm_move_group_;

  moveit::planning_interface::MoveGroupInterface::Plan arm_plan;

  geometry_msgs::msg::Pose arm_target_pose;
};



int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  rclcpp::Node::SharedPtr move_group_node = rclcpp::Node::make_shared("move_group_controller", node_options);
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> arm_move_group =
    std::make_shared<moveit::planning_interface::MoveGroupInterface>(move_group_node, "sgr_arm");

  rclcpp::Node::SharedPtr node = std::make_shared<ControllerNode>("controller_node", arm_move_group);
    
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(move_group_node);
  std::thread([&executor]() { executor.spin(); }).detach();

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}