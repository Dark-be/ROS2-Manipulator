#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include "arm_hardware/sdk_sagittarius_arm_real.h"
#include "arm_hardware/sdk_sagittarius_arm_log.h"
#include "arm_hardware/modern_robotics.h"
#include <rclcpp_action/rclcpp_action.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>

using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
using GoalHandleFollowJointTrajectory = rclcpp_action::ServerGoalHandle<FollowJointTrajectory>;

class SagittariusNode: public rclcpp::Node
{
public:
    explicit SagittariusNode(std::string name): Node(name){
        sar_ = std::make_shared<sdk_sagittarius_arm::SagittariusArmReal>("/dev/ttyACM0", 1000000, 500, 5);
        sar_->SetFreeAfterDestructor(false);
        //sgr_kinematics_ = std::make_shared<sdk_sagittarius_arm::SagittariusArmKinematics>(0, 0, 0);
        RCLCPP_INFO(this->get_logger(), "Sagittarius driver is running");

        joint_states_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>(
            "joint_states", 10
        );
        using namespace std::placeholders;  // NOLINT

        this->action_server_ = rclcpp_action::create_server<FollowJointTrajectory>(
                this, "sgr_arm_controller/follow_joint_trajectory",
                std::bind(&SagittariusNode::HandleGoal, this, _1, _2),
                std::bind(&SagittariusNode::HandleCancel, this, _1),
                std::bind(&SagittariusNode::HandleAccepted, this, _1));

        js_pub_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), std::bind(&SagittariusNode::PublishJointStates, this)
        );
    }
private:
 
    rclcpp_action::GoalResponse HandleGoal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const FollowJointTrajectory::Goal> goal){   
        int point_num = goal->trajectory.points.size();
        RCLCPP_INFO(this->get_logger(), "Received goal request with %d points", point_num);
        //print points

        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse HandleCancel(
        const std::shared_ptr<GoalHandleFollowJointTrajectory> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void Execute(const std::shared_ptr<GoalHandleFollowJointTrajectory> goal_handle){
        const auto goal = goal_handle->get_goal();
        RCLCPP_INFO(this->get_logger(), "Executing goal");
        auto result = std::make_shared<FollowJointTrajectory::Result>();
        int trjPointNum = goal->trajectory.points.size();
        int trjIndex = 0;
        while(trjIndex < trjPointNum){
            if(trjIndex > 0){
                auto last_time = goal->trajectory.points[trjIndex-1].time_from_start;
                auto current_time = goal->trajectory.points[trjIndex].time_from_start;

                rclcpp::Time time1(last_time.sec, last_time.nanosec);
                rclcpp::Time time2(current_time.sec, current_time.nanosec);
                rclcpp::Duration duration = time2 - time1;

                int64_t duration_ns = ((int64_t)duration.seconds()) * 1e9 + ((int64_t)duration.nanoseconds());
                std::chrono::nanoseconds ns(duration_ns);

                rclcpp::sleep_for(ns);
            }
            auto feedback = std::make_shared<FollowJointTrajectory::Feedback>();
            feedback->header.frame_id = goal->trajectory.header.frame_id;
            feedback->header.stamp = goal->trajectory.header.stamp;
            feedback->joint_names = goal->trajectory.joint_names;

            auto current_point = goal->trajectory.points[trjIndex];
            float joint_states[7];
            for(int i = 0; i < 7; i++){
                joint_states[i] = current_point.positions[i];
            }
            sar_->SetAllServoRadian(joint_states);
            sar_->arm_set_gripper_linear_position(current_point.positions[6]*2);

            goal_handle->publish_feedback(feedback);
            if(goal_handle->is_canceling()){
                result->error_code = -1;
                result->error_string = "Goal canceled";
                goal_handle->canceled(result);
                RCLCPP_INFO(this->get_logger(), "Goal canceled");
                return;
            }
            //RCLCPP_INFO(this->get_logger(), "Executing point %d", trijIndex);

            trjIndex++;
        }
        result->error_code = 0;
        goal_handle->succeed(result);
        RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    }

    void HandleAccepted(const std::shared_ptr<GoalHandleFollowJointTrajectory> goal_handle){
        using std::placeholders::_1;
        RCLCPP_INFO(this->get_logger(), "Goal accepted");
        std::thread{std::bind(&SagittariusNode::Execute, this, _1), goal_handle}.detach();
    }

    void PublishJointStates(){
        float buf[7];
        sar_->GetCurrentJointStatus(buf);
        sensor_msgs::msg::JointState joint_states;

        joint_states.header.stamp = rclcpp::Clock().now();
        joint_states.name.resize(8);
        joint_states.position.resize(8);

        joint_states.name[0] = "joint1";
        joint_states.position[0] = buf[0];
        joint_states.name[1] = "joint2";
        joint_states.position[1] = buf[1];
        joint_states.name[2] = "joint3";
        joint_states.position[2] = buf[2];
        joint_states.name[3] = "joint4";
        joint_states.position[3] = buf[3];
        joint_states.name[4] = "joint5";
        joint_states.position[4] = buf[4];
        joint_states.name[5] = "joint6";
        joint_states.position[5] = buf[5];
        joint_states.name[6] = "joint_gripper_left";
        joint_states.position[6] = -buf[6] * 0.026 / 900.0;
        joint_states.name[7] = "joint_gripper_right";
        joint_states.position[7] = -buf[6] * 0.026 / 900.0;

        //RCLCPP_INFO(this->get_logger(), "Publishing joint states");
        joint_states_publisher_->publish(joint_states);
    }

    rclcpp_action::Server<FollowJointTrajectory>::SharedPtr action_server_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_states_publisher_;
    rclcpp::TimerBase::SharedPtr js_pub_timer_;

    std::shared_ptr<sdk_sagittarius_arm::SagittariusArmReal> sar_;
    std::shared_ptr<sdk_sagittarius_arm::SagittariusArmKinematics> sgr_kinematics_;

};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SagittariusNode>("sgr");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;

}

