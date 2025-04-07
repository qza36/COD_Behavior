#include "behaviortree_cpp_v3/bt_factory.h"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp_action/rclcpp_action.hpp"


class SayHello : public BT::SyncActionNode
{
public:
    SayHello(const std::string& name) : BT::SyncActionNode(name, {})
    {
    }

    // 执行此节点时会调用此函数
    BT::NodeStatus tick() override
    {
        RCLCPP_INFO(rclcpp::get_logger("COD_BEHAVIOR"), "Hello, ROS2 Behavior Tree!");
        return BT::NodeStatus::SUCCESS;
    }
};
class SayBye : public BT::SyncActionNode
{
public:
    SayBye(const std::string& name) : BT::SyncActionNode(name, {})
    {
    }

    // 执行此节点时会调用此函数
    BT::NodeStatus tick() override
    {
        RCLCPP_INFO(rclcpp::get_logger("COD_BEHAVIOR"), "Bye, ROS2 Behavior Tree!");
        return BT::NodeStatus::SUCCESS;
    }
};
class SendNav2Goal : public BT::AsyncActionNode
{
public:

    SendNav2Goal(const std::string& name, const BT::NodeConfiguration& config)
        : BT::AsyncActionNode(name, config)
    {
        // 在构造函数中初始化ROS2客户端
        node_ = rclcpp::Node::make_shared("nav2_goal_client");
        action_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
            node_, "navigate_to_pose");
    }

    // 定义节点提供的端口
    static BT::PortsList providedPorts()
    {
        // 可以添加输入参数，如目标位置等
        return { BT::InputPort<geometry_msgs::msg::PoseStamped>("goal_pose", "导航目标位置") };
    }

    // 实现tick方法
    BT::NodeStatus tick() override
    {
        RCLCPP_INFO(node_->get_logger(), "发送导航目标...");

        // 简化测试：直接返回成功
        // return BT::NodeStatus::SUCCESS;


        // 检查服务器是否可用
        if (!action_client_->wait_for_action_server(std::chrono::seconds(5))) {
            RCLCPP_ERROR(node_->get_logger(), "Nav2 Action server not available");
            return BT::NodeStatus::FAILURE;
        }

        // 从端口获取目标位置
        auto goal_pose = getInput<geometry_msgs::msg::PoseStamped>("goal_pose");
        if (!goal_pose) {
            RCLCPP_ERROR(node_->get_logger(), "目标位置未指定");
            return BT::NodeStatus::FAILURE;
        }

        // 创建导航目标
        auto goal_msg = nav2_msgs::action::NavigateToPose::Goal();
        goal_msg.pose = *goal_pose;

        // 发送导航目标
        auto goal_handle_future = action_client_->async_send_goal(goal_msg);

        // 等待结果
        if (rclcpp::spin_until_future_complete(node_, goal_handle_future) !=
            rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_ERROR(node_->get_logger(), "发送导航目标失败");
            return BT::NodeStatus::FAILURE;
        }

        RCLCPP_INFO(node_->get_logger(), "导航目标发送成功");
        return BT::NodeStatus::SUCCESS;

    }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr action_client_;
};


class CheckNavResult : public BT::AsyncActionNode
{
public:
    CheckNavResult(const std::string& name, const BT::NodeConfiguration& config)
        : BT::AsyncActionNode(name, config)
    {

        node_ = rclcpp::Node::make_shared("nav2_result_checker");
        action_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
            node_, "navigate_to_pose");
    }

    // Define the ports provided by this node
    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<std::string>("goal_id", "The ID of the navigation goal to check"),
        };
    }

    // Implementation of the tick method
    BT::NodeStatus tick() override
    {
        RCLCPP_INFO(node_->get_logger(), "Checking navigation result...");

        // Check if action server is available
        if (!action_client_->wait_for_action_server(std::chrono::seconds(5))) {
            RCLCPP_ERROR(node_->get_logger(), "Nav2 Action server not available");
            return BT::NodeStatus::FAILURE;
        }

        // Get the goal ID from the input port
        auto goal_id_str = getInput<std::string>("goal_id");
        if (!goal_id_str) {
            RCLCPP_ERROR(node_->get_logger(), "Goal ID not specified");
            return BT::NodeStatus::FAILURE;
        }

        // Convert string goal ID to appropriate format
        // This depends on how you're passing the goal ID between nodes
        rclcpp_action::GoalUUID goal_uuid;
        try {
            // Parse the goal ID string to convert to GoalUUID
            // This is a simplified example - actual implementation might differ
            std::stringstream ss(*goal_id_str);
            for (size_t i = 0; i < goal_uuid.size(); ++i) {
                uint8_t byte;
                ss >> byte;
                goal_uuid[i] = byte;
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(node_->get_logger(), "Failed to parse goal ID: %s", e.what());
            return BT::NodeStatus::FAILURE;
        }

        // Get the goal handle from the action client
        auto goal_handle = action_client_->async_get_result(goal_uuid);

        // Wait for the result with a timeout
        if (rclcpp::spin_until_future_complete(node_, goal_handle, std::chrono::seconds(5)) !=
            rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_ERROR(node_->get_logger(), "Failed to get navigation result");
            return BT::NodeStatus::FAILURE;
        }

        // Get the result
        auto result = goal_handle.get();

        // Check if the navigation was successful
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
            RCLCPP_INFO(node_->get_logger(), "Navigation completed successfully");
            setOutput("reached_goal", true);
            setOutput("result_message", "Navigation goal reached");
            return BT::NodeStatus::SUCCESS;
        } else {
            // Handle different result codes
            std::string status_msg;
            switch (result.code) {
                case rclcpp_action::ResultCode::ABORTED:
                    status_msg = "Navigation aborted";
                    break;
                case rclcpp_action::ResultCode::CANCELED:
                    status_msg = "Navigation canceled";
                    break;
                default:
                    status_msg = "Navigation failed with unknown reason";
            }

            RCLCPP_ERROR(node_->get_logger(), "%s", status_msg.c_str());
            return BT::NodeStatus::FAILURE;
        }
    }

    // Method to handle halt (when the node is interrupted)
    void halt() override
    {
        RCLCPP_INFO(node_->get_logger(), "Navigation check halted");
    }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr action_client_;
};