#include "behaviortree_cpp_v3/bt_factory.h"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/int32.hpp"

// 定义共享的黑板键名
#define GOAL_HANDLE_KEY "nav2_goal_handle"

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

        // 获取goal handle并保存到黑板
        auto goal_handle = goal_handle_future.get();
        if (!goal_handle) {
            RCLCPP_ERROR(node_->get_logger(), "Goal was rejected by server");
            return BT::NodeStatus::FAILURE;
        }

        // 使用树的共享黑板来存储goal handle
        // 修改：使用config().blackboard替代blackboard()
        config().blackboard->set(GOAL_HANDLE_KEY, goal_handle);

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
        // 初始化ROS2节点
        node_ = rclcpp::Node::make_shared("nav_result_checker");

        // 创建订阅器来监听/isreached话题
        reached_sub_ = node_->create_subscription<std_msgs::msg::Int32>(
            "/isreached", 10,
            std::bind(&CheckNavResult::reachedCallback, this, std::placeholders::_1));
        nav_reached_ = false;
    }

    static BT::PortsList providedPorts()
    {
        return {}; // 不需要端口
    }

    // /isreached话题的回调函数
    void reachedCallback(const std_msgs::msg::Int32::SharedPtr msg)
    {
        // 如果接收到值为1的消息，表示已到达
        if (msg->data == 1) {
            nav_reached_ = true;
            RCLCPP_INFO(node_->get_logger(), "收到到达消息，目标已到达");
        }
    }


    BT::NodeStatus tick() override
    {
        RCLCPP_INFO(node_->get_logger(), "检查导航结果...");

        // 处理任何等待的回调
        rclcpp::spin_some(node_);

        // 检查是否已到达
        if (nav_reached_) {
            RCLCPP_INFO(node_->get_logger(), "导航已完成（收到到达消息）");
            return BT::NodeStatus::SUCCESS;
        }

        // 还未到达，继续运行
        return BT::NodeStatus::RUNNING;
    }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr reached_sub_;
    bool nav_reached_;
};