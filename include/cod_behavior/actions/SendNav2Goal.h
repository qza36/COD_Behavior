//
// Created by arlo on 25-4-30.
//
/**
 *  @功能：发送坐标位置
 *
 *
 *
*/
#ifndef SENDNAV2GOAL_H
#define SENDNAV2GOAL_H
#pragma once
#include "behaviortree_cpp_v3/bt_factory.h"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
class SendNav2Goal : public BT::AsyncActionNode
{
public:
    SendNav2Goal(const std::string& name, const BT::NodeConfiguration& config)
        : AsyncActionNode(name, config)
    {
        node_ = rclcpp::Node::make_shared("nav2_goal_client");
        action_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
            node_, "navigate_to_pose");
    }

    static BT::PortsList providedPorts()
    {
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
        RCLCPP_INFO(node_->get_logger(), "导航目标发送成功");
        return BT::NodeStatus::SUCCESS;
    }
private:
    rclcpp::Node::SharedPtr node_;
    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr action_client_;
};
#endif //SENDNAV2GOAL_H
