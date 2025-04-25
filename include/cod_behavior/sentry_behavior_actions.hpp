#pragma once
#include "behaviortree_cpp_v3/bt_factory.h"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/int32.hpp"
#include <rm_interfaces/msg/serial_receive_data.hpp>

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
class CheckNavResult : public BT::CoroActionNode
{
public:
    CheckNavResult(const std::string& name, const BT::NodeConfiguration& config)
        : CoroActionNode(name, config)
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
        RCLCPP_INFO(node_->get_logger(), "开始检查导航结果...");

        // 重置状态
        nav_reached_ = false;

        // 检查循环
        while (!nav_reached_) {
            // 处理回调
            rclcpp::spin_some(node_); //只处理当前队列中的回调后就返回

            if (nav_reached_) {
                RCLCPP_INFO(node_->get_logger(), "导航已完成（收到到达消息）");
                return BT::NodeStatus::SUCCESS;
            }

            // 返回RUNNING但允许行为树继续执行
            setStatusRunningAndYield();
        }

        return BT::NodeStatus::SUCCESS;
    }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr reached_sub_;
    bool nav_reached_;
};
class CheckGameStatus : public BT::CoroActionNode
{
public:
    CheckGameStatus(const std::string& name,const BT::NodeConfiguration& config)
        : CoroActionNode(name,config)
    {
        node_ = rclcpp::Node::make_shared("game_status_checker");
        game_status_sub_ = node_->create_subscription<rm_interfaces::msg::SerialReceiveData>(
        "/SerialReceiveData",10,
        std::bind(&CheckGameStatus::gameStatusCallback,this,std::placeholders::_1));
        is_start = false;
        is_gohome = false;
    }
    static BT::PortsList providedPorts()
    {
        return {};
    }
    BT::NodeStatus tick() override
    {
        RCLCPP_INFO(node_->get_logger(),"检查比赛状态...");
        is_start = false;
        while (!is_start)
        {
            spin_some(node_);
            if (is_start){
                RCLCPP_INFO(node_->get_logger(),"比赛开始");
                return BT::NodeStatus::SUCCESS;
            }
            setStatusRunningAndYield();
        }
        return BT::NodeStatus::SUCCESS;

    }
    void gameStatusCallback(const rm_interfaces::msg::SerialReceiveData msg)
    {
        if (msg.judge_system_data.game_status==1)
        {
            is_start = true;
        }

    }
private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<rm_interfaces::msg::SerialReceiveData>::SharedPtr game_status_sub_;
    bool is_start;
    bool is_gohome;
};
class isattacked : public BT::CoroActionNode
{
public:
    isattacked(const std::string& name,const BT::NodeConfiguration& config)
        : CoroActionNode(name,config)
    {
        node_ = rclcpp::Node::make_shared("is_attacked_checker");
        isattacked_sub_ = node_->create_subscription<rm_interfaces::msg::SerialReceiveData>(
        "/SerialReceiveData",10,
        std::bind(&isattacked::isattackedCallback,this,std::placeholders::_1));
        is_attacked = false;

    }
    static BT::PortsList providedPorts()
    {
        return {};
    }
    BT::NodeStatus tick() override
    {
        RCLCPP_INFO(node_->get_logger(),"检查是否被击打...");

        // 重置状态
        is_attacked = false;

        // 检查循环
        while (!is_attacked)
        {
            // 处理回调
            rclcpp::spin_some(node_); //只处理当前队列中的回s后就返回
            if (is_attacked) {
                RCLCPP_INFO(node_->get_logger(), "被击打");
                return BT::NodeStatus::SUCCESS;
            }else
            {

                RCLCPP_INFO(node_->get_logger(), "没被击打");
                return BT::NodeStatus::FAILURE;
            }

        }
        return BT::NodeStatus::SUCCESS;
    }

    void isattackedCallback(const rm_interfaces::msg::SerialReceiveData msg)
    {
        if (msg.judge_system_data.operator_command.is_outpost_attacking==1)
        {
            is_attacked = true;
        }
    }
private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<rm_interfaces::msg::SerialReceiveData>::SharedPtr isattacked_sub_;
    bool is_attacked = {};
};
class movearound : public BT::CoroActionNode
{
    public:
    movearound(const std::string& name,const BT::NodeConfiguration& config)
        : CoroActionNode(name,config)
    {
        node_ = rclcpp::Node::make_shared("is_movearound_checker");
        spin_cmd_pub_ = node_->create_publisher<rm_interfaces::msg::OperatorCommand>("SerialSend",10);
    }
    static BT::PortsList providedPorts()
    {
        return {};
    }
    BT::NodeStatus tick() override
    {
        auto msg = rm_interfaces::msg::OperatorCommand();
        msg.is_spin=1;
        RCLCPP_INFO(node_->get_logger(),"开始小陀螺...");
        spin_cmd_pub_->publish(msg);
        return BT::NodeStatus::FAILURE;
    }
private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<rm_interfaces::msg::OperatorCommand>::SharedPtr spin_cmd_pub_;
};
class isgoinghome : public BT::CoroActionNode
{
    public:
    isgoinghome(const std::string& name,const BT::NodeConfiguration& config)
        : CoroActionNode(name,config)
    {
        node_ = rclcpp::Node::make_shared("is_isgoinghome_checker");
        isgoinghome_sub_ = node_->create_subscription<rm_interfaces::msg::SerialReceiveData>(
        "/SerialReceiveData",10,
        std::bind(&isgoinghome::isgoinghomeCallback,this,std::placeholders::_1));
        is_goinghome  = false;

    }
    static BT::PortsList providedPorts()
    {
        return {};
    }
    BT::NodeStatus tick() override
    {
        RCLCPP_INFO(node_->get_logger(),"检测是否要回家...");
        is_goinghome = false;
        while (!is_goinghome)
        {
            rclcpp::spin_some(node_);
            if (is_goinghome)
            {
                RCLCPP_INFO(node_->get_logger(), "接收到回家指令");
                return BT::NodeStatus::SUCCESS;
            }
            setStatusRunningAndYield();
        }

        return BT::NodeStatus::SUCCESS;
    }

    void isgoinghomeCallback(const rm_interfaces::msg::SerialReceiveData msg)
    {
        if (msg.judge_system_data.game_status==1)
        {
            is_goinghome = true;
        }
    }
private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<rm_interfaces::msg::SerialReceiveData>::SharedPtr isgoinghome_sub_;
    bool is_goinghome = {};
};
class lowpower : public BT::CoroActionNode
{
    public:
    lowpower(const std::string& name,const BT::NodeConfiguration& config)
        : CoroActionNode(name,config)
    {
        node_ = rclcpp::Node::make_shared("is_lowpower_checker");
        lowpower_sub_ = node_->create_subscription<rm_interfaces::msg::SerialReceiveData>(
        "/SerialReceiveData",10,
        std::bind(&lowpower::lowpowerCallback,this,std::placeholders::_1));
        is_lowpower = false;

    }
    static BT::PortsList providedPorts()
    {
        return {};
    }
    BT::NodeStatus tick() override
    {
        RCLCPP_INFO(node_->get_logger(),"开始低能模式...");
        return BT::NodeStatus::FAILURE;
    }

    void lowpowerCallback(const rm_interfaces::msg::SerialReceiveData msg)
    {
        if (msg.judge_system_data.is_lowpower==1)
        {
            is_lowpower = true;
        }
    }
private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<rm_interfaces::msg::SerialReceiveData>::SharedPtr lowpower_sub_;
    bool is_lowpower = {};
};
class qsbroke : public BT::CoroActionNode
{
    public:
    qsbroke(const std::string& name,const BT::NodeConfiguration& config)
        : CoroActionNode(name,config)
    {
        node_ = rclcpp::Node::make_shared("is_qsbroke_checker");
        qsbroke_sub_ = node_->create_subscription<rm_interfaces::msg::SerialReceiveData>(
        "/SerialReceiveData",10,
        std::bind(&qsbroke::qsbrokeCallback,this,std::placeholders::_1));
        is_qsbroke = false;

    }
    static BT::PortsList providedPorts()
    {
        return {};
    }
    BT::NodeStatus tick() override
    {
        RCLCPP_INFO(node_->get_logger(),"检查我方前哨战是否被毁");
        return BT::NodeStatus::SUCCESS;
    }

    void qsbrokeCallback(const rm_interfaces::msg::SerialReceiveData msg)
    {
        if (msg.judge_system_data.game_status==2)
        {
            is_qsbroke = true;
        }
    }
private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<rm_interfaces::msg::SerialReceiveData>::SharedPtr qsbroke_sub_;
    bool is_qsbroke = {};
};
class is_lowhp : public BT::CoroActionNode
{
public:
    is_lowhp(const std::string& name,const BT::NodeConfiguration& config)
        : CoroActionNode(name,config)
    {
        node_ = rclcpp::Node::make_shared("lowhp_checker");
        is_lowhp_sub_ = node_->create_subscription<rm_interfaces::msg::SerialReceiveData>(
        "/SerialReceiveData",10,
        std::bind(&is_lowhp::hpCallback,this,std::placeholders::_1));
        is_is_lowhp = false;

    }
    static BT::PortsList providedPorts()
    {
        return {BT::InputPort<int>("hp")};
    }
    BT::NodeStatus tick() override
    {
        is_is_lowhp = false;
        RCLCPP_INFO(node_->get_logger(),"检查血量是否低于阈值");
        while (!is_is_lowhp)
        {
            rclcpp::spin_some(node_);
            if (is_is_lowhp)
            {
                RCLCPP_INFO(node_->get_logger(),"血量低于阈值");
                return BT::NodeStatus::SUCCESS;
            }else
            {
                RCLCPP_INFO(node_->get_logger(),"血量健康");
                return BT::NodeStatus::FAILURE;
            }

        }
        return BT::NodeStatus::SUCCESS;
    }

    void hpCallback(const rm_interfaces::msg::SerialReceiveData msg)
    {   auto hp = getInput<int>("hp");
        // 检查hp是否包含有效值
        if (hp.has_value()) {
            // 使用hp.value()获取实际的int值
            if (msg.judge_system_data.hp < hp.value()) {
                is_is_lowhp = true;
            }
        } else {
            // 处理错误情况
            RCLCPP_ERROR(rclcpp::get_logger("sentry_behavior"),
                        "获取hp参数失败: %s", hp.error().c_str());
        }
    }
private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<rm_interfaces::msg::SerialReceiveData>::SharedPtr is_lowhp_sub_;
    bool is_is_lowhp = {};
};
class check_allow_shut_num : public BT::CoroActionNode
{
public:
    check_allow_shut_num(const std::string& name,const BT::NodeConfiguration& config)
        : CoroActionNode(name,config)
    {
        node_ = rclcpp::Node::make_shared("lowhp_checker");
        shut_num_sub_ = node_->create_subscription<rm_interfaces::msg::SerialReceiveData>(
        "/SerialReceiveData",10,
        std::bind(&check_allow_shut_num::CheckShutNumCallback,this,std::placeholders::_1));
        is_low_shut_num = false;

    }
    static BT::PortsList providedPorts()
    {
        return {BT::InputPort<int>("allow_shut_num")};
    }
    BT::NodeStatus tick() override
    {
        is_low_shut_num = false;
        RCLCPP_INFO(node_->get_logger(),"检查允许发弹量是否低于阈值");
        while (!is_low_shut_num)
        {
            rclcpp::spin_some(node_);
            if (is_low_shut_num)
            {
                RCLCPP_INFO(node_->get_logger(),"允许发弹量低于阈值");
                return BT::NodeStatus::SUCCESS;
            }else
            {
                RCLCPP_INFO(node_->get_logger(),"允许发弹量健康");
                return BT::NodeStatus::FAILURE;
            }
        }
        return BT::NodeStatus::SUCCESS;
    }

    void CheckShutNumCallback(const rm_interfaces::msg::SerialReceiveData msg)
    {   auto allow_shut_num = getInput<int>("allow_shut_num");
        if (allow_shut_num.has_value()) {

            if (msg.judge_system_data.shut_num < allow_shut_num.value()) {
                is_low_shut_num = true;
            }
        } else {
            // 处理错误情况
            RCLCPP_ERROR(rclcpp::get_logger("sentry_behavior"),
                        "获取allow_shut_num参数失败: %s", allow_shut_num.error().c_str());
        }
    }
private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<rm_interfaces::msg::SerialReceiveData>::SharedPtr shut_num_sub_;
    bool is_low_shut_num = {};
};
class buy_shut : public BT::CoroActionNode
{
public:
    buy_shut(const std::string& name,const BT::NodeConfiguration& config)
        : CoroActionNode(name,config)
    {
        node_ = rclcpp::Node::make_shared("buy_shut");
        buy_shut_pub_ = node_->create_publisher<rm_interfaces::msg::OperatorCommand>("SerialSend",10);

    }
    static BT::PortsList providedPorts()
    {
        return {};
    }
    BT::NodeStatus tick() override
    {
        auto msg = rm_interfaces::msg::OperatorCommand();
        msg.is_buyshut=1;
        RCLCPP_INFO(node_->get_logger(),"发布买弹指令...");
        buy_shut_pub_->publish(msg);
        return BT::NodeStatus::SUCCESS;
    }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<rm_interfaces::msg::OperatorCommand>::SharedPtr buy_shut_pub_;
};
class attack_qs : public BT::CoroActionNode
{
public:
    attack_qs(const std::string& name,const BT::NodeConfiguration& config)
        : CoroActionNode(name,config)
    {
        node_ = rclcpp::Node::make_shared("attack_qs");
        attack_qs_pub_ = node_->create_publisher<rm_interfaces::msg::OperatorCommand>("SerialSend",10);

    }
    static BT::PortsList providedPorts()
    {
        return {};
    }
    BT::NodeStatus tick() override
    {
        auto msg = rm_interfaces::msg::OperatorCommand();
        msg.is_buyshut=1;
        RCLCPP_INFO(node_->get_logger(),"发布攻击前哨站指令...");
        attack_qs_pub_->publish(msg);
        return BT::NodeStatus::SUCCESS;
    }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<rm_interfaces::msg::OperatorCommand>::SharedPtr attack_qs_pub_;
};
class is_qs_hp_low : public BT::CoroActionNode
{
public:
    is_qs_hp_low(const std::string& name,const BT::NodeConfiguration& config)
        : CoroActionNode(name,config)
    {
        node_ = rclcpp::Node::make_shared("qs_hp_checker");
        qs_hp_sub_ = node_->create_subscription<rm_interfaces::msg::SerialReceiveData>(
            "/SerialReceiveData",10,
            std::bind(&is_qs_hp_low::QsHpChecker,this,std::placeholders::_1));
        is_qs_hp_low_ = false;


    }
    static BT::PortsList providedPorts()
    {
        return {BT::InputPort<int>("qs_hp")};
    }
    BT::NodeStatus tick() override
    {

        RCLCPP_INFO(node_->get_logger(),"检查前哨战血量是否健康...");
        return BT::NodeStatus::SUCCESS;
    }
    void QsHpChecker(const rm_interfaces::msg::SerialReceiveData::SharedPtr msg)
    {
        auto qs_hp = getInput<int>("qs_hp");
        if (qs_hp.has_value())
        {
           if (msg->judge_system_data.qs_hp< qs_hp.value())
           {
               is_qs_hp_low_ = true;
           }
        }else
        {
            RCLCPP_ERROR(rclcpp::get_logger("sentry_behavior"),
             "获取qs_hp参数失败: %s", qs_hp.error().c_str());
        }
    }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<rm_interfaces::msg::SerialReceiveData>::SharedPtr qs_hp_sub_;
    bool is_qs_hp_low_ = {};

};
