#include "rclcpp/rclcpp.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "std_msgs/msg/int32.hpp"
#include <limits>

// 使用正确的消息类型，注意下划线(_)与双冒号(::)的区别
using FeedbackMsg = nav2_msgs::action::NavigateToPose_FeedbackMessage;

class NavResultChecker : public rclcpp::Node
{
public:
    NavResultChecker() : Node("nav_result_checker")
    {
        RCLCPP_INFO(this->get_logger(), "启动导航结果检查节点");

        // 创建发布器，发布到/isreached话题
        reached_publisher_ = this->create_publisher<std_msgs::msg::Int32>(
            "/isreached", 10);

        // 创建订阅器来监听导航反馈
        feedback_sub_ = this->create_subscription<FeedbackMsg>(
            "/navigate_to_pose/_action/feedback", 10,
            std::bind(&NavResultChecker::feedbackCallback, this, std::placeholders::_1));

        // 初始化变量
        current_distance_ = std::numeric_limits<float>::max();
        nav_reached_ = false;

        // 创建定时器，定期检查和发布状态
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),  // 每秒检查一次
            std::bind(&NavResultChecker::timerCallback, this));

        RCLCPP_INFO(this->get_logger(), "导航结果检查节点已启动，等待反馈消息...");
    }

private:
    // 反馈回调函数
    void feedbackCallback(const FeedbackMsg::SharedPtr msg)
    {
        // 更新当前距离
        current_distance_ = msg->feedback.distance_remaining;

        // 如果距离小于阈值，认为已到达
        if (current_distance_ < 0.2 && !nav_reached_) {
            nav_reached_ = true;
            RCLCPP_INFO(this->get_logger(), "距离目标仅剩 %.2f 米，认为已到达", current_distance_);

            // 发布到/isreached话题
            auto message = std_msgs::msg::Int32();
            message.data = 1;
            reached_publisher_->publish(message);

            RCLCPP_INFO(this->get_logger(), "已发布 1 到 /isreached 话题");
        } else {
            RCLCPP_INFO(this->get_logger(), "距离目标还有 %.2f 米", current_distance_);
        }
    }

    // 定时器回调函数
    void timerCallback()
    {
        RCLCPP_INFO(this->get_logger(), "当前距离目标：%.2f 米", current_distance_);

        // 如果已到达目标但尚未发布消息，则发布
        if (nav_reached_) {
            auto message = std_msgs::msg::Int32();
            message.data = 1;
            reached_publisher_->publish(message);
        }
    }

    rclcpp::Subscription<FeedbackMsg>::SharedPtr feedback_sub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr reached_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    float current_distance_;
    bool nav_reached_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NavResultChecker>();
    RCLCPP_INFO(node->get_logger(), "开始运行导航结果检查节点");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}