#include <rm_interfaces/msg/serial_receive_data.hpp>
#include "rclcpp/rclcpp.hpp"
class sendmsg : public rclcpp::Node
{
public:
    sendmsg():Node("sendmsg")
    {
        publisher_=this->create_publisher<rm_interfaces::msg::SerialReceiveData>("SerialReceiveData",10);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&sendmsg::timer_callback,this));

    }
private:
    void timer_callback()
    {
        auto msg = rm_interfaces::msg::SerialReceiveData();
        msg.judge_system_data.game_status = 1;
        RCLCPP_INFO(this->get_logger(),"发布gamestatus:1");
        publisher_->publish(msg);
    }
    rclcpp::Publisher<rm_interfaces::msg::SerialReceiveData>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc,char** argv)
{
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<sendmsg>());
    rclcpp::shutdown();
    return 0;
}