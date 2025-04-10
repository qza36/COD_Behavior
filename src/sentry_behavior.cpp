#include "cod_behavior/sentry_behavior_actions.hpp"
#include "behaviortree_cpp_v3/loggers/bt_zmq_publisher.h"
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("bt_demo_node");

    // 创建行为树工厂
    BT::BehaviorTreeFactory factory;

    // 注册我们的自定义节点
    factory.registerNodeType<SayHello>("SayHello");
    factory.registerNodeType<SayBye>("SayBye");
    factory.registerNodeType<SendNav2Goal>("SendNav2Goal");
    factory.registerNodeType<CheckNavResult>("CheckNavResult");

    // 使用XML字符串定义行为树
    const std::string cod_bt = "/home/arlo/CLionProjects/bt/cod_behavior/cod_bt/t1.xml";

    try {
        auto tree = factory.createTreeFromFile(cod_bt);

        PublisherZMQ publisher_zmq(tree, 100,1666);
        // 创建目标位置
        auto blackboard = tree.rootBlackboard();
        geometry_msgs::msg::PoseStamped goal;
        goal.header.frame_id = "map";
        goal.header.stamp = node->now();
        goal.pose.position.x = 0.0;
        goal.pose.position.y = 0.0;
        goal.pose.position.z = 0.0;
        goal.pose.orientation.w = 1.0;
        blackboard->set<geometry_msgs::msg::PoseStamped>("goal_position", goal);
        //tree.tickRootWhileRunning();
        rclcpp::Rate rate(10); // 10Hz的执行频率
        while (rclcpp::ok()) {
            tree.tickRoot();
            spin_some(node);
            rate.sleep();
        }

    } catch (const std::exception& e) {
        RCLCPP_ERROR(node->get_logger(), "加载或执行行为树时出错: %s", e.what());
        return 1;
    }


    rclcpp::shutdown();
    return 0;
}
