#include "cod_behavior/sentry_behavior_actions.hpp"
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    const auto node = std::make_shared<rclcpp::Node>("bt_demo_node");

    // 创建行为树工厂
    BT::BehaviorTreeFactory factory;

    // 注册我们的自定义节点
    factory.registerNodeType<SendNav2Goal>("SendNav2Goal");
    factory.registerNodeType<CheckNavResult>("CheckNavResult");
    factory.registerNodeType<CheckGameStatus>("CheckGameStatus");
    factory.registerNodeType<isattacked>("isattacked");
    factory.registerNodeType<movearound>("movearound");
    factory.registerNodeType<isgoinghome>("isgoinghome");
    factory.registerNodeType<lowpower>("lowpower");
    factory.registerNodeType<qsbroke>("qsbroke");

    // 使用XML字符串定义行为树
    const std::string cod_bt = "/home/arlo/CLionProjects/bt/cod_behavior/cod_bt/t1.xml";

    try {
        auto tree = factory.createTreeFromFile(cod_bt);
        // 创建目标位置
        auto blackboard = tree.rootBlackboard();
        geometry_msgs::msg::PoseStamped goal;
        goal.header.frame_id = "map";
        goal.header.stamp = node->now();
        goal.pose.position.x = 1.0;
        goal.pose.position.y = 0.0;
        goal.pose.position.z = 0.0;
        goal.pose.orientation.w = 1.0;
        blackboard->set<geometry_msgs::msg::PoseStamped>("goal_position", goal);

        geometry_msgs::msg::PoseStamped homegoal;
        homegoal.header.frame_id = "map";
        homegoal.header.stamp = node->now();
        homegoal.pose.position.x = 1.0;
        homegoal.pose.position.y = 0.0;
        homegoal.pose.position.z = 0.0;
        homegoal.pose.orientation.w = 1.0;
        blackboard->set<geometry_msgs::msg::PoseStamped>("home_goal_position", homegoal);


        geometry_msgs::msg::PoseStamped baoleigoal;
        baoleigoal.header.frame_id = "map";
        baoleigoal.header.stamp = node->now();
        baoleigoal.pose.position.x = 1.0;
        baoleigoal.pose.position.y = 0.0;
        baoleigoal.pose.position.z = 0.0;
        baoleigoal.pose.orientation.w = 1.0;
        blackboard->set<geometry_msgs::msg::PoseStamped>("baolei_goal_position", baoleigoal);
        tree.tickRootWhileRunning();

    } catch (const std::exception& e) {
        RCLCPP_ERROR(node->get_logger(), "加载或执行行为树时出错: %s", e.what());
        return 1;
    }


    rclcpp::shutdown();
    return 0;
}
