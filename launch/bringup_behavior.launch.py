from launch import LaunchDescription
from launch.actions import GroupAction
from launch_ros.actions import Node
def generate_launch_description():
    load_nodes = GroupAction(
        actions=[
            Node(
                package='cod_behavior',
                executable='cod_behavior',
                name='cod_behavior_node',
                output='screen'
            ),
            Node(
                package='cod_behavior',
                executable='checkNavResult',
                name='CheckNavResult_node',
                output='screen'
            )
        ]
    )

    ld = LaunchDescription()
    ld.add_action(load_nodes)
    return ld