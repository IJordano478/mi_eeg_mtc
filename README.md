# mi_eeg_mtc

Builds on the MoveIt2 Tutorial for pick and place. In terminal 1, you would run:
ros2 launch moveit2_tutorials mtc_demo.launch.py

In terminal 2, you would need to run your custom launch file that calls this ROS2 node and some other items based on the launch file from:
ros2 launch moveit2_tutorials pick_place_demo.launch.py

In this second launch file you would need:
'''
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("moveit_resources_panda").to_dict()

    # MTC Demo node
    pick_place_demo = Node(
        # package="mtc_tutorial",
        # executable="mtc_tutorial",
        package="mi_eeg_mtc",
        executable="mi_eeg_mtc",
        output="screen",
        parameters=[
            moveit_config,
        ],
    )

    return LaunchDescription([pick_place_demo]) 
'''
