from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():


    static_tf_publisher_node = Node(package = "tf2_ros", 
                       executable = "static_transform_publisher",
                       arguments = ["0.165", "0", "0.15", "3.141", "0", "0", "base_link", "laser"])

    nodes = [
        static_tf_publisher_node
    ]
    return LaunchDescription(nodes)