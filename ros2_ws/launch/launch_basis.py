from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='basis',
            namespace='motor',
            executable='motor',
            name='motor'
        ),
        Node(
            package='basis',
            namespace='temp',
            executable='temp',
            name='temp'
        ),
        Node(
            package='basis',
            namespace='camera_gimbal',
            executable='camera_gimbal',
            name='camera_gimbal'
        ),
        Node(
            package='basis',
            namespace='fuses',
            executable='fuses',
            name='fuses'
        ),
        Node(
            package='basis',
            namespace='sensor',
            executable='sensor',
            name='sensor'
        )
    ])