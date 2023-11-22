from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    
    optical_driver_1 =  Node(
            package='optical_odometry',
            executable='usb_driver',
            name='usb_driver',
            namespace='optical_driver_1',
            output='screen',
            emulate_tty=True,
            parameters=[
                {'Vendor ID': 1008},
                {'Product ID': 4938},
                {'Bus': 1},
                {'Address': 29},
                {'Dpi': 800},
            ],
        )
    
    optical_driver_2 =  Node(
            package='optical_odometry',
            executable='usb_driver',
            name='usb_driver',
            namespace='optical_driver_2',
            output='screen',
            emulate_tty=True,
            parameters=[
                {'Vendor ID': 7847},
                {'Product ID': 100},
                {'Bus': 1},
                {'Address': 7},
                {'Dpi': 800},
            ],
        )
    
    ld = LaunchDescription()

    ld.add_action(optical_driver_1) 
    ld.add_action(optical_driver_2) 
    
    return ld
