#! /usr/bin/env python3

import usb.core

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D

class USB(Node):

    def __init__(self):
        # Node setup
        super().__init__('usb_driver') # type: ignore
        self.mouse_pub = self.create_publisher(Pose2D, 'usb_driver', 10)
   

        self.declare_parameter('Vendor ID', 0)
        self.declare_parameter('Product ID', 0)
        self.declare_parameter('Bus', 0)
        self.declare_parameter('Address', 0)
        self.declare_parameter('Dpi', 0)

        vendor_id = self.get_parameter('Vendor ID').get_parameter_value().integer_value
        product_id = self.get_parameter('Product ID').get_parameter_value().integer_value
        bus = self.get_parameter('Bus').get_parameter_value().integer_value
        address = self.get_parameter('Address').get_parameter_value().integer_value
        self.dpi = self.get_parameter('Dpi').get_parameter_value().integer_value

        self.device = usb.core.find(idVendor = vendor_id, idProduct = product_id, bus = bus, address = address)

        if self.device is None:
            self.get_logger().error("Device config mismatch ...")
            exit()


        ep = self.device[0].interfaces()[0].endpoints()[0] # type: ignore
        i = self.device[0].interfaces()[0].bInterfaceNumber # type: ignore

        self.device.reset() # type: ignore

        if self.device.is_kernel_driver_active(i): # type: ignore
            self.device.detach_kernel_driver(i) # type: ignore

        self.device.set_configuration() # type: ignore

        self.eaddr = ep.bEndpointAddress
        self.buffer_size = ep.wMaxPacketSize

        self.dpi_x = 0
        self.dpi_y = 0 

        self.mm_x = 0.0
        self.mm_y = 0.0

        self.mouse_pose = Pose2D()

        self.timer_period = 0.02  # seconds
        self.timer = self.create_timer(self.timer_period, self.master_callback)
     
   
    def master_callback(self):

        try:
            data = self.device.read(self.eaddr, self.buffer_size, timeout = 10) # type: ignore

            dx = data[1]
            dy = data[2]

            self.dpi_x = -dx if dx < 127 else (256 - dx)
            self.mm_x = self.dpi_x * 25.4 / self.dpi

            self.dpi_y =  -dy if dy < 127 else (256 - dy)
            self.mm_y = self.dpi_y * 25.4 / self.dpi

            # self.get_logger().info(f"x: {self.mm_x:.3f}, y: {self.mm_y:.3f}")
        except usb.core.USBTimeoutError as e:
        #    self.get_logger().info(f"x: {self.mm_x:.3f}, y: {self.mm_y:.3f}")
           self.get_logger().info("Robot not moving ... :)", throttle_duration_sec=1)
        except usb.core.USBError as e:
            self.get_logger().error("Device disconnected ...")
            exit()

        self.mouse_pose.x = self.mm_x
        self.mouse_pose.y = self.mm_y

        self.mouse_pub.publish(self.mouse_pose)

def main(args=None):
    rclpy.init(args=args)

    sub_driver_node = USB()
    rclpy.spin(sub_driver_node)

    sub_driver_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()