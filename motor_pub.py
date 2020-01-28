#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class Talker(Node):
    def __init__(self):
        super().__init__('talker')
        self.pub=self.create_publisher(Twist, 'motor_command',10)
        self.tmr=self.create_timer(1,self.timer_callback)

    def timer_callback(self):
        msg=Twist()
        msg.linear.x=0.5
        msg.linear.y=0.5
        self.pub.publish(msg)

def main():
    rclpy.init()
    node=Talker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        msg=Twist()
        msg.linear.x=0.0
        msg.linear.y=0.0
        node.pub.publish(msg)
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
