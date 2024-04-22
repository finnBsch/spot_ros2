#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rclpy.time import Time, Duration

class VelocityDriver(Node):
    def __init__(self):
        super().__init__('velocity_driver')
        self.dt = 0.1
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 1)
        self.subscription_ = self.create_subscription(Twist, 'target_vel', self.vel_callback, 1)
        self.timer_ = self.create_timer(self.dt, self.timer_callback)
        self.linear_x = 0.0
        self.linear_y = 0.0
        self.angular_z = 0.0
        self.deadzone = 0.1
        self.desired_lin_x = 0.0
        self.desired_lin_y = 0.0
        self.desired_ang_z = 0.0
        self.max_acc = 1.0
        self.lin_max = 0.5
        self.ang_max = 0.5
        self.num_no_messages = 0
        self.last_message = None
        self.timeout = Duration(seconds=0.5)


    def vel_callback(self, msg):
        self.last_message = self.get_clock().now()
        self.desired_lin_x = msg.linear.x
        self.desired_lin_y = msg.linear.y
        self.desired_ang_z = msg.angular.z

    def timer_callback(self):
        if not self.last_message :
            return
        elif self.get_clock().now() - self.last_message > self.timeout:
            self.get_logger().warning("Last control input too old")
            self.desired_lin_x = 0
            self.desired_lin_y = 0
            self.desired_ang_z = 0
        dx = self.desired_lin_x - self.linear_x
        dy = self.desired_lin_y - self.linear_y
        dz = self.desired_ang_z - self.angular_z

        if abs(dx) > self.max_acc * self.dt:        
            dx = dx / abs(dx) * self.max_acc * self.dt
        if abs(dy) > self.max_acc * self.dt:        
            dy = dy / abs(dy) * self.max_acc * self.dt
        if abs(dz) > self.max_acc * self.dt:        
            dz = dz / abs(dz) * self.max_acc * self.dt

        self.linear_x += dx
        self.linear_y += dy
        self.angular_z += dz

        self.linear_x = min(max(self.linear_x, -self.lin_max), self.lin_max)
        self.linear_y = min(max(self.linear_y, -self.lin_max), self.lin_max)
        self.angular_z = min(max(self.angular_z, -self.ang_max), self.ang_max)
        
        if abs(self.linear_x) < self.deadzone:
            if self.desired_lin_x > self.deadzone:
                self.linear_x = self.deadzone
            elif self.desired_lin_x < -self.deadzone:
                self.linear_x = -self.deadzone
        if abs(self.linear_y) < self.deadzone:
            if self.desired_lin_y > self.deadzone:
                self.linear_y = self.deadzone
            elif self.desired_lin_y < -self.deadzone:
                self.linear_y = -self.deadzone
        if abs(self.angular_z) < self.deadzone:
            if self.desired_ang_z > self.deadzone:
                self.angular_z = self.deadzone
            elif self.desired_ang_z < -self.deadzone:
                self.angular_z = -self.deadzone
        
        twist = Twist()
        twist.linear.x = self.linear_x
        twist.linear.y = self.linear_y
        twist.angular.z = self.angular_z
        self.publisher_.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = VelocityDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
