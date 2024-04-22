#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from pynput import keyboard

class TeleopNode(Node):
    def __init__(self):
        super().__init__('teleop_keyboard')
        self.publisher_ = self.create_publisher(Twist, 'target_vel', 1)
        self.timer_ = self.create_timer(0.1, self.timer_callback)
        self.linear_factor = 0.5
        self.angular_factor = 0.5
        self.listener = keyboard.Listener(on_press=self.on_press, on_release=self.on_release)
        self.key_states = {}  # Dictionary to store key states
        self.listener.start()


        
    def on_press(self, key):
        try:
            self.key_states[key.char] = True  # Set key state to True when pressed
            print(key.char)
            # ... (update velocities based on key states)
        except AttributeError:
            pass

    def on_release(self, key):
        try:
            self.key_states[key.char] = False  # Set key state to False when released
            print(key.char)
            # ... (update velocities based on key states)
        except AttributeError:
            pass

    def timer_callback(self):
        linear_x = 0.0
        linear_y = 0.0
        angular_z = 0.0
        if self.key_states.get('w', False):
            linear_x += self.linear_factor
        if self.key_states.get('s', False):
            linear_x += -self.linear_factor
        if self.key_states.get('a', False):
            linear_y += self.linear_factor
        if self.key_states.get('d', False):
            linear_y += -self.linear_factor
        if self.key_states.get('q', False):
            angular_z += self.angular_factor
        if self.key_states.get('e', False):
            angular_z += -self.angular_factor
    
        twist = Twist()
        twist.linear.x = linear_x
        twist.linear.y = linear_y
        twist.angular.z = angular_z
        self.publisher_.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = TeleopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
