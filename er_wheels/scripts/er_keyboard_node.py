#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from pynput import keyboard



class KeyboardNode:
    def __init__(self):
        self.speed = 0.25
        self.ang_vel = 0
        self.v = 0

        self.pub = rospy.Publisher('cartesian_motor_controller/twist', Twist, queue_size=10)
        rospy.init_node('keyboard_node', anonymous=True)
        rate = rospy.Rate(10) # 10hz

        with keyboard.Listener(on_press=lambda key: self.on_press(key), on_release=lambda key: self.on_release(key)) as listener:
            listener.join()

    def on_press(self, key):
        try:
            print('alphanumeric key {0} pressed'.format(key.char))
            if key.char == 'w':
                self.v = self.speed
            elif key.char == 'a':
                self.ang_vel = self.speed*3
            elif key.char == 'd':
                self.ang_vel = -self.speed*3
            elif key.char == 's':
                self.v = -self.speed
            elif key.char == 'q' or key == keyboard.Key.esc:
                exit()

            self.publish()
        except AttributeError:
            pass

    def on_release(self,key):
        try:
            print('{0} released'.format(key))
            if key.char == 'w':
                self.v = 0
            elif key.char == 'a':
                self.ang_vel = 0
            elif key.char == 'd':
                self.ang_vel = 0
            elif key.char == 's':
                self.v = 0

            self.publish()
        except AttributeError:
            pass

    def publish(self):
        twist_msg = Twist()
        twist_msg.linear.x = self.v
        twist_msg.angular.z = self.ang_vel
        self.pub.publish(twist_msg)

if __name__ == '__main__':
    try:
        kn = KeyboardNode()
    except rospy.ROSInterruptException:
        pass
