#!/usr/bin/env python3

##from __future__ import division, print_function, absolute_import

import roslib
import rospy
import numpy as np
from qcar.q_misc import *
from qcar.q_ui import *
from qcar.q_interpretation import *

from geometry_msgs.msg import Vector3Stamped
from std_msgs.msg import String
import time

class CommandNode(object):
    def __init__(self):
        self.cmd_sub = rospy.Subscriber("/qcar/keyboard_command", String, self.callback)
        self.cmd_pub_ = rospy.Publisher('/qcar/user_command', Vector3Stamped, queue_size=100)

        self.keyboard_cmd = None
        n = 0

    def callback(self, data):
        if data is None:
            rospy.loginfo("waiting for the keyborad cmd!")
        else:
            rospy.loginfo("receive keyboard commands!")
            self.keyboard_cmd  = data


    def process_command(self, pose):
        pub_cmd = Vector3Stamped()
        pub_cmd.header.stamp = rospy.Time.now()
        pub_cmd.header.frame_id = 'command_input'
        pub_cmd.vector.x = float(pose[0])
        pub_cmd.vector.y = float(pose[1])
        self.cmd_pub_.publish(pub_cmd)

    def run(self):
        throttle = 0.0
        steering_angle = 0.0
        while not rospy.is_shutdown():
            if self.keyboard_cmd is None:
                pose = np.array([throttle, steering_angle])
                self.process_command(pose)
            else:
                if self.keyboard_cmd.data == 'up':
                    throttle = 0.1 + throttle
                    steering_angle = steering_angle
                elif self.keyboard_cmd.data == 'down':
                    throttle = -0.1 + throttle
                    steering_angle = steering_angle
                elif self.keyboard_cmd.data == 'right':
                    throttle = throttle
                    steering_angle = -0.1 + steering_angle
                elif self.keyboard_cmd.data == 'left':
                    throttle = throttle
                    steering_angle = 0.1 + steering_angle
                elif self.keyboard_cmd.data == 'suffle':
                    if n == 0:
                        throttle = 0.1
                        steering_angle = 0.0
                        n = n + 1
                    else:
                        throttle = 0.0
                        steering_angle = 0.0
                elif self.keyboard_cmd.data == 'stop':
                    throttle = 0.0
                    steering_angle = 0.0
                pose = np.array([throttle, steering_angle])
                self.process_command(pose)

        time.sleep(0.01)

if __name__ == '__main__':
    rospy.init_node('command_node')
    r = CommandNode()
    r.run()
