#!/usr/bin/env python

import os
import rospy
from duckietown import DTROS
#duckie bot API:Motors
from duckietown_msgs.msg import WheelsCmdStamped, BoolStamped
from std_msgs.msg import String
from time import sleep

class Demo1(DTROS):

    def __init__(self,node_name):
        super(Demo1,self).__init__(node_name=node_name)
        '''initialise the publisher
        self.pub = rospy.publisher(name,message_type,queue_size=1)
        name:
        + string type /duckiebotX/wheels_driver_node/wheels_cmd
        + X is the duckiebot numver
        message_type:
        + send a message with desired velocities WheelsCmdStamped
        '''
        self.wheelpub = rospy.Publisher(
            '/duckiebot3/wheels_driver_node/wheels_cmd',
            WheelsCmdStamped,
            queue_size=1)

        self.chatterpub = rospy.Publisher('chatter', String,queue_size=10)

    def forward(self,velocity,time):
#        print "Forward"
        msg = WheelsCmdStamped()
        msg.header.stamp = rospy.get_rostime()
        msg.vel_left = velocity
        msg.vel_right = velocity
        self.wheelpub.publish(msg)
        rospy.sleep(time)
        message = "distance: forward"
        rospy.loginfo("Publishing message: '%s'" % message)
        self.chatterpub.publish(message)


    def rotate(self,velocity,time):
#        print "Rotate"
        msg = WheelsCmdStamped()
        msg.header.stamp = rospy.get_rostime()
        msg.vel_left = velocity
        msg.vel_right = -velocity
        self.wheelpub.publish(msg)
        rospy.sleep(time)
        message = "Rotate:done"
        rospy.loginfo("Publishing message: '%s'" % message)
        self.chatterpub.publish(message)

    def stop(self):
        msg = WheelsCmdStamped()
        msg.header.stamp = rospy.get_rostime()
        msg.vel_left = 0.0
        msg.vel_right = 0.0
        self.wheelpub.publish(msg)
        message= "Stop"
        rospy.loginfo("Publishing message: '%s'" % message)
        self.chatterpub.publish(message)

    def run(self):
        rospy.sleep(2)
        self.forward(0.6,2)
        self.rotate(0.5,0.5)
        self.forward(0.6,2)
        self.stop()




if __name__ == '__main__':
    node = Demo1(node_name="demo1")
    node.run()
