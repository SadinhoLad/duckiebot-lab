# #!/usr/bin/env python
# import os
# import rospy
# from duckietown import DTROS
# from std_msgs.msg import String

# class MyNode(DTROS):

#     def __init__(self, node_name):
#         # initialize the DTROS parent class
#         super(MyNode, self).__init__(node_name=node_name)
#         # construct publisher
#         self.pub = rospy.Publisher('chatter', String,queue_size=10)

#     def run(self):
#         # publish message every 1 second
#         rate = rospy.Rate(1) # 1Hz
#         while not rospy.is_shutdown():
#             message = "Hello from %s" % os.environ['VEHICLE_NAME']
#             rospy.loginfo("Publishing message: '%s'" % message)
#             self.pub.publish(message)
#             rate.sleep()

# if __name__ == '__main__':
#     # create the node
#     node = MyNode(node_name='my_node')
#     # run node
#     node.run()
#     # keep spinning
#     rospy.spin()
















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
        self.pub = rospy.Publisher(
            '/duckiebotX/wheels_driver_node/wheels_cmd',
            String,
            queue_size=1)

    def forward(self,velocity,time):
        print "Forward"
        msg = WheelsCmdStamped()
        msg.header.stamp = rospy.get_rostime()
        msg.vel_left = velocity
        msg.vel_right = velocity
        self.pub.publish(msg)
        sleep(time)
        print "distance:'%s'"%(velocity*time)

    def rotate(self,velocity,time):
        print "Rotate"
        msg = WheelsCmdStamped()
        msg.header.stamp = rospy.get_rostime()
        msg.vel_left = velocity
        msg.vel_right = -velocity
        self.pub.publish(msg)
        sleep(time)
        print "Rotate:done"

    def stop(self):
        msg = WheelsCmdStamped()
        msg.header.stamp = rospy.get_rostime()
        msg.vel_left = 0.0
        msg.vel_right = 0.0
        self.pub.publish(msg)
        print("Stop")

    def run(self):
        self.forward(0.6,1)
        self.rotate(0.1,3)
        self.forward(0.6,1)
        self.stop()




if __name__ == '__main__':
    node = Demo1(node_name="demo1")
    node.run()
    rospy.spin()
