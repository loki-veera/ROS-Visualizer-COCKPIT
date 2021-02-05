#!/usr/bin/env python

import rospy
from std_msgs.msg import String
#
# def callback(data):
#     rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data.data)
#
# def listener():
#
#     # In ROS, nodes are uniquely named. If two nodes with the same
#     # name are launched, the previous one is kicked off. The
#     # anonymous=True flag means that rospy will choose a unique
#     # name for our 'listener' node so that multiple listeners can
#     # run simultaneously.
#     rospy.init_node('listener', anonymous=True)
#
#     rospy.Subscriber('chatter', String, callback)
#
#     # spin() simply keeps python from exiting until this node is stopped
#     rospy.spin()
#
# if __name__ == '__main__':
#     listener()
def printer():
    rospy.init_node('printer_node')
    pub_1 = rospy.Publisher('phrases', String, queue_size=10)
    pub_2 = rospy.Publisher('seconds', String, queue_size=10)

    rate = rospy.Rate(2)
    msg_str = String()
    msg_str = "Hello World - ROS TUTORIAL"
    msg_str_1 = String()
    msg_str_1 = "This is second message"

    while not rospy.is_shutdown():
        rospy.loginfo(msg_str)
        rospy.loginfo(msg_str_1)
        pub_1.publish(msg_str)
        pub_2.publish(msg_str_1)
        rate.sleep()

if __name__ == '__main__':
    try:
        printer()
    except rospy.ROSInterruptException:
        pass
