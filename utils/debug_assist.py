#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64

rospy.init_node("debug_assist")
target_to_pub = Float64()
pub=rospy.Publisher("/daedalus/target_ref", Float64, queue_size=10)

def callback(msg):
    target_to_pub.data=msg.data

sub=rospy.Subscriber("/daedalus/target", Float64, callback)

r=rospy.Rate(2)
while not rospy.is_shutdown():
    pub.publish(target_to_pub)
    r.sleep()