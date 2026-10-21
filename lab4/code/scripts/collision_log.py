#!/usr/bin/env python3
"""记录 /tesse/collision（bridge 要加 publish_collisions:=true）：collision_log.py <输出.csv>"""
import sys, rospy
from tesse_ros_bridge.msg import CollisionStats
f = open(sys.argv[1], 'w')
f.write('t,object\n')
def cb(m):
    f.write('%.3f,%s\n' % (m.header.stamp.to_sec(), m.object_name))
    f.flush()
rospy.init_node('collision_log', anonymous=True)
rospy.Subscriber('/tesse/collision', CollisionStats, cb, queue_size=100)
rospy.spin()
