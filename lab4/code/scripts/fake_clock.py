#!/usr/bin/env python3
"""假 /clock：每 1 ms（墙钟）前进 speed ms。用法：fake_clock.py [speed=10]"""
import sys, time
import rospy
from rosgraph_msgs.msg import Clock

speed = float(sys.argv[1]) if len(sys.argv) > 1 else 10.0
pub = rospy.Publisher('/clock', Clock, queue_size=10)
rospy.init_node('fake_clock', anonymous=True, disable_signals=True)
t = 1.0
while True:
    pub.publish(Clock(rospy.Time.from_sec(t)))
    t += 0.001 * speed
    time.sleep(0.001)
