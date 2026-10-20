#!/usr/bin/env python3
# exit 0 if /clock and /current_state are flowing (simulator + bridge alive), else 1
import sys, time, rospy
from rosgraph_msgs.msg import Clock
from nav_msgs.msg import Odometry
try:
    rospy.init_node('sim_ok', anonymous=True, disable_signals=True)
except Exception:
    sys.exit(1)
n = {'c': 0, 'o': 0}
rospy.Subscriber('/clock', Clock, lambda m: n.__setitem__('c', n['c'] + 1))
rospy.Subscriber('/current_state', Odometry, lambda m: n.__setitem__('o', n['o'] + 1))
t = time.time()
while time.time() - t < float(sys.argv[1]) if len(sys.argv) > 1 else 3.0:
    if n['c'] > 5 and n['o'] > 5:
        sys.exit(0)
    time.sleep(0.05)
sys.exit(1)
