#!/usr/bin/env python3
"""RViz 可视化：门框（编号）、期望路径、实际路径。用法：lab4_viz.py [门框边长 m]"""
import math, sys
import rospy
from geometry_msgs.msg import PoseArray, PoseStamped, Point
from nav_msgs.msg import Odometry, Path
from trajectory_msgs.msg import MultiDOFJointTrajectoryPoint
from visualization_msgs.msg import Marker, MarkerArray

SIZE = float(sys.argv[1]) if len(sys.argv) > 1 else 2.0
rospy.init_node('lab4_viz')
gate_pub = rospy.Publisher('/lab4_viz/gates', MarkerArray, queue_size=1, latch=True)
des_pub = rospy.Publisher('/lab4_viz/desired_path', Path, queue_size=1)
act_pub = rospy.Publisher('/lab4_viz/actual_path', Path, queue_size=1)
des, act = Path(), Path()
des.header.frame_id = act.header.frame_id = 'world'
state = {'gates': False, 'last': None}


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def on_gates(m):
    if state['gates']:
        return
    arr = MarkerArray()
    for i, p in enumerate(m.poses):
        yaw, c, h = yaw_of(p.orientation), p.position, SIZE / 2
        ux, uy = -math.sin(yaw) * h, math.cos(yaw) * h   # gate plane is normal to its yaw
        sq = Marker(type=Marker.LINE_STRIP, ns='gate', id=i)
        sq.header.frame_id = 'world'
        sq.pose.orientation.w = 1
        sq.scale.x = 0.25
        sq.color.r, sq.color.g, sq.color.b, sq.color.a = 1.0, 0.55, 0.0, 1.0
        for s, t in [(-1, -1), (1, -1), (1, 1), (-1, 1), (-1, -1)]:
            sq.points.append(Point(c.x + s * ux, c.y + s * uy, c.z + t * h))
        txt = Marker(type=Marker.TEXT_VIEW_FACING, ns='label', id=i, text=str(i))
        txt.header.frame_id = 'world'
        txt.pose.position = Point(c.x, c.y, c.z + h + 1.2)
        txt.pose.orientation.w = 1
        txt.scale.z = 2.2
        txt.color.r = txt.color.g = txt.color.b = txt.color.a = 1.0
        arr.markers += [sq, txt]
    gate_pub.publish(arr)
    state['gates'] = True


def append(path, pub, x, y, z, stamp, min_step=0.1):
    if path.poses:
        q = path.poses[-1].pose.position
        if (q.x - x) ** 2 + (q.y - y) ** 2 + (q.z - z) ** 2 < min_step ** 2:
            return
    ps = PoseStamped()
    ps.header.frame_id = 'world'
    ps.pose.position = Point(x, y, z)
    ps.pose.orientation.w = 1
    path.poses.append(ps)
    path.header.stamp = stamp
    pub.publish(path)


def on_des(m):
    t = m.transforms[0].translation
    append(des, des_pub, t.x, t.y, t.z, rospy.Time.now())


def on_cur(m):
    p = m.pose.pose.position
    append(act, act_pub, p.x, p.y, p.z, m.header.stamp)


rospy.Subscriber('/desired_traj_vertices', PoseArray, on_gates, queue_size=1)
rospy.Subscriber('/desired_state', MultiDOFJointTrajectoryPoint, on_des, queue_size=50)
rospy.Subscriber('/current_state', Odometry, on_cur, queue_size=50)
rospy.spin()
