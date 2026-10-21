#!/usr/bin/env python3
"""量墙的位置：把无人机放到 (x0, y, z) 悬停，再以 speed 沿 -x（或 +x）慢慢移动，
直到 /tesse/collision 报出非地面物体，打印碰撞时的位置。
用法：probe_wall.py x0 y z [direction=-1] [speed=0.15]   （需要 controller_node 在跑）"""
import math, sys, time
import rospy
from nav_msgs.msg import Odometry
from trajectory_msgs.msg import MultiDOFJointTrajectoryPoint
from geometry_msgs.msg import Transform, Twist
from tesse_ros_bridge.msg import CollisionStats
from tesse.env import Env
from tesse.msgs import PropSpeeds, Reposition

x0, y, z = map(float, sys.argv[1:4])
direc = float(sys.argv[4]) if len(sys.argv) > 4 else -1.0
speed = float(sys.argv[5]) if len(sys.argv) > 5 else 0.15
yaw = math.pi if direc < 0 else 0.0
rospy.init_node('probe_wall', anonymous=True)
pub = rospy.Publisher('/desired_state', MultiDOFJointTrajectoryPoint, queue_size=1)
state = {'p': None, 'hit': None}
rospy.Subscriber('/current_state', Odometry, lambda m: state.__setitem__('p', m.pose.pose.position))


def on_col(m):
    if 'Floor' not in m.object_name and state['hit'] is None and state['p'] is not None:
        p = state['p']
        state['hit'] = (m.object_name, p.x, p.y, p.z)


rospy.Subscriber('/tesse/collision', CollisionStats, on_col)


def send(x):
    pt = MultiDOFJointTrajectoryPoint()
    tr = Transform()
    tr.translation.x, tr.translation.y, tr.translation.z = x, y, z
    tr.rotation.z, tr.rotation.w = math.sin(yaw / 2), math.cos(yaw / 2)
    pt.transforms.append(tr)
    pt.velocities.append(Twist())
    pt.accelerations.append(Twist())
    pub.publish(pt)


time.sleep(1.0)
env = Env()
# Unity frame: (x, y_up, z) = (x_ros, z_ros, y_ros); yaw about the vertical axis
qy = -math.sin(yaw / 2)
for _ in range(3):
    send(x0)
    env.send(Reposition(x0, z, y, 0.0, qy, 0.0, math.cos(yaw / 2)))
    time.sleep(0.2)
t_end = time.time() + 6.0
while time.time() < t_end:          # settle in hover
    send(x0)
    time.sleep(0.01)
x = x0
while state['hit'] is None and abs(x - x0) < 8.0 and not rospy.is_shutdown():
    x += direc * speed * 0.01
    send(x)
    time.sleep(0.01)
p = state['p']
print('start x %.2f, set point x %.2f, drone at (%.2f, %.2f, %.2f)' % (x0, x, p.x, p.y, p.z))
print('hit:', state['hit'])
for _ in range(100):                # back off
    send(x - direc * 1.0)
    time.sleep(0.01)
