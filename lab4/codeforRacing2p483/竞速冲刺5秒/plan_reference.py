#!/usr/bin/env python3
"""Execute a saved optimized reference against real TESSE feedback.

This publishes desired states only. It never publishes odometry or changes
simulator clocks, gate poses, collisions or physical parameters.
"""
import bisect
import csv
import math
from pathlib import Path
import shutil
import rospy
from geometry_msgs.msg import Transform, Twist, Vector3Stamped
from nav_msgs.msg import Odometry
from trajectory_msgs.msg import MultiDOFJointTrajectoryPoint


def load_plan(path):
    with Path(path).open(newline='') as handle:
        rows = [{k: float(v) for k, v in r.items()} for r in csv.DictReader(handle)]
    required = ('t','x','y','z','vx','vy','vz','ax','ay','az')
    if len(rows) < 2 or any(not all(k in r and math.isfinite(r[k]) for k in required) for r in rows):
        raise ValueError('Plan needs finite t,p,v,a rows')
    if abs(rows[0]['t']) > 1e-9 or any(b['t'] <= a['t'] for a,b in zip(rows,rows[1:])):
        raise ValueError('Plan must start at zero and increase strictly')
    if any(b['t']-a['t'] < 1e-6 for a,b in zip(rows,rows[1:])):
        raise ValueError('Plan knots closer than 1 microsecond are ill-conditioned')
    return rows


class Reference:
    def __init__(self, rows):
        self.rows = rows
        self.times = [r['t'] for r in rows]
        self.coefficients = []
        # Quintic Hermite interpolation consistently matches p,v,a at both
        # ends. It does not modify the offline optimized samples.
        for a,b in zip(rows,rows[1:]):
            h=b['t']-a['t']; axes=[]
            for p,v,acc in zip(('x','y','z'),('vx','vy','vz'),('ax','ay','az')):
                c0=a[p]; c1=h*a[v]; c2=.5*h*h*a[acc]
                d=b[p]-c0-c1-c2; e=h*b[v]-c1-2*c2; f=h*h*b[acc]-2*c2
                axes.append((c0,c1,c2,10*d-4*e+.5*f,-15*d+7*e-f,6*d-3*e+.5*f))
            self.coefficients.append((h,axes))

    def sample(self,t):
        if t >= self.times[-1]:
            r=self.rows[-1]
            return [[r[k] for k in keys] for keys in (('x','y','z'),('vx','vy','vz'),('ax','ay','az'))]
        i=max(0,bisect.bisect_right(self.times,t)-1)
        h,axes=self.coefficients[i]; u=(t-self.times[i])/h
        p=[]; v=[]; acc=[]
        for c in axes:
            p.append(sum(c[j]*u**j for j in range(6)))
            v.append(sum(j*c[j]*u**(j-1) for j in range(1,6))/h)
            acc.append(sum(j*(j-1)*c[j]*u**(j-2) for j in range(2,6))/(h*h))
        return p,v,acc


def main():
    rospy.init_node('sprint_offline_optimized_reference')
    source=Path(rospy.get_param('~plan'))
    rows=load_plan(source); ref=Reference(rows)
    current=rospy.wait_for_message('/current_state',Odometry,timeout=10)
    p=current.pose.pose.position
    mismatch=math.sqrt(sum((value-rows[0][key])**2 for value,key in zip((p.x,p.y,p.z),('x','y','z'))))
    if mismatch > .08:
        raise ValueError('Refusing reference: actual start differs by %.4f m' % mismatch)
    q=current.pose.pose.orientation
    yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
    destination=rospy.get_param('~dump_file','')
    if destination and source.resolve()!=Path(destination).resolve():
        shutil.copyfile(source,destination)
    publisher=rospy.Publisher('/desired_state',MultiDOFJointTrajectoryPoint,queue_size=1)
    preview_s=float(rospy.get_param('~attitude_preview_s',0.0))
    if not math.isfinite(preview_s) or not 0 <= preview_s <= .1:
        raise ValueError('Attitude preview must be within 0..0.1 simulated seconds')
    preview_publisher=rospy.Publisher('/sprint_acceleration_preview',Vector3Stamped,queue_size=1)
    rospy.sleep(.1)
    start=rospy.Time.now()
    rospy.set_param('/sprint_reference_start',start.to_sec())
    rospy.loginfo('Executing offline optimized reference %.9fs, start mismatch %.6fm',ref.times[-1],mismatch)
    def publish(event):
        t=max(0,min(ref.times[-1],(rospy.Time.now()-start).to_sec()))
        position,velocity,acceleration=ref.sample(t)
        if preview_s>0:
            preview=Vector3Stamped(); preview.header.stamp=rospy.Time.now(); preview.header.frame_id='world'
            preview.vector.x,preview.vector.y,preview.vector.z=ref.sample(t+preview_s)[2]
            preview_publisher.publish(preview)
        msg=MultiDOFJointTrajectoryPoint(); msg.time_from_start=rospy.Duration(t)
        transform=Transform(); transform.translation.x,transform.translation.y,transform.translation.z=position
        transform.rotation.z=math.sin(yaw/2); transform.rotation.w=math.cos(yaw/2)
        vel=Twist(); vel.linear.x,vel.linear.y,vel.linear.z=velocity
        acc=Twist(); acc.linear.x,acc.linear.y,acc.linear.z=acceleration
        msg.transforms=[transform]; msg.velocities=[vel]; msg.accelerations=[acc]
        try:
            publisher.publish(msg)
        except rospy.ROSException:
            if not rospy.is_shutdown():
                raise
    timer=rospy.Timer(rospy.Duration(.01),publish)
    rospy.on_shutdown(timer.shutdown)
    rospy.spin()


if __name__=='__main__':
    main()
