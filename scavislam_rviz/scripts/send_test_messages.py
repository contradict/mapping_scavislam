#!/usr/bin/env python

from scavislam_messages.msg import SLAMGraph, Vertex
from geometry_msgs.msg import Point, Pose, Quaternion
from std_msgs.msg import UInt32
import rospy
import numpy as np
import tf

def generate():
    sg = SLAMGraph()
    sg.header.frame_id = "/base_link"
    xs=np.linspace(0,10,11)
    positions = np.vstack((
        xs,
        np.sin(2*np.pi*xs/10),
        np.zeros_like(xs)))
    headings = np.r_[0,np.arctan2(np.diff(positions[1,:]), np.diff(positions[0,:]))]
    orientations = np.array(
            map(lambda yaw:Quaternion(*tf.transformations.quaternion_from_euler(0,0,yaw)),
                headings))
    poses=[ Pose(Point(*p), o) for p,o in zip(positions.T, orientations)]
    neighbors=[(1,)]+[ (x-1, x+1) for x in range(1,len(xs)-1)]+[(9,)]
    neighbors[5] = (3,4,6,8)
    points=[[0],[0]]
    for x in xs:
        ps=np.random.random((2,10))
        ps[1,:] -= 0.5
        ps[1,:] *= np.linspace(0,6,10)
        ps[0,:] *= 5
        ps[0,:] += x
        points = np.c_[points,ps]
    sg.points = [Point(*p) for p in np.r_[points, np.zeros((1,points.shape[1]))].T]

    point_indices=[]
    for i in range(len(xs)):
        pi = np.random.randint(10*i, 110, 8+np.random.randint(12))
        point_indices += [list(pi)]

    sg.vertices=[ Vertex(pose, [UInt32(n) for n in nbs], [UInt32(p) for p in pts]) for pose,nbs,pts in zip(poses, neighbors, point_indices)]

    return sg

def run(once=False):
    topic = 'test_slamgraph'
    publisher = rospy.Publisher( topic, SLAMGraph, latch=once )

    rospy.init_node( 'test_slamgraph' )

    br = tf.TransformBroadcaster()
    rate = rospy.Rate(10)

    sg=generate()
    seq = 0;
    while not rospy.is_shutdown():

        sg.header.stamp = rospy.Time.now()
        sg.header.seq=seq
        seq += 1

        publisher.publish( sg )

        br.sendTransform((0, 0, 0),
                         tf.transformations.quaternion_from_euler(0, 0, 0),
                         rospy.Time.now(),
                         "base_link",
                         "map")
        rate.sleep()
        if once:
            rospy.sleep(3)
            break;

run(True)
