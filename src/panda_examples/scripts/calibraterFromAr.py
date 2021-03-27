#!/usr/bin/env python

'''
This script converts output from haf_grasping into input for our panda controller.
Top grasping.

Returns 10 values, first 3 being the location of the first target, next 3 being the location of the next target where the actual grasping will occur, last 4 being the orientation of the gripper.
'''
import rospy
import ros_numpy
from ar_track_alvar_msgs.msg import AlvarMarkers
import numpy as np
import math

class Calibrate():
    def __init__(self, scan_topic="/ar_pose_marker"):
        self.markers = [11,12,13,101,101]
        self.count = [0,0,0,0,0]
        self.cumu_pos_marker_0 = [0,0,0]
        self.cumu_pos_marker_1 = [0,0,0]
        self.cumu_pos_marker_2 = [0,0,0]
        self.cumu_pos_marker_3 = [0,0,0]
        self.cumu_ori_marker_4 = [0,0,0] #xyz without w
    	rospy.init_node('calibrate', anonymous=True)
        self.scan_sub = rospy.Subscriber(scan_topic, AlvarMarkers, self.on_scan)
        rospy.spin()


    def on_scan(self, scan):
        #vec = [float(i) for i in scan.data.split()]
        #print(vec)
        #print(scan.markers[0])
        for i in scan.markers:
            if i.id == self.markers[0]:
                self.cumu_pos_marker_0[0] += i.pose.pose.position.x
                self.cumu_pos_marker_0[1] += i.pose.pose.position.y
                self.cumu_pos_marker_0[2] += i.pose.pose.position.z
                self.count[0] += 1.0
                print("ID "+str(int(i.id))+" over "+str(self.count[0])+" points: "+str(np.array(self.cumu_pos_marker_0)/self.count[0]))
            elif i.id == self.markers[1]:
                self.cumu_pos_marker_1[0] += i.pose.pose.position.x
                self.cumu_pos_marker_1[1] += i.pose.pose.position.y
                self.cumu_pos_marker_1[2] += i.pose.pose.position.z
                self.count[1] += 1.0
                print("ID "+str(int(i.id))+" over "+str(self.count[1])+" points: "+str(np.array(self.cumu_pos_marker_1)/self.count[1]))
            elif i.id == self.markers[2]:
                self.cumu_pos_marker_2[0] += i.pose.pose.position.x
                self.cumu_pos_marker_2[1] += i.pose.pose.position.y
                self.cumu_pos_marker_2[2] += i.pose.pose.position.z
                self.count[2] += 1.0
                print("ID "+str(int(i.id))+" over "+str(self.count[2])+" points: "+str(np.array(self.cumu_pos_marker_2)/self.count[2]))
            elif i.id == self.markers[3]:
                self.cumu_pos_marker_3[0] += i.pose.pose.position.x
                self.cumu_pos_marker_3[1] += i.pose.pose.position.y
                self.cumu_pos_marker_3[2] += i.pose.pose.position.z
                self.count[3] += 1.0
                print("Position of ID "+str(int(i.id))+" over "+str(self.count[3])+" points: "+str(np.array(self.cumu_pos_marker_3)/self.count[3]))
                self.cumu_ori_marker_4[0] += i.pose.pose.orientation.x
                self.cumu_ori_marker_4[1] += i.pose.pose.orientation.y
                self.cumu_ori_marker_4[2] += i.pose.pose.orientation.z
                print("Orientation of ID "+str(int(i.id))+" over "+str(self.count[3])+" points: "+str(np.array(self.cumu_ori_marker_4)/self.count[3]))
                print("w: "+str((1-i.pose.pose.orientation.x**2-i.pose.pose.orientation.y**2-i.pose.pose.orientation.z**2)**0.5))
            else:
                print("")
        v11_12 = np.array(self.cumu_pos_marker_1)/self.count[1] - np.array(self.cumu_pos_marker_0)/self.count[0]
        v11_13 = np.array(self.cumu_pos_marker_2)/self.count[2] - np.array(self.cumu_pos_marker_0)/self.count[0]

        #print(np.cross(v11_12,v11_13))
        normalvec = np.cross(v11_13,v11_12)/np.linalg.norm(np.cross(v11_13,v11_12))
        print("Plane parameters are: ")
        print(normalvec)
        print(-np.dot(np.array(self.cumu_pos_marker_0)/self.count[0],normalvec))


if __name__ == '__main__':
    try:
        Calibrate()
    except rospy.ROSInterruptException:
        pass
