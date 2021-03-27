#!/usr/bin/env python

'''
This script converts output from haf_grasping into input for our panda controller.
Top grasping.

Returns 10 values, first 3 being the location of the first target, next 3 being the location of the next target where the actual grasping will occur, last 4 being the orientation of the gripper.
'''
import rospy
import ros_numpy
from std_msgs.msg import String, Float32MultiArray
import numpy as np
import math
from tf.transformations import quaternion_from_euler

class GraspHypo():
    def __init__(self, scan_topic="/haf_grasping/grasp_hypothesis_with_eval"):
        self.success_threshold=30
        self.ee_to_base_finger_distance = 0.058 #distance in z-axis.
        self.base_finger_to_object_distance = 0.02 #to be manually adjusted
        self.distance_away_from_object = 0.05 #first trajectory should be 5cm away from object, by the approach vector.
    	rospy.init_node('grasphypo', anonymous=True)
        self.scan_sub = rospy.Subscriber(scan_topic, String, self.on_scan)
        self.pub = rospy.Publisher('target', Float32MultiArray, queue_size=10)
        #self.printjunk()
        rospy.spin()

    def printjunk(self):
        ######DEBUGGING
        self.rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            rospy.loginfo("Published array")
            self.pub.publish(Float32MultiArray(data=np.array([1,1,1,2,2,2,0,0,0,1])))
            self.rate.sleep()


    def on_scan(self, scan):
        vec = [float(i) for i in scan.data.split()]
        #print(vec)
        self.process_grasp_hypo(vec)
        
    def process_grasp_hypo(self, vec):
        if vec[0] > self.success_threshold:
            blah = vec
            self.approach_vector = np.array(vec[7:10])/np.linalg.norm(np.array(vec[7:10]))
            self.first_location = np.array(vec[10:13]) + self.approach_vector * (self.ee_to_base_finger_distance + self.base_finger_to_object_distance + self.distance_away_from_object)
            self.second_location = np.array(vec[10:13]) + self.approach_vector * (self.ee_to_base_finger_distance + self.base_finger_to_object_distance)
            self.orientation = quaternion_from_euler(-math.pi, 0, math.pi*vec[13]/180.0-math.pi/4, axes='rxyz') ### when output of haf_grasping is 0 for gripper arm orientation, panda's position is actually at -pi/4.
            rospy.loginfo("Published target pose")
            self.pub.publish(Float32MultiArray(data=np.concatenate((self.first_location,self.second_location,self.orientation),axis=0)))
        elif vec[0] < -1:
            rospy.loginfo("No grasp detected")
        else:
            rospy.loginfo("Predicted success rate below 30%")

if __name__ == '__main__':
    try:
        GraspHypo()
    except rospy.ROSInterruptException:
        pass
