#!/usr/bin/env python

import rospy
import math

from std_msgs.msg import Bool
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PoseStamped

class Aruco_Tracker():

    def __init__(self):
        rospy.init_node('Aruco_Tracker', anonymous=True)
        self.subPose = rospy.Subscriber('single_localizer/pose', PoseStamped, self.pose_callback)
        self.pubDrive = rospy.Publisher('Desired_Ackermann_Drive', AckermannDriveStamped, queue_size=10)
        self.rate = rospy.Rate(10)
        self.lastPoseMsgTime = rospy.Time.now()
        self.lastPose = PoseStamped()
        self.neutralAckermannStamped = AckermannDriveStamped()
        
        self.stop = 2
        self.speedlimit = 0.5

    def pose_callback(self, pose):
        self.lastPose = pose

    def checkPoseMsg(self, time):
        diff = (time - self.lastPose.header.stamp).to_sec()
        rospy.loginfo(diff)
        return (diff < self.stop)

    def tracking(self):
        time = rospy.Time.now()
        if self.checkPoseMsg(time) :
            x = self.lastPose.pose.position.x
            z = self.lastPose.pose.position.z
            angle = math.degrees(math.atan(x/z))
            if abs(angle) < 2:
                angle = 0

            if z < 0.30 :
                speed = 0
            else:
                speed = self.speedlimit
            msg = AckermannDriveStamped()
            msg.drive.steering_angle = angle
            msg.drive.speed = speed
            msg.header.stamp = time
            
            self.pubDrive.publish(msg)
        else:
            self.neutralAckermannStamped.header.stamp = rospy.Time.now()
            self.pubDrive.publish(self.neutralAckermannStamped)
            rospy.loginfo("too late")
            
    def run(self):
       while (not rospy.is_shutdown()):
           self.tracking()
           self.rate.sleep() 

if __name__ == '__main__':
    try:
        node = Aruco_Tracker()
        node.run()
    except rospy.ROSInterruptException:
        pass