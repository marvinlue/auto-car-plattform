#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from ackermann_msgs.msg import AckermannDriveStamped

class Car_Controller():

    def __init__(self):
        rospy.init_node('Car_Control', anonymous=True)
        self.subArm = rospy.Subscriber('Car_Armed', Bool, self.arm_callback)
        self.subDrive = rospy.Subscriber('Desired_Ackermann_Drive', AckermannDriveStamped, self.ackermann_callback)
        self.pubDrive = rospy.Publisher('Ackermann_Drive', AckermannDriveStamped, queue_size=10)
        self.rate = rospy.Rate(10)
        self.armed = False
        self.lastArmMsgTime = rospy.Time.now()
        self.lastAckerStampedMsg = AckermannDriveStamped()
        self.neutralAckermannStamped = AckermannDriveStamped()

    def arm_callback(self, data):
        self.armed = data.data
        self.lastArmMsgTime = rospy.Time.now()

    def ackermann_callback(self, data):
        self.lastAckerStampedMsg = data
        
    def checkArmedMsg(self, time):
        diff = (time - self.lastArmMsgTime).to_sec()
        return ((diff < 1) and self.armed)

    def checkAckerMsg(self, time):
        diff = (time - self.lastAckerStampedMsg.header.stamp).to_sec()
        return (diff < 1)

    def carControl(self):
        time = rospy.Time.now()
        if self.checkArmedMsg(time) and self.checkAckerMsg(time) :
            self.pubDrive.publish(self.lastAckerStampedMsg)
        else:
            self.neutralAckermannStamped.header.stamp = rospy.Time.now()
            self.pubDrive.publish(self.neutralAckermannStamped)
            
    
    def run(self):
       while (not rospy.is_shutdown()):
           self.carControl()
           self.rate.sleep() 

if __name__ == '__main__':
    try:
        node = Car_Controller()
        node.run()
    except rospy.ROSInterruptException:
        pass