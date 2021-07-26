#!/usr/bin/env python3

import rospy
from ackermann_msgs.msg import AckermannDriveStamped

def Ackermann_Drive_pub():
    pub = rospy.Publisher('Desired_Ackermann_Drive', AckermannDriveStamped, queue_size=10)
    rospy.init_node('Ackermann_Drive_Pub', anonymous=True)
    rate = rospy.Rate(1) # 1hz

    max_steering_angle = 20
    max_speed = 15

    counter = 0
    
    while (not rospy.is_shutdown()):
        msg = AckermannDriveStamped()
        msg.drive.steering_angle = counter*0.1*max_steering_angle
        msg.drive.speed = - counter*0.1*max_speed
        msg.header.stamp = rospy.Time.now()
        pub.publish(msg)
        rate.sleep()

        counter = (counter + 1) % 10        

if __name__ == '__main__':
    try:
        Ackermann_Drive_pub()
    except rospy.ROSInterruptException:
        pass
