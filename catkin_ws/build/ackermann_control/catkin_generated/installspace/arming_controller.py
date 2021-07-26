#!/usr/bin/env python3

import rospy
from std_msgs.msg import Bool
from ackermann_msgs.msg import AckermannDrive
from pynput.keyboard import Listener, Key

class Arming_Controller():

    def on_press(self, key):
        if key == Key.enter:  # Write the character pressed if available
            self.armed = True    
        elif key == Key.space:  # If space was pressed, write a space
            self.armed = False

    def __init__(self):
        self.pub = rospy.Publisher('Car_Armed', Bool, queue_size=10)
        rospy.init_node('Arm_Control', anonymous=True)
        self.rate = rospy.Rate(5) # 5hz
        self.armed = False
        self.listener = Listener(on_press=self.on_press)
        self.listener.start()

    def run(self):
       while (not rospy.is_shutdown()):
           self.pub.publish(self.armed)
           self.rate.sleep() 

if __name__ == '__main__':
    try:
        node = Arming_Controller()
        node.run()
    except rospy.ROSInterruptException:
        pass
