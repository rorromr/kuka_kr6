#!/usr/bin/env python
import rospy
from std_msgs.msg import Bool

class Led(object):
    def __init__(self, topic = '/gripper_cmd'):
        self.pub = rospy.Publisher('/gripper_cmd', Bool, queue_size=10)
        self.msg = Bool()

    def set(self, status = False):
        self.msg.data = status
        self.pub.publish(self.msg)

    def on(self):
        self.set(True)

    def off(self):
        self.set(False)

        

def main():  
    rospy.init_node('gripper_commander', anonymous=True)
    rate = rospy.Rate(1) # 1hz
    led = Led()

    while not rospy.is_shutdown():
        led.on()
        rate.sleep()
        led.off()
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
