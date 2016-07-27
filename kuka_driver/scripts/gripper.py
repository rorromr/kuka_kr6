#!/usr/bin/env python
import rospy
from std_msgs.msg import Bool
from threading import Thread


class Gripper(object):
    def __init__(self, topic = '/gripper_cmd'):
        self.pub = rospy.Publisher('/gripper_cmd', Bool, queue_size=10)
        self.msg = Bool()
        # Publish msg on thread
        self.state_update_rate = 10
        self.is_running = True
        self.pub_thread = None

    def start(self):
        self.pub_thread = Thread(target=self.update_state)
        self.pub_thread.start()

    def set(self, status = False):
        self.msg.data = status
        
    def close(self):
        self.set(True)

    def open(self):
        self.set(False)

    def update_state(self):
        rate = rospy.Rate(self.state_update_rate) 
        while not rospy.is_shutdown() and self.is_running:  
            self.pub.publish(self.msg)
            rate.sleep()
    
    def shutdown(self):
        self.is_running=False


def main():  
    rospy.init_node('gripper_commander', anonymous=True)
    rate = rospy.Rate(1) # 1hz
    gripper= Gripper()
    gripper.start()
    while not rospy.is_shutdown():
        gripper.close()
        rospy.sleep(1.0)
        gripper.open()
        rospy.sleep(1.0)
    gripper.shutdown()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
