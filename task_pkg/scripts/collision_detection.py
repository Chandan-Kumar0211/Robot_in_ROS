#!/usr/bin/env python3

import rospy
from gazebo_msgs.msg import ContactsState

def contact_callback(msg):
    if len(msg.states) > 0:
        print("Collision detected!")
        print(msg.states.collision1_name)
        print(msg.states.collision2_name)


def main():
    rospy.init_node('contact_sensor_node')
    rospy.Subscriber('/rover/base_link/contact_sensor', ContactsState, contact_callback)
    rospy.spin()

if __name__ == '__main__':
    main()