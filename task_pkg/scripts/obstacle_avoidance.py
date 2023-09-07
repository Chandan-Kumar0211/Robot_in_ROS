#! /usr/bin/env python3

import rospy

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
pub = None

def callback_laser(msg):
  # Since out of 720 we choose only five samples
  regions = {
        'right':  min(min(msg.ranges[0:143]), 30),
        'fright': min(min(msg.ranges[144:287]), 30),
        'front':  min(min(msg.ranges[288:431]), 30),
        'fleft':  min(min(msg.ranges[432:575]), 30),
        'left':   min(min(msg.ranges[576:719]), 30),
    }
  
  take_action(regions)
  
def take_action(regions):
  threshold_dist = 2.2
  # threshold_fright = 1.5
  # threshold_fleft = 1.5
  linear_speed = 2.0
  angular_speed = 1.0

  msg = Twist()
  linear_x = 0
  angular_z = 0
  
  state_description = ''
  
  if regions['front'] > threshold_dist and regions['fleft'] > threshold_dist and regions['fright'] > threshold_dist:
    state_description = 'case 1 - no obstacle'
    linear_x = linear_speed
    angular_z = 0
  elif regions['front'] < threshold_dist and regions['fleft'] > threshold_dist and regions['fright'] > threshold_dist:
    state_description = 'case 2 - front'
    linear_x = 0
    angular_z = angular_speed
  elif regions['front'] > threshold_dist and regions['fleft'] > threshold_dist and regions['fright'] < threshold_dist:
    state_description = 'case 3 - fright'
    linear_x = 0
    angular_z = -angular_speed
  elif regions['front'] > threshold_dist and regions['fleft'] < threshold_dist and regions['fright'] > threshold_dist:
    state_description = 'case 4 - fleft'
    linear_x = 0
    angular_z = angular_speed
  elif regions['front'] < threshold_dist and regions['fleft'] > threshold_dist and regions['fright'] < threshold_dist:
    state_description = 'case 5 - front and fright'
    linear_x = 0
    angular_z = -angular_speed
  elif regions['front'] < threshold_dist and regions['fleft'] < threshold_dist and regions['fright'] > threshold_dist:
    state_description = 'case 6 - front and fleft'
    linear_x = 0
    angular_z = angular_speed
  elif regions['front'] < threshold_dist and regions['fleft'] < threshold_dist and regions['fright'] < threshold_dist:
    state_description = 'case 7 - front and fleft and fright'
    linear_x = -linear_speed
    angular_z = angular_speed # Increase this angular speed for avoiding obstacle faster
  elif regions['front'] > threshold_dist and regions['fleft'] < threshold_dist and regions['fright'] < threshold_dist:
    state_description = 'case 8 - fleft and fright'
    linear_x = linear_speed
    angular_z = 0
  else:
    state_description = 'unknown case'
    rospy.loginfo(regions)

  rospy.loginfo(state_description)
  msg.linear.x = linear_x
  msg.angular.z = angular_z
  pub.publish(msg)

def main():
  global pub
  
  rospy.init_node('avoid_obstacle')
  
  pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
  
  sub = rospy.Subscriber('/scan', LaserScan, callback_laser)
  
  rospy.spin()

if __name__ == '__main__':
  main()