#! /usr/bin/env python
# -*- coding: utf-8 -*-
#
# Released under the BSD License.import sys

import rospy
import sys
from geometry_msgs.msg import Twist, Vector3

class Megarover_Change_Speed_Angular():
    def __init__(self):
        # Create twist publisher:
        rospy.init_node('megarover_change_speed_angular')
        self._pub_cmd = rospy.Publisher('rover_twist', Twist, queue_size=100)

        # Initialize twist components to zero:
        self._linear_x  = 0.0
        self._angular_z = 0.0

    def get_speed(self):
        print("speed = %f")%self._linear_x
        return self._linear_x
    
    def set_speed(self, speed_p):
        self._linear_x  = speed_p
        print("set speed = %f")%self._linear_x
    
    def get_angular(self):
        print("angular = %f")%self._angular_z
        return self._angular_z
    
    def set_angular(self, angular_p):
        self._angular_z = angular_p
        print("set angular = %f")%self._angular_z
    
    def _send_twist(self):
        linear  = Vector3(self._linear_x, 0.0, 0.0)
        angular = Vector3(0.0, 0.0, self._angular_z)
        r = rospy.Rate(10)
        twist = Twist(linear, angular)
        for i in range(0, 5):
            self._pub_cmd.publish(twist)
            r.sleep()
        print(twist)
    
    speed = property(get_speed, set_speed)
    angular = property(get_angular, set_angular)

def usage():
    return "%s [speed(m/s)] [angular(rad/s)]"%sys.argv[0]

if __name__ == '__main__':
    try:
        if len(sys.argv) == 3:
            mcsa = Megarover_Change_Speed_Angular()
            mcsa.set_speed(float(sys.argv[1]))
            mcsa.set_angular(float(sys.argv[2]))
            mcsa._send_twist()
        else:
            print(usage())
            sys.exit(1)
    except rospy.ROSInterruptException:
        pass
