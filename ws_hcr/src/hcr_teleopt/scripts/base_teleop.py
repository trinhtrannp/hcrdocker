#!/usr/bin/env python

# Copyright (c) 2011, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#    * Neither the name of the Willow Garage, Inc. nor the names of its
#      contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.


############################################################
# Code is form turtlebot3 and adapted for HCR application! #
############################################################

import rospy

from geometry_msgs.msg import Twist

import sys, select, termios, tty

msg = """
Contorle HCR
---------------------------
Moving around:
        w
   a    s    d
        x

w/s     : increase/decrease linear velocity
a/d     : increase/decrease angular velocity
x       : force stop
space   : set angular velocity to zero

CTRL-C to quit
"""

def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def vels(target_linear_vel, target_angular_vel):
    return "currently:\tlinear vel %s\t angular vel %s " % (target_linear_vel,target_angular_vel)

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('hcr_teleop')
    pub = rospy.Publisher('/diff_drive_controller/cmd_vel', Twist, queue_size=5)

    target_linear_vel = 0
    target_angular_vel = 0
    control_linear_vel = 0
    control_angular_vel = 0

    max_linear_vel = 0.9
    max_angular_vel = 3.5

    try:
        print msg
        while(1):
            key = getKey()
            if key == 'w' :
                target_linear_vel = target_linear_vel + 0.01
                if target_linear_vel >= max_linear_vel :
                    target_linear_vel = max_linear_vel
                print vels(target_linear_vel,target_angular_vel)
            elif key == 's' :
                target_linear_vel = target_linear_vel - 0.01
                if target_linear_vel <= max_linear_vel * (-1) :
                    target_linear_vel = max_linear_vel * (-1)
                print vels(target_linear_vel,target_angular_vel)
            elif key == 'a' :
                target_angular_vel = target_angular_vel + 0.05
                if target_angular_vel > max_angular_vel :
                    target_angular_vel = max_angular_vel
                print vels(target_linear_vel,target_angular_vel)
            elif key == 'd' :
                target_angular_vel = target_angular_vel - 0.05
                if target_angular_vel < max_angular_vel * (-1) :
                    target_angular_vel = max_angular_vel * (-1)
                print vels(target_linear_vel,target_angular_vel)
            elif key == ' ' :
                target_angular_vel  = 0
                control_angular_vel = 0
                print vels(target_linear_vel,target_angular_vel)
            elif key == 'x' :
                target_linear_vel   = 0
                control_linear_vel  = 0
                target_angular_vel  = 0
                control_angular_vel = 0
                print vels(0, 0)
            else:
                if (key == '\x03'):
                    break

            control_linear_vel = target_linear_vel
            control_angular_vel = target_angular_vel

            twist = Twist()
            twist.linear.x = control_linear_vel; twist.linear.y = 0; twist.linear.z = 0
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = control_angular_vel
            pub.publish(twist)

    except:
        print e

    finally:
        twist = Twist()
        twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        pub.publish(twist)

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
