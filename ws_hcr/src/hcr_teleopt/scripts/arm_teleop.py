#!/usr/bin/env python
# Initial code created by Graylin Trevor Jay (tjay@cs.brown.edu) and published under Creative Commons Attribution license.
# addition for signal interrupt by Koen Buys

import roslib
import rospy
#from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
import sys, select, termios, tty, signal

msg = """
Reading from the keyboard  and Publishing to Twist!
---------------------------
Moving around:
   'q' 'w' (4)      'i' 'o' (3)
    'a' 's' (5)      'k' 'l' (2)
     'y'  'x' (6)     ',' '.' (1)

CTRL-C to quit
"""

# x,y,theta ratio
moveBindings = {
',':(1,0,0,0,0,0),  
'.':(-1,0,0,0,0,0), 
'k':(0,1,0,0,0,0),
'l':(0,-1,0,0,0,0),
'i':(0,0,1,0,0,0),
'o':(0,0,-1,0,0,0),
'q':(0,0,0,1,0,0),
'w':(0,0,0,-1,0,0),
'a':(0,0,0,0,1,0),
's':(0,0,0,0,-1,0),
'y':(0,0,0,0,0,1),
'x':(0,0,0,0,0,-1),
}

speedBindings = {
'q':(1.1, 1.1),
'z':(.9, .9),
'w':(1.1, 1),
'x':(.9, 1),
'e':(1, 1.1),
'c':(1, .9),
}

class TimeoutException(Exception): 
    pass 

def getKey():
    def timeout_handler(signum, frame):
        raise TimeoutException()
    old_handler = signal.signal(signal.SIGALRM, timeout_handler)
    signal.alarm(1)  # this is the watchdog timing
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    try:
       key = sys.stdin.read(1)
       # print "Read key"
    except TimeoutException:
       # print "Timeout"
       return "-"
    finally:
       signal.signal(signal.SIGALRM, old_handler)
    signal.alarm(0)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

speed = 0.1
turn = 0.1

def vels(speed, turn):
    return "currently:\tspeed %s\tturn %s " % (speed, turn)

if __name__ == "__main__":
    settings = termios.tcgetattr(sys.stdin)
    pub1 = rospy.Publisher('/joint1_position_controller/command', Float64)
    pub2 = rospy.Publisher('/joint2_position_controller/command', Float64)
    pub3 = rospy.Publisher('/joint3_position_controller/command', Float64)
    pub4 = rospy.Publisher('/joint4_position_controller/command', Float64)
    pub5 = rospy.Publisher('/joint5_position_controller/command', Float64)
    pub6 = rospy.Publisher('/joint6_position_controller/command', Float64)



    rospy.init_node('teleop_keyboard')
    x = 0 # arm_1
    x2 = 0  #arm_3
    x3 = 0
    x4 = 0
    x5 = 0
    x6 = 0
    th = 1
    th2 = 0.8
    th3 = 0
    th4 = 0.8
    th5 = 1
    th6 = 0.3
    
   
    #status = 0
    try:
        print msg
        print vels(speed, turn)
        while(1):
            key = getKey()
            if key in moveBindings.keys():
                x = moveBindings[key][0]
                th = th + x * 0.05
                #y = moveBindings[key][1]
                #th = th - y * 0.1
                x2 = moveBindings[key][1]
                th2 = th2 + x2 * 0.05



                x3 = moveBindings[key][2]
                th3 = th3 + x3 * 0.05
               # y3 = moveBindings[key][4]
                #th3 = th3 - y3 * 0.1

                x4 = moveBindings[key][3]
                th4 = th4 - x4 * 0.05
                #y4 = moveBindings[key][6]
                #th4 = th4 - y4 * 0.1

                x5 = moveBindings[key][4]
                th5 = th5 + x5 * 0.05
                x6 = moveBindings[key][5]
                th6 = th6 - x6 * 0.05
             
               
            #elif key in speedBindings.keys():
             #   speed = speed * speedBindings[key][0]
              #  turn = turn * speedBindings[key][1]
              #  print vels(speed, turn)
               # if (status == 14):
                #    print msg
                #status = (status + 1) % 15
            else:
                #x = 0
                #th2 = 0.5
                #x3 = 0
                #th4 = 3.2
               # y = 0
               # th = 0
                if (key == '\x03'):
                    break
            arm1 = Float64()
            arm1.data = th 
            arm2 = Float64()
            arm2.data = th2
            arm3 = Float64()
            arm3.data = th3
            arm4 = Float64()
            arm4.data = th4
            arm5 = Float64()
            arm5.data = th5
            arm6 = Float64()
            arm6.data = th6
            
            pub1.publish(arm1)
            pub2.publish(arm2)
            pub3.publish(arm3)
            pub4.publish(arm4)
            pub5.publish(arm5)
            pub6.publish(arm6)

    except:
        print e
    finally:
        float64 = Float64()
        float64.data = 0;
        joint2 = Float64()
        joint2.data = 0.5; 
        joint4 = Float64()
        joint4.data = 3.2;
        joint5 = Float64()
        joint5.data = 1;
        joint6 = Float64()
        joint6.data = 2.3;
        pub1.publish(float64)
        pub2.publish(joint2)
        pub3.publish(float64)
        pub4.publish(arm4)
        pub5.publish(joint5)
        pub6.publish(joint6)

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
