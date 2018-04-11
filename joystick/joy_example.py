#!/usr/bin/env python

import rospy
from turtlesim.srv import SetPen, TeleportAbsolute
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty
from turtlesim.msg import Pose
from pylab import *

circ = False

setpen = rospy.ServiceProxy('set_pen', SetPen)
clear = rospy.ServiceProxy('/clear', Empty)
clear()
teleport = rospy.ServiceProxy('teleport_absolute', TeleportAbsolute)

rospy.init_node('joy_bridge')

twist = Twist()
cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=100)

def move(v, w):
    twist.linear.x = v
    twist.angular.z = w
    cmd_pub.publish(twist)


def pen(r, g, b, w, on):
    setpen(r, g, b, w, not on)

axes = [0 for i in range(10)]
buttons = [0 for i in range(10)]

def joy_callback(msg):
    global axes
    global buttons
    axes = msg.axes
    buttons = msg.buttons

x = 0
y= 0


def pose_callback(msg):
    global x
    global y
    x = msg.x
    y = msg.y    
    
def circle(x, y, r):
    angle = linspace(-pi, pi, 100)
    x = x + r*cos(angle)
    y = y + r*sin(angle)
    pen(0,255,0,False)
    teleport(x[0], y[0], 0)
    pen(0,0,0,2, True)
    for i in range(100):
        teleport(x[i], y[i], angle[i]+pi/2)

rospy.Subscriber('joy', Joy, joy_callback)
rospy.Subscriber('pose', Pose, pose_callback)

m = 5.5
n = 5.5
r = 5
clear()
if circ:
    circle(m, n, r)


while not rospy.is_shutdown():

    print('Axes: {}'.format(axes))
    print('Buttons: {}'.format(buttons))
    print('Robot @ ({}, {})'.format(x, y))
    
    if buttons[3]:
        clear()
       
    if  buttons[1] or buttons[0] or buttons[2]:
        # button 5 to line width
        w = 2 + (1-axes[5])*4
        print w
        pen(buttons[1]*255,buttons[0]*255,buttons[2]*255,w, True)
    else:
        pen(0,0,0,2, False)

    move(axes[1]*6,axes[0]*5)

    rospy.sleep(0.01)
    






