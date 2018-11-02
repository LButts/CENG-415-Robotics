#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node

from time import *
from std_msgs.msg import Float32
from std_msgs.msg import ByteMultiArray
from geometry_msgs.msg import Pose2D
from veranda.SimTimer import SimTimer

from math import *
from struct import *

# Globals
x, y, theta = 0.0, 0.0, 0.0 # gps loc of robot
goalX = 7
goalY = -8
r = 0.5 # radius of wheel (m)
l = 0.6 # half the distance between wheels (m)
dtheta = 0.0
rps = 0.5

# Callbacks/Veranda Sensors
##################################################################

# GPS
def get_position(message):
    global x, y, theta, dtheta
    #print("Robot is at (" + str(message.x) + "," + str(message.y) + ") facing " + str((message.theta*180/pi) % 360) + " degrees")
    #print("----------------")
    x = message.x
    y = message.y
    theta = message.theta
    dtheta_calc()

# Touch Sensor
def get_hit(message):
    global timer_handle
    simTime.destroy_timer(timer_handle)

    #hits = message.data
    #for i in range(len(hits)):
    #    hit = unpack('b', hits[i])[0]
    #    if hit != 0: # hit detected
    print("_________________________")
    print("I'm Hit I'm Hit I'm Hit")
    print("_________________________")

    timer_handle = simTime.create_timer(0.15, back_up)


##################################################################
# Functions:
##################################################################

def move_to_goal():
    global timer_handle
    simTime.destroy_timer(timer_handle)

    msgl = Float32() # left wheel msg
    msgr = Float32() # right wheel msg

    if ( (x < goalX + 3) and (x > goalX - 3) and
        (y < goalY + 3) and (y > goalY - 3) ) :

        print("********************************************")
        print("MISSION COMPLETE!!!")
        print("MISSION COMPLETE!!!")
        print("MISSION COMPLETE!!!")
        print("********************************************")
        msgl.data = 0.0
        msgr.data = 0.0
        publeft.publish(msgl)
        pubright.publish(msgr)

        node.destroy_node()
        rclpy.shutdown()


    print("_________________________")
    print("move_to_goal")
    print("_________________________")

    t = 0.1
    msgl.data = 3.0
    msgr.data = 3.0
    publeft.publish(msgl)
    pubright.publish(msgr)

    timer_handle = simTime.create_timer(0.1, move_to_goal)

def back_up() :
    global publeft, pubright, timer_handle
    simTime.destroy_timer(timer_handle)

    print("___________________________")
    print("back_up")
    print("___________________________")

    msgl = Float32() # left wheel msg
    msgr = Float32() # right wheel msg

    t = 0.1
    msgl.data = -2.0
    msgr.data = -2.0
    publeft.publish(msgl)
    pubright.publish(msgr)

    timer_handle = simTime.create_timer(t, turn_right)

def turn_right() :
    global publeft, pubright, timer_handle
    simTime.destroy_timer(timer_handle)

    msgl = Float32() # left wheel msg
    msgr = Float32() # right wheel msg

    print("_________________________")
    print("turn_right")
    print("_________________________")

    # turn 90
    rps = 0.5 # phidot1 = -phidot2
    t = (pi / 2) * l / (r*rps) # pivot time
    msgl.data = rps
    msgr.data = -rps
    publeft.publish(msgl)
    pubright.publish(msgr)
    timer_handle = simTime.create_timer(t, motion_forward)

def motion_forward():
    global publeft, pubright, timer_handle
    simTime.destroy_timer(timer_handle)

    msgl = Float32() # left wheel msg
    msgr = Float32() # right wheel msg

    print("_________________________")
    print("Move forward a LITTLE")
    print("_________________________")

    t = 1.5
    msgl.data = 2.0
    msgr.data = 2.0
    publeft.publish(msgl)
    pubright.publish(msgr)

    timer_handle = simTime.create_timer(t, face_goal)

def face_goal():
    global publeft, pubright, rps, timer_handle
    simTime.destroy_timer(timer_handle)
    dtheta_calc()

    msgl = Float32() # left wheel msg
    msgr = Float32() # right wheel msg

    print("_________________________")
    print("face_goal")
    print("_________________________")

    rps = 0.5
    print("dtheta: " + str(dtheta))
    t = dtheta * l / (r*rps) # pivot time to face goal
    print("t = " + str(t))
    msgl.data = -rps
    msgr.data = rps
    publeft.publish(msgl)
    pubright.publish(msgr)

    timer_handle = simTime.create_timer(t, move_to_goal)

def dtheta_calc():
    global dtheta, theta

    if theta < 0 :
        theta += 2.0 * pi
    if theta > 2.0 * pi :
        theta -= 2.0 * pi

    dtheta = atan2((goalY - y), (goalX - x))

    if dtheta < 0 :
        dtheta += 2.0 * pi
    if dtheta > 2.0 * pi :
        dtheta -= 2.0 * pi

    dtheta = dtheta - theta

    if dtheta < 0 :
        dtheta += 2.0 * pi
    if dtheta > 2.0 * pi :
        dtheta -= 2.0 * pi

##################################################################
# Setup Robot & Sensors
rclpy.init()
node = Node("BM_node")
simTime = SimTimer(True, 'veranda/timestamp', node)
publeft = node.create_publisher(Float32, 'robot1/left')
pubright = node.create_publisher(Float32, 'robot1/right')
gps = node.create_subscription(Pose2D, 'robot1/gps', get_position)
subtouches = node.create_subscription(ByteMultiArray, 'robot1/touches', get_hit)
##################################################################
# Main Code:
##################################################################

timer_handle = simTime.create_timer(0.1, face_goal)

rclpy.spin(node)

# Clean up and End Program
node.destroy_node()
rclpy.shutdown()
