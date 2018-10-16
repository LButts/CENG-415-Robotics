import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from std_msgs.msg import ByteMultiArray
from veranda.SimTimer import SimTimer
from geometry_msgs.msg import Pose2D
from struct import *
from math import *

rclpy.init()
node = Node("BM")

simTime= SimTimer(True, 'veranda/timestamp', node)

publeft = node.create_publisher(Float32, 'robot1/left')
pubright = node.create_publisher(Float32, 'robot1/right')


goal = [2000, 0, 0]
travel = [0, 0, 0]
position = [0, 0, 0]
distance = []
r = 0.5 # radius of wheel (m)
l = 0.6 # half the distance between wheels (m)
count = 0
pivotTime = 0.0
moveTime = 0.0
dTheta = 0.0
bump = 0

def updatePos(message):
    global timer_handle, position, goal, travel
    simTime.destroy_timer(timer_handle)

    position[0] = message.x
    position[1] = message.y
    position[2] = ((message.theta*180/pi)%360)

    timer_handle = simTime.create_timer(4, drive_forward)


def checkForGoal():
    global timer_handle, position, goal, travel
    simTime.destroy_timer(timer_handle)

    travel = [(goal[0] - position[0]), (goal[1] - position[1]), atan2((goal[1] - position[1]), (goal[0] - position[0]))*(180/pi)]

    if position == goal:
        while 1:
            print("Inside while loop in checkForGoal()")
            msg = Float32()
            msg.data = 20.0
            pubright.publish(msg)
            msg.data = -20.0
            publeft.publish(msg)

    bump = 0

    timer_handle = simTime.create_timer(4, motionFunc)


def findObst(message):
    global timer_handle, bump, position, goal, travel
    simTime.destroy_timer(timer_handle)

    hits = message.data

    for i in range(len(hits)):
        hit = unpack('b', hits[i])[0]

        if hit != 0:
            print("Touched on: ", i)
    print("----------------")
    bump = 1

def follow_boundary():
    global timer_handle, position, goal, travel
    updatePos()
    i = 0
    while bump == 1:
        print("Inside while loop in follow_boundary()")
        distance[i] = sqrt((goal[0]-postion[0])^2 + (goal[1]-postion[1])^2)
        i +=1
        timer_handle = simTime.create_timer(1, drive_forward)


def drive_forward():
    global timer_handle, position, goal, travel
    simTime.destroy_timer(timer_handle)

    msg = Float32()
    msg.data = 5.0
    pubright.publish(msg)
    publeft.publish(msg)

    timer_handle = simTime.create_timer(5, motionFunc)


def turn_to_goal():
    global timer_handle, position, goal, travel
    simTime.destroy_timer(timer_handle)

    # simTime.destroy_timer(timer_handle)
    msg = Float32()

    while position[2] != travel[2]:
        msg.data = 1.0
        pubright.publish(msg)
        msg.data = -1.0
        publeft.publish(msg)

    timer_handle = simTime.create_timer(5, motionFunc)

def turn_right():
    global timer_handle, position, goal, travel
    simTime.destroy_timer(timer_handle)
    msg = Float32()

    degGoal = position[2] + 90
    while position[2] != degGoal:
        print("Inside while loop in turn_right()")
        msg.data = 1.0
        publeft.publish(msg)
        msg.data = -1.0
        pubright.publish(msg)


    timer_handle = simTime.create_timer(5, motionFunc)

def motionFunc():
    global timer_handle, position, goal, travel
    simTime.destroy_timer(timer_handle)

    print("starting Basic Motion Function")

    timer_handle = simTime.create_timer(1, turn_to_goal)
    while position != goal or bump != 1:
        print("Inside while loop #1 in motionFunc()")
        timer_handle = simTime.create_timer(1, drive_forward)

    if(postion == goal):
        return

    baseposition = postion
    timer_handle = simTime.create_timer(1, turn_right)
    timer_handle = simTime.create_timer(1, drive_forward)

    while baseposition != postion:
        print("Inside while loop #2 in motionFunc()")
        timer_handle = simTime.create_timer(1, follow_boundary)
        count += 1

        timer_handle = simTime.create_timer(1, turn_to_goal)

timer_handle = simTime.create_timer(5, turn_right)
#timer_handle = simTime.create_timer(5, drive_forward)


gps = node.create_subscription(Pose2D, 'robot1/gps', updatePos)
subtouches = node.create_subscription(ByteMultiArray, 'robot1/touches' , findObst)

rclpy.spin(node)
node.destroy_node()
rclpy.shutdown()
