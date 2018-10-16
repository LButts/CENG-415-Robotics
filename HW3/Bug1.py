import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from std_msgs.msg import ByteMultiArray
from veranda.SimTimer import SimTimer
from geometry_msgs.msg import Pose2D
from struct import *
from math import *
import time

rclpy.init()
node = Node("BM")

simTime= SimTimer(True, 'veranda/timestamp', node)

publeft = node.create_publisher(Float32, 'robot1/left_wheel')
pubright = node.create_publisher(Float32, 'robot1/right_wheel')

def updatePos(message):
    global position, travel, goal, bump

    position[0] = message.x
    position[1] = message.y
    position[2] = ((message.theta*180/pi)%360)
    travel[0] = goal[0] - position[0]
    travel[1] = goal[1] - position[1]
    travel[2] = atan2((goal[1] - position[1]), (goal[0] - position[0]))*(180/pi)



def checkForGoal():
    global position, travel, goal, bump

    if position == goal:
        while 1:
            print("Inside while loop in checkForGoal()")
            msg = Float32()
            msg.data = 20.0
            pubright.publish(msg)
            msg.data = -20.0
            publeft.publish(msg)

    bump = 0



def findObst(message):
    global position, travel, goal, bump

    hits = message.data

    for i in range(len(hits)):
        hit = unpack('b', hits[i])[0]

        if hit != 0:
            print("Touched on: ", i)
            bump = 1

    print("----------------")



# def follow_boundary():
#     i = 0
#     while bump == 1:
#         print("Inside while loop in follow_boundary()")
#         distance[i] = sqrt((goal[0]-postion[0])^2 + (goal[1]-postion[1])^2)
#         i +=1


def drive_forward():
    global position, travel, goal, bump

    msg = Float32()
    msg.data = 5.0
    pubright.publish(msg)
    publeft.publish(msg)


    checkForGoal()


def turn_to_goal():
    global position, travel, goal, bump

    msg = Float32()

    if position[2] != travel[2]:
        msg.data = 1.0
        pubright.publish(msg)
        msg.data = -1.0
        publeft.publish(msg)


    drive_forward()


def turn_right():
    global position, travel, goal, bump
    msg = Float32()

    degGoal = position[2] + 90
    if position[2] != degGoal:
        print("Inside while loop in turn_right()")
        msg.data = 1.0
        publeft.publish(msg)
        msg.data = -1.0
        pubright.publish(msg)


    drive_forward()






gps = node.create_subscription(Pose2D, 'robot1/gps', updatePos)
subtouches = node.create_subscription(ByteMultiArray, 'robot1/touches' , findObst)

global position, travel, goal, bump

goal = [100, 0, 0]
travel = [0, 0, 0]
position = [0, 0, 0]
bump = 0

# msg = Float32()
# msg.data = 5.0
# pubright.publish(msg)
# publeft.publish(msg)

while rclpy.ok():

    rclpy.spin_once(node)

    # print("starting Basic Motion Function")

    if position != goal or bump != 1:
        # print("Inside if #1 in motionFunc()")
        drive_forward()

    # if(postion == goal):
    #     print("Inside if #2 in motionFunc()")
    #     return

    if bump == 1:
        # print("Inside if #2 in motionFunc()")
        turn_right()

    turn_to_goal()

node.destroy_node()
rclpy.shutdown()
