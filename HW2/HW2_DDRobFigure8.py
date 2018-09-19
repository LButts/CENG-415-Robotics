import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from std_msgs.msg import ByteMultiArray
from veranda.SimTimer import SimTimer
from struct import *
from math import *

rclpy.init()
node = Node("Triangle")

simTime= SimTimer(True, 'veranda/timestamp', node)

publeft = node.create_publisher(Float32, 'robot1/left_wheel')
pubright = node.create_publisher(Float32, 'robot1/right_wheel')

r = 0.5 # radius of wheel (m)
l = 0.6 # half the distance between wheels (m)
count = 0
pivotTime = 0.0
moveTime = 0.0
dTheta = 0.0

xPoints = [0, 15]
yPoints = [0, 0]

def drive_forward():
    global timer_handle, r, count, xPoints, yPoints, moveTime, dTheta
    simTime.destroy_timer(timer_handle)
    msg = Float32()

    xStart = xPoints[count]
    xEnd = xPoints[count+1]

    yStart = yPoints[count]
    yEnd = yPoints[count+1]

    d = sqrt((xEnd - xStart)*(xEnd - xStart) + (yEnd - yStart)*(yEnd - yStart))

    moveTime = d
    rps = 1/r

    msg.data = rps
    pubright.publish(msg)
    publeft.publish(msg)

    timer_handle = simTime.create_timer(moveTime, turn_left)

def turn_left():
    global timer_handle, r, count, xPoints, yPoints, moveTime, dTheta
    simTime.destroy_timer(timer_handle)
    msg = Float32()

    dTheta = pi/2

    rps = 0.5

    pivotTime = dTheta * l / (r*rps)

    msg.data = rps
    pubright.publish(msg)
    msg.data = -1 * rps
    publeft.publish(msg)

    timer_handle = simTime.create_timer(pivotTime, drive_circle)

def drive_circle_bottom():
    global timer_handle, r, count, xPoints, yPoints, moveTime, dTheta
    simTime.destroy_timer(timer_handle)
    msg = Float32()

    circ = pi*14
    moveTime = (2*circ)/7
    if(count < 1):
        moveTime = (2*circ)/(3*7)

    msg.data = 8.2
    pubright.publish(msg)
    msg.data = 7.0
    publeft.publish(msg)

    count = count + 1

    timer_handle = simTime.create_timer(moveTime, drive_circle_top)


def drive_circle_top():
    global timer_handle, r, count, xPoints, yPoints, moveTime, dTheta
    simTime.destroy_timer(timer_handle)
    msg = Float32()

    circ = pi*16
    moveTime = (2*circ)/8

    msg.data = 9.2
    publeft.publish(msg)
    msg.data = 8.0
    pubright.publish(msg)

    timer_handle = simTime.create_timer(moveTime, drive_circle_bottom)

timer_handle = simTime.create_timer(0.1, drive_circle_bottom)

rclpy.spin(node)
node.destroy_node()
rclpy.shutdown()
