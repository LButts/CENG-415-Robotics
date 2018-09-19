import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from std_msgs.msg import ByteMultiArray
from veranda.SimTimer import SimTimer
from struct import *
from math import *

rclpy.init()
node = Node("Square")

simTime= SimTimer(True, 'veranda/timestamp', node)

publeft = node.create_publisher(Float32, 'robot1/left_wheel')
pubright = node.create_publisher(Float32, 'robot1/right_wheel')

r = 0.5 # radius of wheel (m)
l = 0.6 # half the distance between wheels (m)
count = 0
pivotTime = 0.0
moveTime = 0.0
dTheta = 0.0

xPoints = [0, 10, 10, 0, 0, 10, 10, 0, 0, 0, 0]
yPoints = [0, 0, 10, 10, 0, 0, 10, 10, 0, 0, 0]
theta = [0, pi/2, pi/2, pi/2, pi/2, pi/2, pi/2, pi/2, 0, 0, 0]

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

    count = count + 1
    if(count >= 8):
        return

    timer_handle = simTime.create_timer(moveTime, turn_left)

def turn_left():
    global timer_handle, r, count, xPoints, yPoints, moveTime, dTheta
    simTime.destroy_timer(timer_handle)
    msg = Float32()

    dTheta = theta[count]

    if dTheta < 0.0:
        dTheta += (2.0 * pi)

    # check for extra radians to prevent spinning
    if dTheta > 2.0*pi:
        dTheta -= 2.0*pi

    rps = 0.5

    pivotTime = dTheta * l / (r*rps)

    msg.data = rps
    pubright.publish(msg)
    msg.data = -1 * rps
    publeft.publish(msg)

    timer_handle = simTime.create_timer(pivotTime, drive_forward)

timer_handle = simTime.create_timer(0.1, drive_forward)

rclpy.spin(node)
node.destroy_node()
rclpy.shutdown()
