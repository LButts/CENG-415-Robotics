import rclpy
from rclpy.node import Node
from math import *
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int32

rclpy.init()

p1 = [None]*100
p2 = [None]*100
linSpeed = [None]*100
angSpeed = [None]*100
d = 10
l = 20


subNode = rclpy.create_node('min_sub')
ForwardK = rclpy.create_node('minimal_publisher')
FKPub = ForwardK.create_publisher(Float32MultiArray, 'RobotVel')

def callback(msg):
  global var
  var = msg.data
  print("Reading Wheel Velocities\n")

def linearV(d, p1, p2):
    r = d/2
    linear = (r/2)*(p1+p2)
    return linear

def angularV(d, l, p1, p2):
    r = d/2
    angular = (r/(2*l))*(p1-p2)
    return angular

wheelSubscription = subNode.create_subscription(Float32MultiArray, 'WheelVel', callback)

while rclpy.ok():
    rclpy.spin_once(subNode)
    p1 = var
    #print(p1, '\n\n')
    rclpy.spin_once(subNode)
    p2 = var
    #print(p2, '\n\n')

    for i in range(100):
        linSpeed[i] = linearV(d, p1[i], p2[i])
        angSpeed[i] = angularV(d, l, p1[i], p2[i])

    print(linSpeed, '\n\n')
    print(angSpeed, '\n\n')
