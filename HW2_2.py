import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from math import *

rclpy.init()
physNode = rclpy.create_node('minimal_publisher')
publisher = physNode.create_publisher(Float32MultiArray, 'physData')

a1 = a2 = 10.0
x = np.linspace(0, 10, 100)
y = 15 - x

xarr = [None]*100
yarr = [None]*100

for i in range(100):
    xarr[i] = x[i]
    yarr[i] = y[i]

msg = Float32MultiArray(data = xarr)
publisher.publish(msg)
msg = Float32MultiArray(data = yarr)
publisher.publish(msg)

rclpy.spin(physNode)

node.destroy_node()
rclpy.shutdown()
