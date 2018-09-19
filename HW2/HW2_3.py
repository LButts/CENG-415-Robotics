import rclpy
from rclpy.node import Node
from math import *
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int32

p1 = [None]*100
p2 = [None]*100

rclpy.init()

control = rclpy.create_node('Talker')
wheelPub = control.create_publisher(Float32MultiArray, 'WheelVel')
actPub = control.create_publisher(Int32, 'Active')

for i in range(100):
    p1[i] = 2 + 2*pow(e, (-i/10))
    p2[i] = 2 + pow(e, (-i/10))


#print(p1, '\n\n\n', p2)

msg1 = Float32MultiArray(data = p1)
msg2 = Float32MultiArray(data = p2)
msg3 = Int32(data = 1)

wheelPub.publish(msg1)
wheelPub.publish(msg2)
actPub.publish(msg3)

rclpy.spin(control)

control.destroy_node()
rclpy.shutdown()
