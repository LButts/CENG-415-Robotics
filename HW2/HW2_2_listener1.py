import rclpy
from std_msgs.msg import Float32MultiArray
import math

rclpy.init(args=None)
node = rclpy.create_node('min_sub')
thetaNode = rclpy.create_node('minimal_publisher')
thetaPublisher = thetaNode.create_publisher(Float32MultiArray, 'thetaData')
th1 = [None]*100
th2 = [None]*100

def callback(msg):
  global var
  var = msg.data


def twolinkik(a1, a2, xend, yend):
 d = (xend*xend + yend*yend - a1*a1 - a2*a2)/(2*a1*a2)
 theta2 = math.atan2(-math.sqrt(1.0 - d*d),d)
 theta1 = math.atan2(yend,xend) - math.atan2(a2*math.sin(theta2), a1+a2*math.cos(theta2))
 return theta1*180/math.pi, theta2*180/math.pi

subscription = node.create_subscription(Float32MultiArray, 'physData', callback)
while rclpy.ok():
  rclpy.spin_once(node)
  x = var
  rclpy.spin_once(node)
  y = var

  for i in range(100):
      th1[i], th2[i] = twolinkik(10, 10, x[i], y[i])

  msg = Float32MultiArray(data = th1)
  thetaPublisher.publish(msg)
  msg = Float32MultiArray(data = th2)
  thetaPublisher.publish(msg)
