import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from std_msgs.msg import ByteMultiArray
from veranda.SimTimer import SimTimer
from geometry_msgs.msg import Pose2D
from sensor_msgs.msg import LaserScan
from struct import *
from math import *


def updatePos(message):
	global position, travel, goal, bump

	position[0] = message.x
	position[1] = message.y
	position[2] = ((message.theta*180/pi)%360)

	# print("Robot: (" + str(message.x) + "," + str(message.y) + ") theta: " + str((message.theta*180/pi) % 360))
	motionFunc()


def getTargetPos(message):
	global position, travel, goal, bump, flag
	goal[0] = message.x
	goal[1] = message.y
	goal[2] = ((message.theta*180/pi)%360)

	# print("Target: (" + str(message.x) + "," + str(message.y) + ") theta: " + str((message.theta*180/pi) % 360))
	travelCalc()

def get_lidar(message):
	global mid, left, right, mid_left, mid_right
	left = message.ranges[40]
	mid_left = message.ranges[32]
	mid = message.ranges[24]
	mid_right = message.ranges[16]
	right = message.ranges[8]

	print ("Left: " + str(left))
	print ("Right: " + str(right))
	print ("Mid: " + str(mid))

	motionFunc()

def turn_to_goal():
	global position, travel, goal, bump
	print("\nAt turn_to_goal bump is: ", bump)

	if position[2] < (travel[2] - 10):
		msg = Float32()
		msg.data = 0.5
		pubright.publish(msg)
		msg.data = -0.5
		publeft.publish(msg)
	elif position[2] > (travel[2] + 10):
		msg = Float32()
		msg.data = -0.5
		pubright.publish(msg)
		msg.data = 0.5
		publeft.publish(msg)
	else:
		msg = Float32()
		msg.data = 0.0
		pubright.publish(msg)
		publeft.publish(msg)


def turnAway():
	global left, right, mid, mid_left, mid_right, position, travel, goal
	msg = Float32()

	if left < 2:
		msg.data = 3.0
		publeft.publish(msg)
		msg.data = 1.0
		pubright.publish(msg)

	elif right < 2:
		msg.data = 1.0
		publeft.publish(msg)
		msg.data = 3.0
		pubright.publish(msg)

	elif mid < 2:
		msg.data = -2.0
		publeft.publish(msg)
		msg.data = 2.0
		pubright.publish(msg)

def drive():
	global position, travel, goal, bump

	if position[2] > (travel[2] - 10) and position[2] < (travel[2] + 10):
		msg = Float32()
		msg.data = 4.0
		pubright.publish(msg)
		msg.data = 4.0
		publeft.publish(msg)
		# print("On correct path\n")

	else:
		turn_to_goal()


def travelCalc():
	global position, travel, goal, bump

	travel = [(goal[0] - position[0]), (goal[1] - position[1]), atan2((goal[1] - position[1]), (goal[0] - position[0]))*(180/pi)]
	if travel[2] < 0:
		travel[2] += 360

	print("Position angle: ", position[2])
	print("Travel angle: ", travel[2])
	# print("Bump is: ", bump)


def motionFunc():
	global position, travel, goal, bump, mid, left, right
	# print("Inside Motion Function\n")

	if mid > 2 and left > 2 and right > 2:
		drive()

	else:
		turnAway()


rclpy.init()
node = Node("BM")
publeft = node.create_publisher(Float32, 'robot1/left_wheel')
pubright = node.create_publisher(Float32, 'robot1/right_wheel')
gps = node.create_subscription(Pose2D, 'robot1/gps', updatePos)
subtaget = node.create_subscription(Pose2D, 'target/gps', getTargetPos)
lidar = node.create_subscription(LaserScan, 'robot1/lidar', get_lidar)


goal = [100, 0, 0]
flag = 0
travel = [0, 0, 0]
position = [0, 0, 0]
bump = 0
count = 0
left, right, mid = 0, 0, 0


rclpy.spin(node)
node.destroy_node()
rclpy.shutdown()
