import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from std_msgs.msg import ByteMultiArray
from veranda.SimTimer import SimTimer
from geometry_msgs.msg import Pose2D
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
	global position, travel, goal, bump
	goal[0] = message.x
	goal[1] = message.y
	goal[2] = ((message.theta*180/pi)%360)

	# print("Target: (" + str(message.x) + "," + str(message.y) + ") theta: " + str((message.theta*180/pi) % 360))
	travelCalc()


# def findObst(message):
# 	global position, travel, goal, bump

# 	hits = message.data

# 	for i in range(len(hits)):
# 		hit = unpack('b', hits[i])[0]

# 		if hit != 0:
# 			bump = 1
# 			msg = Float32()
# 			msg.data = -5.0
# 			pubright.publish(msg)
# 			publeft.publish(msg)

# 			turn_left()


def turn_to_goal():
	global position, travel, goal, bump
	print("\nAt turn_to_goal bump is: ", bump)
	if position[2] < (travel[2] - 5):
		msg = Float32()
		msg.data = 0.5
		pubright.publish(msg)
		msg.data = -0.5
		publeft.publish(msg)
	elif position[2] > (travel[2] + 5):
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



def turn_left():

	print("\nAt turn_left() bump is: ", bump)

	msg = Float32()

	for i in range(30):
		msg.data = -5.0
		pubright.publish(msg)
		publeft.publish(msg)

	for i in range(30):
		msg.data = 0.0
		pubright.publish(msg)
		publeft.publish(msg)

	travel[2] += 90
	turn_to_goal()


def drive():
	global position, travel, goal, bump

	# print("at drive bump is: ", bump)
	# if bump == 1:
	#     msg = Float32()
	#     msg.data = 3.0
	#     pubright.publish(msg)
	#     msg.data = 3.0
	#     publeft.publish(msg)
	#     bump = 0
	#     motionFunc()

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
	global position, travel, goal, bump
	if bump == 1:
		turn_left()
	else:
		drive()


rclpy.init()
node = Node("BM")
publeft = node.create_publisher(Float32, 'robot1/left_wheel')
pubright = node.create_publisher(Float32, 'robot1/right_wheel')
gps = node.create_subscription(Pose2D, 'robot1/gps', updatePos)
subtouches = node.create_subscription(ByteMultiArray, 'robot1/touches' , findObst)
subtaget = node.create_subscription(Pose2D, 'target/gps', getTargetPos)

goal = [100, 0, 0]
travel = [0, 0, 0]
position = [0, 0, 0]
bump = 0

rclpy.spin(node)
node.destroy_node()
rclpy.shutdown()
