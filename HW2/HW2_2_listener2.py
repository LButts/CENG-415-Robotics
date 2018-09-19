import rclpy
from std_msgs.msg import Float32MultiArray
import math
import matplotlib.pyplot as plt

rclpy.init(args=None)

thetaNode = rclpy.create_node('min_sub')
physNode = rclpy.create_node('min_sub')

x = [None]*100
y = [None]*100
theta1 = [None]*100
theta2 = [None]*100
count = 1
xcomp = [None]*100
ycomp = [None]*100

def callbackP(msg):
    global varP
    varP = msg.data
    print("reading physData message")
    print('\n')
    #print(varP)

def callbackT(msg):
    global varT
    varT = msg.data
    print("reading thetaData message")
    print('\n')
    #print(varT)

def twolinkfk(a1, a2, theta1, theta2):
 x = a2*math.cos(theta1+theta2)+a1*math.cos(theta1)
 y = a2*math.sin(theta1+theta2)+a1*math.sin(theta1)
 return x, y


sub1 = physNode.create_subscription(Float32MultiArray, 'physData', callbackP)
sub2 = thetaNode.create_subscription(Float32MultiArray, 'thetaData', callbackT)

while rclpy.ok():

    rclpy.spin_once(physNode)
    if(count % 2 != 0):
        x = varP
        #print(x)
    else:
        y = varP
        #print(y)

    rclpy.spin_once(thetaNode)
    if(count % 2 != 0):
        theta1 = varT
        #print(theta1)
    else:
        theta2 = varT
        #print(theta2)

    if(count >= 2):
        for i in range(100):
            xcomp[i], ycomp[i] = twolinkfk(10, 10, (theta1[i]*math.pi/180), (theta2[i]*math.pi/180))
        break

    count = count + 1

plt.axis([-1, 11, 4, 16])
plt.plot(x, y, 'g')
plt.plot(xcomp, ycomp, 'b^')
plt.show()

physNode.destroy_node()
thetaNode.destroy_node()
rclpy.shutdown()
