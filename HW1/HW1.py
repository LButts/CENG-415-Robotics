import math
import numpy as np
import sympy as sy
import matplotlib.pyplot as plt

def twolinkfk(a1, a2, theta1, theta2):
 x = a2*np.cos(theta1+theta2)+a1*np.cos(theta1)
 y = a2*np.sin(theta1+theta2)+a1*np.sin(theta1)
 return x, y

def twolinkik(a1, a2, xend, yend):
 d = (xend*xend + yend*yend - a1*a1 - a2*a2)/(2*a1*a2)
 theta2 = math.atan2(-math.sqrt(1.0 - d*d),d)
 theta1 = math.atan2(yend,xend) - math.atan2(a2*math.sin(theta2), a1+a2*math.cos(theta2))
 return theta1*180/math.pi, theta2*180/math.pi

def twolinkvk(a1, a2, theta1, theta2, dtheta1, dtheta2):
 x1 = a2*math.cos(theta1+theta2)+a1*math.cos(theta1)
 y1 = a2*math.sin(theta1+theta2)+a1*math.sin(theta1)
 x2 = a2*math.cos((theta1+dtheta1)+(theta2+dtheta2))+a1*math.cos(theta1+dtheta1)
 y2 = a2*math.sin((theta1+dtheta1)+(theta2+dtheta2))+a1*math.sin(theta1+dtheta1)
 dx = x2 - x1
 dy = y2 - y1
 return dx, dy

def twolinkgraphf(a1, a2, theta1, theta2):
 x, y = twolinkfk(a1, a2, theta1, theta2)
 plt.xlim(3, 22)
 plt.ylim(-2, 17)
 plt.plot(x, y)
 plt.show()

def twolinkgraphth(a1, a2):

 xa = np.array([5, 5, 20, 20, 5])
 ya = np.array([0, 15, 15, 0, 0])
 #xa = np.arange(1, 25)
 #ya = np.arange(24, 0, -1)
 d = (xa*xa + ya*ya - a1*a1 - a2*a2)/(2*a1*a1)
 th2a = np.arctan2(-np.sqrt(1.0 - d*d),d)
 th1a = np.arctan2(ya,xa) - np.arctan2(a2*np.sin(th2a), a1+a2*np.cos(th2a))
 plt.xlim(-0, 2.5)
 plt.ylim(-2, -0.5)
 plt.plot(th1a,th2a)
 plt.show()
 t = np.linspace(0,3.14,25)
 xb = 10*np.cos(t) + 15
 yb = 10*np.sin(t)
 d = (xb*xb + yb*yb - a1*a1 - a2*a2)/(2*a1*a1)
 th2b = np.arctan2(-np.sqrt(1.0 - d*d),d)
 th1b = np.arctan2(yb,xb) - np.arctan2(a2*np.sin(th2b), a1+a2*np.cos(th2b))
 plt.xlim(0, 2.5)
 plt.ylim(-3, -1)
 plt.plot(th1b,th2b)
 plt.show()
 return (th1a, th2a)

print(twolinkfk(12, 7, (45*(math.pi/180)), (45*(math.pi/180))))
print()
print(twolinkvk(12, 7, (45*(math.pi/180)), (45*(math.pi/180)), (5*(math.pi/180)), (5*(math.pi/180))))
print()
print(twolinkik(12, 7, 12, 14))
print()
theta1, theta2 = twolinkgraphth(15, 15)
twolinkgraphf(15, 15, theta1, theta2)
