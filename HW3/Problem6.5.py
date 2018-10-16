import pylab as plt
import numpy as np
from math import *
N=52
x = np.zeros(N)
y = np.zeros(N)
q = np.zeros(N)
x[0] = 0; y[0] = 0; q[0]  = 0.0
t = 0;  dt = 0.1

def ddstep(xc, yc, qc,r,l,dt,w1,w2):
   xn = xc + (r*dt/2.0)*(w1+w2)*cos(qc)
   yn = yc + (r*dt/2.0)*(w1+w2)*sin(qc)
   qn = qc + (r*dt/(2.0*l))*(w1-w2)
   return (xn,yn,qn)


for i in range(N-1):
   w1 = 0.25*t*t
   w2 = 0.5*t
   x[i+1], y[i+1], q[i+1] = ddstep(x[i], y[i], q[i],20,12.0,dt,w1,w2)
   t = t + dt
   print(t)

plt.plot(x,y,'b')
plt.show()
