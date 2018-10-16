import pylab as plt
import numpy as np
from math import *
N=52
mu, sigma = 0.0, 0.3

filename = "output.txt"
file = open(filename, "w")

for k in range(100):
    x = np.zeros(N)
    y = np.zeros(N)
    x_with_error = np.zeros(N)
    y_with_error = np.zeros(N)
    q = np.zeros(N)
    xerr = np.random.normal(mu,sigma, 100)
    yerr = np.random.normal(mu,sigma, 100)
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

        x_with_error[i] = xerr[i] + x[i]
        y_with_error[i] = yerr[i] + y[i]
        x[i+1], y[i+1], q[i+1] = ddstep(x[i], y[i], q[i],20,12.0,dt,w1,w2)
        x_with_error[i+1], y_with_error[i+1], q[i+1] = ddstep(x_with_error[i], y_with_error[i], q[i],20,12.0,dt,w1,w2)
        t = t + dt
        print(t)
    file.write('{0} {1}\n'.format(x[51], y[51]))
    k+=1

file.close()

plt.plot(x_with_error,y_with_error,'b')
plt.show()
