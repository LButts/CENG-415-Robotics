from math import *
import numpy as np
import sympy as sy
import matplotlib.pyplot as plt

f = 0.08
b = 30
u = 0
a = 0
aLow = 360
aHigh = 0
uLow = 100000
uHigh = 0
aLow2 = 0
aHigh2 = 0
uLow2 = 100000
uHigh2 = 0


for i in range(360):

	for j in range(20, 100):

		if i == 0:
			break

		u = ((b*f)/j) - f*(1/tan((i*pi/180)))

		if u < uLow:
			uLow = u

		if u > uHigh:
			uHigh = u

		if i < aLow:
			aLow = i

		if i > aHigh:
			aHigh = i



for i in range(360):

	for j in range(10, 30):

		if i == 0:
			break

		u = ((j/b) - 1)*f*(1/tan(i*pi/180))

		if u < uLow2:
			uLow2 = u

		if u > uHigh2:
			uHigh2 = u

		if i < aLow2:
			aLow2 = i

		if i > aHigh2:
			aHigh2 = i


if uLow < uLow2:
	uLow = uLow2

if uHigh > uHigh2:
	uHigh = uHigh2

if aLow < aLow2:
	aLow = aLow2

if aHigh > aHigh2:
	aHigh = aHigh2

print("aLow = ", aLow, " aHigh = ", aHigh)
print("uLow = ", uLow, " uHigh = ", uHigh)
print("aLow2 = ", aLow2, " aHigh2 = ", aHigh2)
print("uLow2 = ", uLow2, " uHigh2 = ", uHigh2)
