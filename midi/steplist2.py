#!/usr/bin/env python
from __future__ import print_function
import numpy as np

S_W = 0.2 #stance width
SW2 = S_W/2.0
D1  = np.pi/180
D15 = 15*D1

class Foot:
    LEFT = 1
    RIGHT = -1
    def __init__(self,xyt,side):
        self.x = xyt[0]
        self.y = xyt[1]
        self.t = xyt[2]
        self.side = side #Left +1, Right -1

class Center:
    def __init__(self,foot,bigR):
        self.foot = foot
        self.r = bigR
        l = bigR - foot.side*SW2
        self.x = foot.x - l*np.sin(foot.t)
        self.y = foot.y + l*np.cos(foot.t)
        
    def newFoot(self,th):
        side = -1*self.foot.side
        theta = self.foot.t + th
        l = self.r - side*SW2
        x = self.x + l*np.sin(theta)
        y = self.y - l*np.cos(theta)
        return Foot([x,y,theta],side)
        
class Steplist:
    def __init__(self,first,other):
        self.list=[other,first]
        self.center=[]
        self.complete = 1
        self.current = None
        
    def fstep(self,R,th):
	c = Center(self.list[-1],R)
        f = c.newFoot(th)
        self.center.append(c)
        self.list.append(f)
	return (f.x,f.y,f.t)  

    def step(self,R,th):
	c = Center(self.list[-1],R)
        f = c.newFoot(th)
        self.center.append(c)
        self.list.append(f)
	return self.midway()       

    def stepn(self,R,th,n=1):
        for i in range(n):
            self.step(R,th)
        self.step(R,0)

    def midway(self):
	x = (self.list[-1].x + self.list[-2].x)/2
	y = (self.list[-1].y + self.list[-2].y)/2
	t = (self.list[-1].t + self.list[-2].t)/2
	return (x,y,t)

