from __future__ import print_function
import numpy as np
import matplotlib.pyplot as plt
from six.moves import cPickle as pickle
from six.moves import range
from mpl_toolkits.axes_grid1.inset_locator import inset_axes
from matplotlib.lines import Line2D
import math

S_W = 0.2 #stance width
SW2 = S_W/2.0
D1  = math.pi/180
D15 = 15*D1

def wedge(x,y,th,siz=0.1,wid=0.3):
    u = [x-siz*math.cos(th+wid/2),y-siz*math.sin(th+wid/2)]
    c = [x,y]
    l = [x-siz*math.cos(th-wid/2),y-siz*math.sin(th-wid/2)]
    return [u,c,l]

class Foot:
    LEFT = 1
    RIGHT = -1
    def __init__(self,xyt,side):
        self.x = xyt[0]
        self.y = xyt[1]
        self.t = xyt[2]
        self.side = side #Left +1, Right -1
    
    def draw(self,ax):
        color = 'r' if self.side > 0 else 'b'
        points = wedge(self.x,self.y,self.t)
        ax.add_patch(plt.Polygon(points, fc=color))

class Center:
    def __init__(self,foot,bigR):
        self.foot = foot
        self.r = bigR
        l = bigR - foot.side*SW2
        self.x = foot.x - l*math.sin(foot.t)
        self.y = foot.y + l*math.cos(foot.t)
        
    def draw(self,ax):
        points = wedge(self.x,self.y,self.foot.t)
        ax.add_patch(plt.Polygon(points, fc='k'))
        line = Line2D([self.x,self.foot.x],[self.y,self.foot.y], lw=1, c='k', alpha=0.3)
        ax.add_line(line)
        
    def newFoot(self,th):
        side = -1*self.foot.side
        theta = self.foot.t + th
        l = self.r - side*SW2
        #print(l,l*math.sin(theta),l*math.cos(theta))
        x = self.x + l*math.sin(theta)
        y = self.y - l*math.cos(theta)
        return Foot([x,y,theta],side)
        
class Steplist:
    def __init__(self,x,y,th,first=Foot.LEFT):
        lx = x - SW2*math.sin(th)
        ly = y + SW2*math.cos(th)
        first = Foot([lx,ly,th],first)
        other = Center(first,1.0).newFoot(0)
        self.list=[other,first]
        self.center=[]
        
    def step(self,R,th):
        self.center.append(Center(self.list[-1],R))
        self.list.append(self.center[-1].newFoot(th))
        
    def stepn(self,R,th,n=1):
        for i in range(n):
            self.step(R,th)
        self.step(R,0)
        
    def draw(self,ax,lastn=0):
        n = len(self.list) if lastn == 0 else lastn
        [f.draw(ax) for f in self.list]
        [f.draw(ax) for f in self.center]

fig,(ax) = plt.subplots(1, 1, figsize=(8,8))
ax.axis([-3, 3, -3, 3])

s = Steplist(-0.5,0.5,4.7)
s.step(-2,-0.1)
s.step(2,0.1)
s.step(-2,-0.1)
s.step(2,0.1)
s.step(0.7,.157)
s.step(0.7,.157)
s.step(0.7,.157)
s.step(0.7,.157)
s.step(-2,-0.1)
s.step(2,0.1)
s.step(-2,-0.1)
s.step(2,0.1)
s.stepn(0.11,D15,6)
#s.stepn(-0.09,-2*D15,3)
s.stepn(0.01,2*D15,3)
s.draw(ax)
plt.show()





