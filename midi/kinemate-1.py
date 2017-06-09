#!/usr/bin/env python
#
# joystick input driver for Korg NanoKontrol input device
#
# Author: Austin Hendrix
# Author: Allison Thackston


import rospy
import tf
import tf2_ros
import time
import csv

from axis import Axis

import itertools
from tf.transformations import quaternion_from_euler

from sensor_msgs.msg import JointState
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Quaternion

class Datum():
    def __init__(self,k,v):
        self.key = k
        self.val = v
    def __repr__(self):
        return "!%s %s = %d" % (self.__class__.__name__,
		self.key,self.val)
    def get_rosval(self):
        return self.val
    def get_sibling(self,idx):
        return Datum(idx,0.5)

class ArmPos():
    def __init__(self):
        self.js = None
        if not rospy.has_param('/ihmc_ros/robot_name'):
            rospy.logerr("Missing parameter '/ihmc_ros/robot_name'")
        else:
            ROBOT_NAME = rospy.get_param('/ihmc_ros/robot_name')
            jss_name = "/ihmc_ros/{0}/output/joint_states".format(ROBOT_NAME)
            self.jsSubscriber = rospy.Subscriber( jss_name,
                JointState, self.recievedJointStates )
        self.names = None
        self.idx = None

    def __repr__(self):
        return "!%s ('%s')" % (self.__class__.__name__, "','".join(self.names))

    def subscribe(self,names):
        self.names = names
        self.idx = [ self.js.name.index(n) for n in names ]
        if len(self.names) != len(self.idx):
            rospy.logerr("ArmPos: Only %d of %d names found." % (len(self.idx),len(self.names))) 

    def val(self):
        if self.idx == None:
            rospy.logerr("ArmPos: subscribe first")
            return []
        return [ self.js.position[i] for i in self.idx ]

    def recievedJointStates(self,msg):
        self.js = msg #header,name,position,velocity,effort

class Kinemate(object):

    ''' Class to connect to and publish the Korg NanoKontrol midi device
        as a ros joystick
    '''
    #ROS joint names 
    leftandright = ['leftShoulderPitch', 'leftShoulderRoll', 'leftShoulderYaw',
			 'leftElbowPitch', 'rightShoulderPitch', 'rightShoulderRoll',
                         'rightShoulderYaw', 'rightElbowPitch' ]
    #Axis joint names
    lalr = ['la0','la1','la2','la3','ra0','ra1','ra2','ra3']

    def __init__(self):

        self.axis = Axis(verbose=0)
        self.axis.init()
        self.poser = ArmPos()
        self.csvf = open('kin.csv', 'wb')
        self.csv  = csv.writer(self.csvf)

        #complicated reset just for fun with Python comprehensions
        self.angles = [0.2,  1.4, 0.2,  1.5]
        self.rtinvert = [ 1.0, 1.0, 1.0, 1.0]
        self.lfinvert = [ 1.0,-1.0, 1.0,-1.0]
        self.lch = ['la0', 'la1', 'la2', 'la3']
        self.rch = ['ra0', 'ra1', 'ra2', 'ra3']
        data = [ Datum(j[0],j[1]*j[2]) for j in zip(self.rch,self.angles,self.rtinvert)]
	self.axis.process_keys(data)
        rospy.sleep(0.5)
        data = [ Datum(j[0],j[1]*j[2]) for j in zip(self.lch,self.angles,self.lfinvert)]
	self.axis.process_keys(data)

        self.pose = [0.2,  1.4, 0.2,  1.5, 0.2,-1.4, 0.2,-1.5]
        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer)
        self.poser.subscribe(Kinemate.leftandright)

    def finish(self):
        self.axis.fini()
        self.csvf.close()

    def setPose(self,joint,val):
        self.pose[Kinemate.lalr.index(joint)] = val
        self.axis.process_keys([Datum(joint,val)])
        rospy.sleep(0.05)
        pass

    def delta(self,Euc):
        lpalmtf = self.tfBuffer.lookup_transform('torso', 'leftPalm', rospy.Time())
        lp = lpalmtf.transform.translation
        rpalmtf = self.tfBuffer.lookup_transform('torso', 'rightPalm', rospy.Time())
        rp = rpalmtf.transform.translation
        pos = [lp.x,lp.y,lp.z,rp.x,rp.y,rp.z]
        return [ pos[i]-Euc[i] for i in range(len(Euc)) ]

    def derivative(self):
        palm = self.delta([0,0,0,0,0,0])
        pose = self.pose
        for j in range(4):
            ljoint = Kinemate.lalr[j]
            lcurval = pose[j]
            self.axis.process_keys([Datum(ljoint,lcurval+0.1)])
            rospy.sleep(0.05)
            rjoint = Kinemate.lalr[j+4]
            rcurval = pose[j+4]
            self.axis.process_keys([Datum(rjoint,rcurval+0.1)])

            print "Derivative %d:" % j
            self.savePose(start=palm)
            self.axis.process_keys([Datum(ljoint,lcurval)])
            rospy.sleep(0.05)
            self.axis.process_keys([Datum(rjoint,rcurval)])

    def savePose(self,start=[0,0,0,0,0,0]):
        rospy.sleep(1.0)
        palm = self.delta(start)
        requested = [ "%.3f" % v for v in self.pose]
        actual = [ "%.3f" % v for v in self.poser.val()]
        xyz = [ "%.3f" % v for v in palm]
        print "Req: %s" % requested
        print "Act: %s" % actual 
        #print "(%.3f,%.3f,%.3f)" % (palm.x,palm.y,palm.z)
        print "Result: %s" % xyz
        self.writePose(rospy.Time.now().to_sec(),self.pose,self.poser.val(),palm)

    def writePose(self,ts,req,act,xyz):
        row = [ts]
        row.extend(req)
        row.extend(act)
        row.extend(xyz)
        ans = [ round(v,4) for v in row]
        self.csv.writerow(ans)
        print ans

    def run(self):

        rate = rospy.Rate(10) # 10hz
        time.sleep(1)

        rtShoulderPitch = [-0.2,-0.7]
        rtShoulderRoll  = [ 1.4, 0.9]
        rtShoulderYaw   = [ 0.2, 0.0]
        rtElbowPitch    = [ 1.5, 1.0]

        try:
            while not rospy.is_shutdown():
              for j0 in rtShoulderPitch:
                self.setPose('ra0',j0)
                self.setPose('la0',j0)
                for j1 in rtShoulderRoll:
                  self.setPose('ra1',j1)
                  self.setPose('la1',-j1)
                  for j2 in rtShoulderYaw:
                    self.setPose('ra2',j2)
                    self.setPose('la2',-0.5-j2)
                    for j3 in rtElbowPitch:
                      self.setPose('ra3',j3)
                      self.setPose('la3',-j3)
                      print "   "
                      print "===Test Position==="
                      self.savePose()
                      self.derivative()

              rospy.signal_shutdown("Kinemate Done")

        finally:
            rospy.loginfo("Kinemate Died")
            self.finish()

if __name__ == '__main__':
    rospy.init_node('Kinemate')
    teleop = Kinemate()
    teleop.run()
