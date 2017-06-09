#!/usr/bin/env python
from __future__ import print_function

import subprocess as prc
import rospy
import time

from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Quaternion
from ihmc_msgs.msg import OneDoFJointTrajectoryRosMessage
from ihmc_msgs.msg import TrajectoryPoint1DRosMessage
from ihmc_msgs.msg import SO3TrajectoryPointRosMessage

t1=['/ihmc_ros/localization/pelvis_odom_pose_correction', 
'/ihmc_ros/localization/pelvis_pose_correction', 
'/ihmc_ros/valkyrie/control/abort_walking', 
'/ihmc_ros/valkyrie/control/arm_desired_joint_accelerations', 
'/ihmc_ros/valkyrie/control/arm_trajectory', 
#5
'/ihmc_ros/valkyrie/control/chest_trajectory', 
'/ihmc_ros/valkyrie/control/end_effector_load_bearing', 
'/ihmc_ros/valkyrie/control/foot_trajectory', 
'/ihmc_ros/valkyrie/control/footstep_list', 
'/ihmc_ros/valkyrie/control/go_home',
#10 
'/ihmc_ros/valkyrie/control/hand_desired_configuration', 
'/ihmc_ros/valkyrie/control/hand_trajectory', 
'/ihmc_ros/valkyrie/control/head_trajectory', 
'/ihmc_ros/valkyrie/control/high_level_state', 
'/ihmc_ros/valkyrie/control/low_level_control_mode', 
#15
'/ihmc_ros/valkyrie/control/neck_desired_acceleration', 
'/ihmc_ros/valkyrie/control/neck_trajectory', 
'/ihmc_ros/valkyrie/control/pause_walking', 
'/ihmc_ros/valkyrie/control/pelvis_height_trajectory', 
'/ihmc_ros/valkyrie/control/pelvis_orientation_trajectory',
#20
'/ihmc_ros/valkyrie/control/pelvis_trajectory', 
'/ihmc_ros/valkyrie/control/request_stop', 
'/ihmc_ros/valkyrie/control/stop_all_trajectories', 
'/ihmc_ros/valkyrie/control/whole_body_trajectory', 
'/ihmc_ros/valkyrie/output/behavior',
#25 
'/ihmc_ros/valkyrie/output/capturability/capture_point', 
'/ihmc_ros/valkyrie/output/capturability/center_of_mass', 
'/ihmc_ros/valkyrie/output/capturability/desired_capture_point', 
'/ihmc_ros/valkyrie/output/capturability/is_in_double_support', 
'/ihmc_ros/valkyrie/output/capturability/left_foot_support_polygon',
#30 
'/ihmc_ros/valkyrie/output/capturability/right_foot_support_polygon', 
'/ihmc_ros/valkyrie/output/foot_force_sensor/left', 
'/ihmc_ros/valkyrie/output/foot_force_sensor/right', 
'/ihmc_ros/valkyrie/output/footstep_status', 
'/ihmc_ros/valkyrie/output/high_level_state',
#35 
'/ihmc_ros/valkyrie/output/high_level_state_change', 
'/ihmc_ros/valkyrie/output/imu/pelvis_pelvisMiddleImu', 
'/ihmc_ros/valkyrie/output/imu/pelvis_pelvisRearImu', 
'/ihmc_ros/valkyrie/output/imu/torso_leftTorsoImu', 
'/ihmc_ros/valkyrie/output/imu/upperNeckPitchLink_head_imu_sensor', 
#40
'/ihmc_ros/valkyrie/output/joint_states', 
'/ihmc_ros/valkyrie/output/robot_motion_status', 
'/ihmc_ros/valkyrie/output/robot_pose', 
'/ihmc_ros/valkyrie/output/walking_status', 
'/imu_states', 
#45
'/leftTorsoImu/imu' ]

class Listener:
    def __init__(self,i,m):
        print("Listener(%d,%s) " % (i,m))
        self.index = i
        self.msgtyp = m
        self.msg = None
        self.lasttim = 0.0
        self.rate = 0.0
  
    def cb(self,msg):
	self.msg = msg
        time = rospy.Time.now().to_sec()
        dur = time - self.lasttim
        self.rate = 0.9 * self.rate + 0.1 * dur
        self.lasttim = time


topics = []
listeners = []
subscribers = []

try:
    rospy.init_node('subscribeall')
    time.sleep(1)
    rate = rospy.Rate(10) # 10hz
    time.sleep(1)
except Exception as e:
    print(e)

for i,t in enumerate(t1):
    cmd = 'rostopic info %s | grep Type:' % t
    ef = prc.check_output(cmd, shell=True)
    t2 = ef.strip().split(' ')[1]
    topics.append( (t,t2) )
    msg = t2.split('/')
    imp = 'from %s.msg import %s' % (msg[0],msg[1])
    try:
        exec(imp,globals())
        l = Listener(i,msg[1])
        s = rospy.Subscriber( t, globals()[msg[1]], l.cb )
        listeners.append( l )
        subscribers.append( s )
    except Exception as e:
        print(e)

try:
    #print(globals())
    while not rospy.is_shutdown():
      print([listeners[i].rate for i in [42,43]])
      rate.sleep()

except rospy.ROSInterruptException:
        pass

