#!/usr/bin/env python
import roslib
roslib.load_manifest('tf')
   
import rospy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped, Twist 
import tf

import numpy as np
import threading, platform, copy, time, os, math

import sys; [sys.path.append( i ) for i in ['..', '../msg', './msg']]
from Node import *
import stdmsg


class Driver:
    log_flag = False
    
    cmd_annotation = ""
    last_cmd_time = time.time()

    cmd_lock = threading.RLock()
    cmd_v = 0; cmd_w = 0; cmd_manual = False

    odom_lock = threading.RLock()
    x = 0; y = 0; theta = 0; v = 0; w = 0
    
    laser_x = 0.7
    laser_y = 0
    laser_theta = 0
        
    def __init__(self, file = None):
        self.nh = Node( 'tcp://*:8065' )
        self.rpc = RPC()
        self.rpc.connect("tcp://127.0.0.1:9000")
        
        self.is_running = True
        
        
        self.seq = 1
        rospy.init_node('listener', anonymous=True)
        rospy.Subscriber("/scan", LaserScan, self._laser)
        
        self.cmdpub = rospy.Publisher('/cmd_vel', Twist)
        self.tflistener = tf.TransformListener()

    def _ros(self):
        while True:
            rospy.spin()

    def _laser(self, laser):
        laser_data = stdmsg.Laser_Scan()
        laser_data.config.angle_min = laser.angle_min #-90.0/180.0*math.pi#laser.angle_min
        laser_data.config.angle_max = laser.angle_max #90.0/180.0*math.pi#laser.angle_max
        laser_data.config.angle_increment = laser.angle_increment
        laser_data.config.range_max = 20
        laser_data.seq = self.seq

        pos,rot = self.tflistener.lookupTransform( '/odom', laser.header.frame_id,  rospy.Time(0) )#- rospy.Duration(1.0) )
        if rot[2] > 0:
            theta = 2 * math.acos( rot[3])
        else:
            theta = -2 * math.acos( rot[3])
        # translate the robot pose to laser pose
        laser_data.pose.position.x = pos[0]
        laser_data.pose.position.y = pos[1]
        laser_data.pose.orentation.yaw = theta

        pos,rot = self.tflistener.lookupTransform( '/odom', '/base_link',  rospy.Time(0) )
        if rot[2] > 0:
            theta = 2 * math.acos( rot[3])
        else:
            theta = -2 * math.acos( rot[3])
        laser_data.robot.position.x = pos[0]
        laser_data.robot.position.y = pos[1]
        laser_data.robot.orentation.yaw = theta
        laser_data.ClearField("ranges")
        
        laser_data.ranges.extend(laser.ranges )
        
        laser_data.annotation = ''
        self.seq += 1
        
        self.publish(laser_data)


    def publish(self, laser_data):
        x = laser_data.robot.position.x
        y = laser_data.robot.position.y
        theta = laser_data.robot.orentation.yaw
        laser_data.pose.position.x = x + self.laser_x * math.cos(theta) - self.laser_y * math.sin(theta)
        laser_data.pose.position.y = y + self.laser_x * math.sin(theta) + self.laser_y * math.cos(theta)
        laser_data.robot.orentation.yaw = theta + self.laser_theta
        
        cmd = stdmsg.Velocity()
        ret = self.rpc.call('update', laser_data , 1)
        if ret != None:    
            cmd.ParseFromString( ret )
        t = Twist()
        t.linear.x = cmd.v
        t.angular.z = cmd.w
        self.cmdpub.publish(t) 
        
        print('%d\t'%laser_data.seq),
        print('robot=(%.2f %.2f %.2f)'%(laser_data.robot.position.x, laser_data.robot.position.y, laser_data.robot.orentation.yaw)),
        print('cmd = (%.2f %.2f)'%(t.linear.x, t.angular.z))

    def run(self, timeout = -1):
        self.nh.run(timeout)
    
    def ok(self):
        return self.is_running
    
               
if __name__ == '__main__':
    driver = Driver()

    try:
        while driver.ok():
            driver.run(1)
    except KeyboardInterrupt:
        del driver
        print('exit')
    




    


    
