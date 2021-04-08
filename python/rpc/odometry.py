#!/usr/bin/env python

 ############################################################
 #  Created on: 2013.04                                     #
 #  Author: LIAO Wenlong                                    #
 #  Email:  Volans.liao@gmail.com                           #
 #  This is part of the Localize and Navigation Toolkit     #
 ############################################################

import sys
import serial, platform, threading, time, struct
#from robot import Pose
#import middleware

import math
class Kinematics:
    x = 0
    y = 0
    theta = 0
    v = 0
    w = 0

    def set_parameter(self, left_wheel, right_wheel, space, _time):
        self.r_left_wheel  = left_wheel
        self.r_right_wheel = right_wheel
        self.space         = space
        self.T             = _time
        self.vw = [ [self.r_right_wheel * 0.5,          self.r_left_wheel * 0.5],
                    [self.r_right_wheel / self.space , -self.r_left_wheel /self.space ] ]
        det = (self.vw[0][0]*self.vw[1][1] - self.vw[0][1]*self.vw[1][0])
        self.vw_inverse = [ [ self.vw[1][1] / det, -self.vw[0][1] / det],
                            [-self.vw[1][0] / det,  self.vw[0][0] / det] ]

    def update(self, delta_left, delta_right):
        if not hasattr(self, 'r_left_wheel'):
            raise 'please call set_parameter first'
        xyt = [[math.cos(self.theta),0],
               [math.sin(self.theta),0],
               [0,1]]
        self.v = self.vw[0][0] * delta_right + self.vw[0][1] * delta_left
        self.w = self.vw[1][0] * delta_right + self.vw[1][1] * delta_left

        delta_x      = xyt[0][0] * self.v + xyt[0][1] * self.w
        delta_y      = xyt[1][0] * self.v + xyt[1][1] * self.w
        delta_theta  = xyt[2][0] * self.v + xyt[2][1] * self.w
        self.x += delta_x
        self.y += delta_y
        self.theta += delta_theta

        while self.theta > math.pi:
            self.theta -= 2 * math.pi
        while self.theta < -math.pi:
            self.theta += 2 * math.pi
        self.v = self.v / self.T
        self.w = self.w / self.T

    def inverse(self, v, w):
        if not hasattr(self, 'r_left_wheel'):
            raise 'please call set_parameter first'
        v *= self.T
        w *= self.T
        delta_right = self.vw_inverse[0][0] * v + self.vw_inverse[0][1] * w
        delta_left  = self.vw_inverse[1][0] * v + self.vw_inverse[1][1] * w
        
        return delta_left, delta_right

    def velocity(self):
        return self.v, self.w

    def odometry(self):
        return self.x, self.y, self.theta


class Odometry(object, Kinematics):
    START = '\xEE\xAA'
    END = '\x00\xBB'
    sigma_left = 0
    sigma_right = 0
    last_v = 0
    last_w = 0
    def __init__(self, tty, baudrate):
        self.ser = serial.Serial(tty, baudrate=baudrate, timeout=2)
        if platform.system() == 'Linux': 
            self.ser.open()
        self.buffer = ''
        Kinematics.set_parameter(self, 0.0000120, 0.0000120, 0.39, 0.01)        #kinamatic configure

    def read(self):
        while self.buffer.find(self.START)<0 or self.buffer[self.buffer.find(self.START)+len(self.START):].find(self.END) <0:
            self.buffer += self.ser.read(1)
            #print `self.buffer`
        #print `self.buffer`
        self.buffer = self.buffer.split(self.START,2)[1]
        
        datastr = self.buffer.split(self.END, 2)
        if len(datastr) > 1: self.buffer = datastr[1]
        else: self.buffer = ''
        str = datastr[0]
        #print `str`
        left,right, angle = struct.unpack('>3h', str)
        #print left, right, angle
        return -left, right

    def write(self, left, right):
        #if abs(left)<600:left = 0
        #if abs(right)<600: right = 0
        data = struct.pack('>4h', abs(left), abs(right), left<0, right<0)
        data = data.replace('\xff','\xfe')
        data = '\xFF'+data+'\xEE'
        #print `data`
        self.ser.write(data)

    def __del__(self):
        self.ser.flush()
        self.ser.close()

    def update(self, v = 0, w = 0):
        #v = v / 2.0
        #w = w / 4.0i
        #if (v) <0.01 and abs(w) < 0.01:
        #    v = 0
        #    w = 0
        if v - self.last_v >  0.02: v = self.last_v + 0.02
        if v - self.last_v < -0.02: v = self.last_v - 0.02
        if w - self.last_w >  0.02: w = self.last_w + 0.02
        if w - self.last_w < -0.02: w = self.last_w - 0.02
        try:left, right = self.read()
        except:
            left = 0
            right = 0
        Kinematics.update(self, left, right)

        target_left, target_right = Kinematics.inverse(self, v, w)
        #print target_left, target_right
        left_error = target_left - left
        self.sigma_left += left_error
        output_left = target_left * 10.0 + left_error * 15
        if self.sigma_left > 2000:
            self.sigma_left = 2000
        elif self.sigma_left < -2000:
            self.sigma_left = -2000
        #output_left += self.sigma_left * 1.5
        
        right_error = target_right - right
        self.sigma_right += right_error
        output_right = target_right * 10.0 + right_error * 15
        if self.sigma_right > 2000:
            self.sigma_right = 2000
        elif self.sigma_right < -2000:
            self.sigma_right = -2000 
        #output_right += self.sigma_right * 1.5
        
        if abs(v) <0.05 and abs(w) < 0.05:
            output_left = 0
            output_right = 0
            self.sigma_right = 0
            self.sigma_left = 0
        #print time.time(), '\t',left, right, int(target_left), int(target_right),
        #print int(output_left),int( output_right), self.sigma_left, self.sigma_right
        
        if output_left > 12000:output_left = 12000
        elif output_left <-12000:output_left = -12000
        if output_right > 12000:output_right = 12000
        elif output_right <-12000:output_right = -12000
        
        self.last_v = v
        self.last_w = w
        self.write(output_left, output_right)
        #print output_left, output_right

if __name__ == "__main__":
    if platform.system() == 'Windows':
        odom = Odometry("COM2", 38400)
    else:
        odom = Odometry("/dev/ttyS1", 38400) 
    import time
    R = 0
    L = 0
    while True:
        odom.write(2000,2000)
        left, right = odom.read()
        L += left
        R += right
        print L, R

    #odom = FakeOdometry()
    
    #middleware.SetRemoteObject( 'odometry.luneng', odom)
        
    #odom.Stop()
    #odom.Close()
        


