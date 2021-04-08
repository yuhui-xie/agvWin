#!/usr/bin/env python

 ############################################################
 #  Created on: 2013.04                                     #
 #  Author: LIAO Wenlong                                    #
 #  Email:  Volans.liao@gmail.com                           #
 #  This is part of the Localize and Navigation Toolkit     #
 ############################################################

import sys, platform, ctypes ,time, array, serial, threading, struct
import copy, os, math
          
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
    angle = []
    
    def __init__(self, tty, baudrate):
        self.ser = serial.Serial(tty, baudrate=baudrate, timeout=1)
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
        return -left, right, angle

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

    def steer(self):
        angle = 0
        for i in self.angle: angle += i
        if len(self.angle) ==0 :
            return angle/len(self.angle)
        else:
            return 0
        
    def update(self, v = 0, w = 0):
        if v - self.last_v >  0.02: v = self.last_v + 0.02
        if v - self.last_v < -0.02: v = self.last_v - 0.02
        if w - self.last_w >  0.02: w = self.last_w + 0.02
        if w - self.last_w < -0.02: w = self.last_w - 0.02
        try:
            left, right, angle = self.read()
            angle -= 620
            angle /= 200.0
            if len(self.angle)< 5: 
                self.angle.append(angle)
            else:
                self.angle = self.angle[1: ]
                self.angle.append(angle)
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
                
from joystick import *
import time, array, threading, copy,  math


def create_thread(task):
    if callable(task):
        thread = threading.Thread(target = task)
        thread.setDaemon(True)
        thread.start()
        return thread
    else:
        raise 'task must be callable'

class Driver:
    cmd_lock = threading.RLock()
    cmd_v = 0; cmd_w = 0; cmd_manual = False

    odom_lock = threading.RLock()
    x = 0; y = 0; theta = 0; v = 0; w = 0; steer = 0
   

    def __init__(self, tty , baudrate, joy_index = 0):
        self.is_running = True
        
        
        self.odom = Odometry(tty, baudrate  )
        try:       
            self.joy = Joystick( joy_index )  
            self.joy_thread = create_thread(self._joystick)
        except:
            print 'no joystick found'
            
    def __del__(self):
        self.is_running = False
        self.joy_thread.join(1000)


    def update(self, v, w):
        with self.cmd_lock:
            if not self.cmd_manual:
                self.cmd_v = v; self.cmd_w = w
            cmd_v = copy.copy(self.cmd_v)
            cmd_w = copy.copy(self.cmd_w)
        self.odom.update( cmd_v, cmd_w)
        with self.odom_lock:
            self.x, self.y, self.theta = self.odom.odometry()
            self.v, self.w = self.odom.velocity()
            self.steer = self.odom.steer()
    def _joystick(self):
        while self.is_running:
            self.joy.update()
            with self.cmd_lock:
                if self.joy.button(9):self.cmd_manual = True
                elif self.joy.button(8):
                    self.cmd_manual = False
                    self.cmd_v = 0
                    self.cmd_w = 0
                if self.cmd_manual:
                    self.cmd_v = -self.joy.axis(3)/2.0
                    self.cmd_w = -self.joy.axis(2)/3.0
                             
#driver = Driver('/dev/ttyS1', 38400, 0)
driver = Driver('com9',19200,0)
def update(v,w):
    driver.update(v,w)
    x, y, t = driver.odom.odometry()
    v, w = driver.odom.velocity()
    steer = driver.odom.steer()
    #the pose is in the frame of body
    t = t - steer
    return x, y, t, v, w, steer
    
while __name__ == '__main__':
    x, y, t, v, w, steer = update(0,0)
    print('pos=(%.2f,%.2f,%.2f)\tvw=(%.2f,%.2f)\tsteer=%.2f'%(x, y, t, v, w, steer)),
    print('cmd=(%.2f,%.2f)'%(driver.cmd_v, driver.cmd_w))
    
    
    
    
