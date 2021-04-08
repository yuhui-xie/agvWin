#!/usr/bin/env python2

 ############################################################
 #  Created on: 2013.04                                     #
 #  Author: LIAO Wenlong                                    #
 #  Email:  Volans.liao@gmail.com                           #
 #  This is part of the Localize and Navigation Toolkit     #
 ############################################################


import sys, serial, struct, time,threading


BAUD_RATES = (  
    300,600,1200,2400,4800,9600,14400,19200,
    28800,38400,57600,115200)


class Odometry(object):
    
    x = 0.0;y = 0.0;theta = 0.0; v_cur = 0.0; w_cur = 0.0; v_cmd = 0.0; w_cmd = 0.0;
    js = 0;    
    START ='\xEE\xAA'
    END ='\x00\xBB'

    def __init__(self, tty, baudrate):
        # serial
        self.ser = serial.Serial(tty, baudrate=baudrate, timeout=2)
        self.buffer = ''
        #self.ser.open()
        self.ser.write('\xEE\xAA\x01\x00\x00\x01\x00\x00\x02\x00\x00\xBB')
        self.lock = threading.RLock()

    def read(self):
        #print self.ser.inWaiting() != 0
        #find tou and weiba 
        while self.buffer.find(self.START)<0 or ( self.buffer[self.buffer.find(self.START)+len(self.START):] ).find(self.END) < 0:
            self.buffer += self.ser.read(1)
    
            
        self.buffer = self.buffer.split(self.START,2)[1]
        str = self.buffer[0:10]
    #    print len(str)
        self.buffer = self.buffer[10:]
        
        if(len(str)<10):
            return

        with self.lock:
            self.x, self.y, self.theta, self.v_cur,self.w_cur =  struct.unpack('>3h2b', str[0:8])
            #print 5
            self.x = self.x/100.0
            self.y = self.y/100.0
            self.theta = self.theta / 180.0 * 3.141592653
            self.v_cur /= 100.0; self.w_cur /= 100.0; 
            if self.theta > 3.1415926: 
                self.theta -= 2*3.1415926
            if self.theta < -3.1415926:
                self.theta += 2*3.1415926

        '''str1 = self.ser.read(2)
            while self.ser.read(2) != '\xEE\xAA' :
               print str1
                                                                                        
               # self.js += 1
               # if self.js >= 8:
               #     self.ser.read(1)
            #    pass
           # self.js = 0
           # str = self.ser.read(10)
           # self.x,self.y,self.theta,\
           #     self.v_cur,self.w_cur, self.joystick_v,self.joystick_w\
           #     =  struct.unpack('>3h4b', str)
            #self.x = self.x/100.0
           # self.y = self.y/100.0
           # self.v_cur /= 100.0; self.w_cur /= 100.0; 
           # if abs(self.joystick_v)<=6:
           #     self.joystick_v = 0
          #  if abs(self.joystick_w)<=6:
               self.joystick_w = 0
            self.joystick_v /= 150.0; self.joystick_w  /= 150.0
            self.theta = self.theta / 180.0 *3.141592653
            print self.x,self.y,self.theta,\
                self.v_cur,self.w_cur, self.joystick_v, self.joystick_w'''
                
    def write (self):
        with self.lock:
            if self.cmd_v > 0:
                vsign = 0x01
                vabs = self.cmd_v * 1000.00
            else:
                vsign = 0x02
                vabs = -self.cmd_v * 1000.00
            if self.cmd_w > 0:
                wsign = 0x01
                wabs = self.cmd_w * 180 /3.141592653
            else:
                wsign = 0x02
                wabs = - self.cmd_w * 180 /3.141592653
            #print  vsign, vabs, wsign, wabs
            data = struct.pack('>3BhBh4B', 0xee, 0xaa, vsign, vabs, wsign, wabs, 0x00, 0x00, 0x00, 0xBB)

            self.ser.write(data);
            
    def set_velocity (self, v, w):
        with self.lock:
            self.cmd_v = v
            self.cmd_w = w
    
    def close(self):
        self.ser.flush()
        self.ser.close()
    
    def update(self, v, w):
        self.set_velocity(v,w)
        self.write()
        self.read()
        
    def steer(self):
        return 0
    def odometry(self):
        return self.x, self.y, self.theta
        
    def velocity(self):
        return self.v_cur, self.w_cur

             
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
   

    def __init__(self, tty, baudrate, joy_index = 0):
        self.is_running = True
        
        try:
            from joystick import *
            self.joy = Joystick( joy_index ) 
            self.joy_thread = create_thread(self._joystick)
        except:
            print 'no joystick found'
           
        self.odom = Odometry(tty, baudrate  )    


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
                if self.joy.button(5):self.cmd_manual = True
                elif self.joy.button(4):
                    self.cmd_manual = False
                    self.cmd_v = 0
                    self.cmd_w = 0
                if self.cmd_manual:
                    #self.cmd_v = 0.16
                    self.cmd_v = -self.joy.axis(1)/6.0
                    self.cmd_w = -self.joy.axis(0)/6.0
                             
#driver = Driver("/dev/ttyUSB0", 19200)
driver = Driver('com4',19200,0)
def update(v,w):
    driver.update(v,w)
    x, y, t = driver.odom.odometry()
    v, w = driver.odom.velocity()
    steer = driver.odom.steer()
    #the pose is in the frame of body
    t = t - steer
    return x, y, t, v, w, steer
#jilu = open('julu.txt','w')    
while __name__ == '__main__':
    
    #dcmd = "%.2f,%.2f" %(driver.cmd_v, driver.cmd_w)
   # jilu.write(dcmd)
    #jilu.write('\n')
    x, y, t, v, w, steer = update(0,0)
    print('pos=(%.2f,%.2f,%.2f)\tvw=(%.2f,%.2f)\tsteer=%.2f'%(x, y, t, v, w, steer)),
    print('cmd=(%.2f,%.3f)'%(driver.cmd_v, driver.cmd_w))
