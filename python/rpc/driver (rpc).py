# -*- coding: utf-8 -*-
try:
    from logger import *
    from joystick import *
    from odometry import *
    from laser import *
except:
    print ('no driver found')

import threading, platform, copy, time, os, math

import sys; [sys.path.append( i ) for i in ['..', '../msg', './msg']]
from Node import *
import stdmsg

def create_thread(task):
    if callable(task):
        thread = threading.Thread(target = task)
        thread.setDaemon(True)
        thread.start()
        return thread
    else:
        raise 'task must be callable'

class Driver:
    log_flag = False

    cmd_lock = threading.RLock()
    cmd_v = 0; cmd_w = 0; cmd_manual = False

    odom_lock = threading.RLock()
    x = 0; y = 0; theta = 0; v = 0; w = 0; steer = 0
        
    laser_x = -0.4; laser_y = 0;laser_theta = 0

    def __init__(self, file = None):
        self.rpc = RPC()
        self.rpc.connect("tcp://127.0.0.1:9000")
        
        if file:
            self._start_playback(file)
        else:
            self._start_hardware()
           

    def _start_playback(self, filename):
        self.is_running = True
        self.filename = filename
        self.playback_thread  = create_thread(self._playback)
    def _playback(self):
        log = LogFile()
        log.open( self.filename )
        laser_data = stdmsg.Laser_Scan()

        for scan in log:
            laser_data.ParseFromString(scan)
            self.publish(laser_data)
            time.sleep(0.05)
        print('playback end')
        self.is_running = False
        
        
    def _start_hardware(self):
        self.is_running = True
        self.joy_thread = create_thread(self._joystick)
        self.odom_thread = create_thread(self._odometry)
        self.laser_thread = create_thread(self._laser)
    def _end_hardware(self):
        self.is_running = False
        self.joy_thread.join(1000)
        self.odom_thread.join(1000)
        self.laser_thread.join(1000)
        
    def _odometry(self):
        odom = Odometry( '/dev/ttyS1', 38400  )           #configure

        while self.is_running:
            with self.cmd_lock:
                cmd_v = copy.copy(self.cmd_v)
                cmd_w = copy.copy(self.cmd_w)
            odom.update( cmd_v, cmd_w)
            with self.odom_lock:
                self.x, self.y, self.theta = odom.odometry()
                self.v, self.w = odom.velocity()
            #print('vw=(%.2f %.2f) cmd=(%.2f %.2f)'%( self.v, self.w,self.cmd_v, self.cmd_w))
            
    def _joystick(self):
        joy = Joystick( 0 )               #configure
        while self.is_running:
            joy.update()
            with self.cmd_lock:
                if joy.button(9):self.cmd_manual = True
                elif joy.button(8):
                    self.cmd_manual = False
                    self.cmd_v = 0
                    self.cmd_w = 0
                if self.cmd_manual:
                    self.cmd_v = -joy.axis(3)/2.0
                    self.cmd_w = -joy.axis(2)/3.0
                    
    def _laser(self):
        laser = Laser('192.168.1.30', 2111 )                        #configure
        
        laser_data = stdmsg.Laser_Scan()
        laser_data.config.angle_min = laser.angle_min # -135.0/180.0*math.pi #
        laser_data.config.angle_max = laser.angle_max #  135.0/180.0*math.pi #
        laser_data.config.angle_increment = laser.angle_increment
        laser_data.config.range_max = 50
        laser_data.seq = 0
        count = 0
        while self.is_running:
            count += 1
            laser.read()
            if count >= 10:count = 0
            else:continue
            with self.odom_lock:
                x = copy.copy(self.x)
                y = copy.copy(self.y)
                theta = copy.copy(self.theta)
            laser_data.ClearField("ranges")
            laser_data.ranges.extend(laser.ranges[\
                    int((laser_data.config.angle_min - laser.angle_min) /laser.angle_increment) :\
                    int((laser_data.config.angle_max - laser.angle_min) /laser.angle_increment + 1) \
                ] )
            laser_data.robot.position.x = x
            laser_data.robot.position.y = y
            laser_data.robot.orentation.yaw = theta
            # translate the robot pose to laser pose
            laser_data.pose.position.x = x + self.laser_x * math.cos(theta) - self.laser_y * math.sin(theta)
            laser_data.pose.position.y = y + self.laser_x * math.sin(theta) + self.laser_y * math.cos(theta)
            laser_data.pose.orentation.yaw = theta + self.laser_theta
            laser_data.seq += 1

            self.publish(laser_data)

    def echo(self, laser_data):
            # echo the lasetada
            print('%d\t'%laser_data.seq),
            print('robot=(%.2f %.2f %.2f)'%(laser_data.robot.position.x, laser_data.robot.position.y, laser_data.robot.orentation.yaw)),
            print('laser=(%.2f %.2f %.2f)'%(laser_data.pose.position.x, laser_data.pose.position.y, laser_data.pose.orentation.yaw)),
            if hasattr(self, 'cmd_v'):print('cmd=(%.2f %.2f)'%( self.cmd_v, self.cmd_w)),
            print('L%d'% (len(laser_data.ranges)) )


    def publish(self, laser_data):
        x = laser_data.robot.position.x
        y = laser_data.robot.position.y
        theta = laser_data.robot.orentation.yaw
        laser_data.pose.position.x = x + self.laser_x * math.cos(theta) - self.laser_y * math.sin(theta)
        laser_data.pose.position.y = y + self.laser_x * math.sin(theta) + self.laser_y * math.cos(theta)
        laser_data.robot.orentation.yaw = theta + self.laser_theta

        self.echo(laser_data)
        if self.log_flag:
            self.logger.append( laser_data.SerializeToString() )
            if len(self.logger) > 100:
                self.logger.save(self.log_file)

        cmd = stdmsg.Velocity()
        ret = self.rpc.call('update', laser_data , 1)
        if ret != None:    
            cmd.ParseFromString( ret )
            
            
    def log(self, file = None):
        if file:
            self.log_flag = True
            self.log_file = file
            self.logger = LogFile()
    def run(self, timeout = -1):
        self.nh.run(timeout)
        
    def ok(self):
        return self.is_running
    

if __name__ == '__main__':
    if len(sys.argv) > 2 and sys.argv[1] =='playback':
        driver = Driver(sys.argv[2])
    elif len(sys.argv) > 2 and sys.argv[1] =='log':
        if os.path.exists(sys.argv[2]):
            confirm = raw_input("waring: %s exist, overwrite it? Y/N"%sys.argv[2])
            if confirm in ['Y', 'y', "yes", "YES", "Yes"]:
                os.remove(sys.argv[2])
            else:
                sys.argv[2] += '.new'
        driver = Driver()
        driver.log(sys.argv[2])
    else:
        driver = Driver()


    try:
        while driver.ok():
            driver.run(1)
    except KeyboardInterrupt:
        del driver
        print('exit')
    
