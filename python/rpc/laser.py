#!/usr/bin/env python

 ############################################################
 #  Created on: 2013.04                                     #
 #  Author: LIAO Wenlong                                    #
 #  Email:  Volans.liao@gmail.com                           #
 #  This is part of the Localize and Navigation Toolkit     #
 ############################################################
# test
import socket, string, time, copy, sys, threading

# for SICK LMS111
class Laser(object):
    angle_increment = 0
    angle_min = 0
    angle_max = 0
    ranges = []
    def __init__ (self, host = '192.168.1.105', port = 2112):
        #connect to the laser
        print( 'connect the device : '+ host+':'+str(port))
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.connect((host, port))
        #setup the device
        #print( 'setup the device' )
        #self.sock.send('\x02sMN SetAccessMode 03 F4724744\x03')
        #self.sock.recv(100)
        
        #self.sock.send('\x02sMN mLMPsetscancfg +2500 +1 +2500 -450000 +450000\x03')
        #self.sock.recv(100)
        #get the laser config
        print( 'get the device config:' ),
        sys.stdout.flush()        
        self.sock.send("\x02sRN LMPscancfg\x03")
        data = self.sock.recv(100)
        data = data.replace('\x03',' ')
        data = data.split()
        self.angle_increment = int(data[4],16)/10000.0 * 3.141592653/180;
        self.angle_min = int(data[5],16)
        if self.angle_min > 0x8000000:
            self.angle_min = self.angle_min - 1
            self.angle_min = self.angle_min - 0xFFFFFFFF
        self.angle_min = self.angle_min/10000.0 *3.141592653/180 - 3.141592653/2;
        self.angle_max = int(data[6],16)/10000.0 *3.141592653/180 - 3.141592653/2;
        print('angle = %.4f:%.4f:%.4f'%( self.angle_min, self.angle_increment, self.angle_max))
        #self.laserData.set_param(self.angle_increment, self.angle_min, self.angle_max)
        self.ranges = [0] * int( (self.angle_max - self.angle_min)/self.angle_increment+1)
        #startup the device
        print('startup the device'),
        sys.stdout.flush()        
        self.sock.send('\x02sWN LMDscandatacfg 01 00 1 1 0 00 00 0 0 0 1 +1\x03')
        self.sock.recv(100)
        self.sock.send('\x02sMN LMCstartmeas\x03')
        self.sock.recv(100)
        #wait it ok
        status = 0
        while status != 7:
            print('.'),
            sys.stdout.flush()            
            self.sock.send('\x02sRN STlms\x03')
            data = self.sock.recv(100)
            data = data.split()
            status = int(data[2],16)
            time.sleep(0.1)
            self.sock.send('\x02sEN LMDscandata 1\x03')
            self.sock.recv(100)
        print 'success!'
        #thread 
        self.lock = threading.RLock()
        self.readThread = None    
            
        self.buffer = ''
        

    def read(self):
        #print 'start read'
        while self.buffer.find('\03') < 0:
            #print self.buffer
            self.buffer = self.buffer + self.sock.recv(5120)
        data = self.buffer.split('\03')
        # remain sth to process
        if len(data) > 1:
            self.buffer = data[-1]
        else:
            self.buffer = ''
        if data[0].find('DIST1') <0:
            return 
        #get the useful data
        data = data[0]
        data = data.split('DIST1');
        data = data[1]#data contain rangedata + 'RSSI1' + Rdata
        data = data.split('RSSI1');
        ranges_str = data[0].split()
        ranges = self.ranges
        try:
            self.ranges = [int(ranges_str[i+5],16)/1000.00 for i in range(0,int( (self.angle_max - self.angle_min)/self.angle_increment + 1 ) )]
        except:
            self.ranges = ranges
            print('no enough data')
        #intensities_str = data[1].split()
        '''for i in range(0,len(self.):
            self.laserData.ranges[i] = int(ranges_str[i+5],16)/1000.00
            if self.laserData.ranges[i] < 0.1:
                self.laserData.ranges[i] = 20.0
            #self.laser_data.intensities[i] = int(intensities_str[i+5],16)
'''

    
    def __del__(self):
        #self.Stop()
        #stop the device
        print('stop the device')
        self.sock.send( "\x02sMN LMCstopmeas\x03")
        self.sock.recv(100)
        time.sleep(0.00001);
        self.sock.send("\x02sEN LMDscandata 0\x03")
        self.sock.recv(100)
        self.sock.close()
        print('device closed!')



    
if __name__ == "__main__":
    # initialize a node which provide the capability to
    #    publish the laser scan 
    #nh = middleware.Node('laser')
    
    laser = Laser('192.168.1.30',2111)
    
    # loop begin
    try: 
        while True:
            laser.read()
            print 'F%d'%len(laser.ranges)
            sys.stdout.flush() 
    except KeyboardInterrupt: 
        # close the laser
        pass
        #laser.Close()
