#!/usr/bin/env python

 ############################################################
 #  Created on: 2013.04                                     #
 #  Author: LIAO Wenlong                                    #
 #  Email:  Volans.liao@gmail.com                           #
 #  This is part of the Localize and Navigation Toolkit     #
 ############################################################

import sys,time,copy
import Node
SPLIT = '\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF'
class LogFile:
    scans = []
    def __init__(self):
        self._index = 0

    def open(self, filename):
        raw = open(filename, 'rb').read().split(SPLIT)
        self.scans = []
        for serial in raw:
            if serial == "": continue
            #scan = Laser_Scan()
            #scan.ParseFromString(serial)
            self.scans.append( copy.copy(serial) )
        self._index = 0

    def __len__(self):
        return len(self.scans)
    def __getitem__(self, index):
        return self.scans[index]
    def __iter__(self):
        return self
    def next(self):
        if self._index >= len(self.scans):
            self._index = 0
            raise StopIteration()
        else:
            self._index += 1
            return self.scans[self._index - 1]
    def clear(self):
        self.scans = []

    def append(self, scan):
        self.scans.append( scan )

    def remove(self, index):
        self.scans.pop(index)

    def save(self, filename):
        logFile = open(filename, 'ab+')
        for scan in self.scans:
            logFile.write(scan)
            logFile.write(SPLIT)
        logFile.close()
        self.clear()

if __name__ == '__main__':
    if len(sys.argv) == 1:
        print('please specify the logfile')
        sys.exit()
      
    log = LogFile()
    
    import Node
    nh = Node.Node('tcp://127.0.0.1:5000')
    nh.connect('tcp://127.0.0.1:6000')
    nh.subscrible('laser',log.append)
    try:
        while True:
            nh.run()  
    except:
        # maybe a fatal error occur! save the log is important
        print('save scan to file')
        log.save( sys.argv[1] )
        sys.exit()
        
    log.save( sys.argv[1] )
