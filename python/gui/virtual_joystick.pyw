#!/usr/bin/env python
import sys, inspect, os, stdmsg, Node

from PyQt4 import QtGui, QtCore
from PyQt4 import QtGui, QtCore
nh = Node.Node("tcp://*:9010")

vel = stdmsg.Velocity()
vel.v = 0
vel.w = 0
nh.publish('velocity', vel)
rpc = Node.RPC()
rpc.connect('tcp://192.168.0.100:9000')
cal = stdmsg.String()
cal.str = 'MANUAL'
rpc.call('set_mode',cal, 1)
class MainWindow(QtGui.QMainWindow):   
    x_min = -0.50
    x_max = 0.50
    r_min =  -0.5
    r_max =  0.5
    def __init__(self):  
        super(MainWindow, self).__init__()
        self.timer_rate = 50

        self.initUI()
           
    def initUI(self):      
        img_path = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe()))) + "\crosshair.jpg"
        img_path = img_path.replace('\\', '/')
        self.statusBar()
        self.setStyleSheet("border-image: url(%s); " % img_path)
        
        self.setGeometry(100, 100, 200, 200)
        self.setWindowTitle('Virtual Joystick')
        self.show()
        self.timer = QtCore.QBasicTimer()
        self.statusBar().showMessage('started')
        
    def mousePressEvent(self, event):
        self.statusBar().showMessage('mouse clicked')
        self.timer.start(self.timer_rate, self)
        self.get_position(event)
           
    def mouseReleaseEvent(self, event): 
        self.statusBar().showMessage('mouse released')
        self.timer.stop()
        vel = stdmsg.Velocity()
        vel.v = 0
        vel.w = 0
        nh.publish('velocity', vel)
          
    def mouseMoveEvent(self, event):  
        self.get_position(event)
        
    def get_position(self, event): 
        s = self.size()
        s_w = s.width()
        s_h = s.height()
        pos = event.pos()
        self.x = 1.0 * pos.x() / s_w
        self.y = 1.0 * pos.y() / s_h
        
    def timerEvent(self, event): 
        self.pubTwist()
        
    def pubTwist(self):
        v = (1-self.y) * (self.x_max - self.x_min) + self.x_min
        w = (1-self.x) * (self.r_max - self.r_min) + self.r_min
        
        if v > self.x_max:
            v = self.x_max
        if v < self.x_min:
            v = self.x_min
        if w > self.r_max:
            w = self.r_max
        if w < self.r_min:
            w = self.r_min
    
        self.statusBar().showMessage('vel (%0.2f, %0.2f)' % (v, w))
        vel = stdmsg.Velocity()
        vel.v = v
        vel.w = w
        nh.publish('velocity', vel)

if __name__ == '__main__':
    app = QtGui.QApplication(sys.argv)
    ex = MainWindow()
    sys.exit(app.exec_())