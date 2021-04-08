#!/usr/bin/env python

 ############################################################
 #  Created on: 2013.04                                     #
 #  Author: LIAO Wenlong                                    #
 #  Email:  Volans.liao@gmail.com                           #
 #  This is part of the Localize and Navigation Toolkit     #
 ############################################################

from PyQt4 import QtGui, QtCore, uic
from PyQt4.QtCore import Qt
import sys, math, copy, threading
from math import sin, cos, atan2
import stdmsg

class LaserScan (QtGui.QGraphicsItem):
    Type = QtGui.QGraphicsItem.UserType + 3

    def __init__(self, scan, resolution = 0.1):
        super(LaserScan, self).__init__()
        self.setAcceptDrops(False)
        self.scan = scan
        self.resolution = resolution
        self.normalize()

    def normalize(self):
        self.points = QtGui.QPolygonF()
        minx, miny, maxx, maxy = 1000, 1000, -1000, -1000

        angle = self.scan.config.angle_min + self.scan.pose.orentation.yaw
        for i in range(0, len(self.scan.ranges) ):
            if self.scan.ranges[i] >= self.scan.config.range_max - 1.5:
                angle += self.scan.config.angle_increment
                continue
            
            lx = cos( angle ) * self.scan.ranges[i] + self.scan.pose.position.x
            ly = sin( angle ) * self.scan.ranges[i] + self.scan.pose.position.y
            lx /= self.resolution
            ly /= self.resolution
            angle += self.scan.config.angle_increment
            self.points.append( QtCore.QPointF(lx, ly) )
            if (lx > maxx): maxx = lx;
            if (lx < minx): minx = lx;
            if (ly > maxy): maxy = ly;
            if (ly < miny): miny = ly;
        
        self.rect = QtCore.QRectF(minx, miny, maxx - minx, maxy - miny)

    def type(self):
        return LaserScan.Type

    def boundingRect(self):
        return self.rect

    def paint(self, qp, option, widget):
        pen = QtGui.QPen(Qt.red)
        pen.setWidth(3);
        qp.setPen(pen)
        qp.drawPoints(self.points)

class Robot (QtGui.QGraphicsItem):
    Type = QtGui.QGraphicsItem.UserType + 3

    def __init__(self, resolution = 1):
        super(Robot, self).__init__()
        self.setAcceptDrops(False)
        self.pic = QtGui.QPixmap('./pics/agv.png')
        self.scale( 1.1/(self.pic.width() / 20.0 * 12) / resolution, -1.1/(self.pic.width() / 20.0 * 12) /resolution )
        self.setZValue(1)

    def type(self):
        return Robot.Type

    def boundingRect(self):
        return QtCore.QRectF(-self.pic.width() / 2.0,- self.pic.height() / 2.0,
                              self.pic.width(), self.pic.height() ) 

    def paint(self, qp, option, widget):
        qp.translate( -self.pic.width()/2,-self.pic.height()/2)
        qp.drawPixmap(0,0,self.pic)
        
class laserWidget(QtGui.QWidget):
    resolution = 1
    def __init__(self, parent = None):
        super(laserWidget, self).__init__(parent)
        self.points = QtGui.QPolygonF()
    def setLaser(self, scan, resolution):
        self.resolution = resolution
        self.points = QtGui.QPolygonF()
        angle = scan.config.angle_min + scan.pose.orentation.yaw
        for i in range(0, len(scan.ranges) ):
            if scan.ranges[i] >= scan.config.range_max - 1.5:
                angle += scan.config.angle_increment
                continue
            
            lx = cos( angle ) * scan.ranges[i] + 1
            ly = sin( angle ) * scan.ranges[i] + 1
            lx /= self.resolution
            ly /= self.resolution
            angle += scan.config.angle_increment
            self.points.append( QtCore.QPointF(lx, ly) )
        self.repaint()
        
    def paintEvent(self, e):
        qp = QtGui.QPainter() ;
        pen = QtGui.QPen() ;
        pen.setWidth(3);
        qp.begin(self);
        qp.setPen(pen);

        gradient = QtGui.QRadialGradient(self.width()/2.0 , self.height(), self.height());
        gradient.setFocalPoint( self.width()/2.0, self.height());
        gradient.setColorAt(1, QtGui.QColor(Qt.gray).light(120));
        gradient.setColorAt(0, QtGui.QColor(Qt.gray).light(240));
        qp.setBrush(gradient);
        qp.drawRect(0,0,self.width(),self.height());
        qp.translate(self.width()/2.0 ,self.height());

        qp.rotate(-90);
        qp.drawPoints(self.points);
        qp.end();
    
class Map (QtGui.QGraphicsItem):
    Type = QtGui.QGraphicsItem.UserType + 3

    GOAL, POSE = range(0,2)
    SETMODE = None
    resolution = 0.05

    def __init__(self, map, resolution = 0.1):
        super(Map, self).__init__()
        self.map = map
        self.resolution = resolution
        self.getGoal = False
        self.setFlag(QtGui.QGraphicsItem.ItemIsSelectable,False)
        self.setAcceptDrops(False)

    def SetGetGoal(self, flag):
        self.getGoal = flag

    def type(self):
        return Map.Type

    def boundingRect(self):
        return QtCore.QRectF( 0,0, self.map.width(), self.map.height() )

    def paint(self, qp, option, widget):
        qp.setPen(Qt.red)
        qp.drawPixmap(0,0,self.map)
        if self.SETMODE != None and hasattr( self, 'startPos') and hasattr(self, 'endPos') and self.endPos != None:
            qp.drawLine(self.startPos.x(), self.startPos.y(), self.endPos.x(),self.endPos.y())

    def mousePressEvent(self, e):
        self.startPos = e.pos()
        

    def mouseMoveEvent(self, e):
        if self.SETMODE == None:
            return
        self.endPos = e.pos()
        self.update()
        

    def mouseReleaseEvent (self, e):
        if self.SETMODE == None:
            return
        self.setFlag(QtGui.QGraphicsItem.ItemIsSelectable,False)
        x = self.startPos.x() * self.resolution
        y = self.startPos.y() * self.resolution
        theta = math.atan2( e.pos().y() - self.startPos.y(), \
                                e.pos().x() - self.startPos.x() )
        
        try:
            if self.SETMODE == self.POSE:
                pos = stdmsg.Pose()
                pos.position.x = x
                pos.position.y = y
                pos.orentation.yaw = theta
                print 'initial_pose is :', x,y,theta
                if(rpc!= None):rpc.call('initial_pose', pos )
                else: print 'rpc not set'
            elif self.SETMODE == self.GOAL:
                pos = stdmsg.Pose()
                pos.position.x = x
                pos.position.y = y
                pos.orentation.yaw = theta
                print 'initial_pose is :', x,y,theta
                if(rpc!= None):rpc.call('set_goal', pos )
                else: print 'rpc not set'
        except e: 
            print e, 'socket error!'
        self.SETMODE = None
    
        
    def set_pose (self):
        self.setFlag(QtGui.QGraphicsItem.ItemIsSelectable,True)
        self.SETMODE = self.POSE
        self.endPos = None
    
    def set_goal (self):
        self.setFlag(QtGui.QGraphicsItem.ItemIsSelectable,True)
        self.SETMODE = self.GOAL
        self.endPos = None
        #print self.SETMODE

class GraphWidget(QtGui.QGraphicsView):
    resolution = 0.1
    focusOnRobot = False
    showHistoryRobot = True
    historyRobotItemList = []
    
    def __init__(self, parent):
        super(GraphWidget, self).__init__()
        self.scene = QtGui.QGraphicsScene(self)
        self.setScene(self.scene)

        self.scale(1,-1)
        self.setCacheMode(QtGui.QGraphicsView.CacheBackground)
        self.setViewportUpdateMode(QtGui.QGraphicsView.BoundingRectViewportUpdate)
        self.setTransformationAnchor(QtGui.QGraphicsView.AnchorUnderMouse)
        self.setResizeAnchor(QtGui.QGraphicsView.AnchorViewCenter)
        self.setDragMode(QtGui.QGraphicsView.ScrollHandDrag);
        self.setBackgroundBrush( QtGui.QBrush(QtGui.QColor(240,  240, 240), Qt.Dense3Pattern)) ;

        self.path = None
        self.scan = None
        self.map = None

    def init(self, r):
        global rpc
        rpc = r
        
        self.requestMap()
        self.robot = Robot(self.resolution)
        self.scene.addItem( self.robot )

        
        
    def requestMap(self):
        if self.map != None:
            self.scene.removeItem( self.map )
            
        
        rmap = stdmsg.Data()
        ret = rpc.call('map', rmap , 1)
        #print len(ret)
        if ret != None:    
            rmap.ParseFromString( ret )
        else:
            print 'rpc error !'
            
        #print len(rmap.data)
        import tempfile
        f = open('map.png', 'wb')
        f.write( rmap.data )
        f.close()

        img = QtGui.QImage('map.png')

            

        png = QtGui.QPixmap.fromImage( img )
        import struct
        s = struct.pack('I',img.pixel(0,0))
        s = s[1:3]+s[0]+s[3]
        self.resolution = struct.unpack('f',s)[0]
        self.map = Map( png, self.resolution )
        
        self.scene.addItem( self.map )
  

    def enterSetGoal(self):
        self.map.set_goal()

    def enterSetPose(self):
        self.map.set_pose()

    def wheelEvent(self, event):
        self.scaleView(math.pow(2.0, event.delta() / 240.0))

    def scaleView(self, scaleFactor):
        factor = self.matrix().scale(scaleFactor, scaleFactor).\
                mapRect(QtCore.QRectF(0, 0, 1, 1)).width()
        if factor < 0.01 or factor > 100:
            return
        self.scale(scaleFactor, scaleFactor)

    def zoom (self, num):
        factor = pow(2.713,num/50.0);
        nowFactor = self.transform().mapRect(QtCore.QRectF(0, 0, 1, 1)).width();
        if (factor < 0.07 or factor > 100):
            return;
        self.scale(factor/nowFactor, factor/nowFactor);
        
    def setFocusOnRobot (self, flag):
        self.focusOnRobot = flag;
        if self.focusOnRobot:
            self.setDragMode(QtGui.QGraphicsView.NoDrag);
            self.centerOn(self.robot);
        else:
            self.setDragMode(QtGui.QGraphicsView.ScrollHandDrag);
    def setShowHistoryRobot (self, flag):
        self.showHistoryRobot = flag;

        if( not self.showHistoryRobot):
            for item in self.historyRobotItemList:
                self.scene.removeItem(item);
            self.historyRobotItemList = []
            
    def startGoal (self):
        return
        try:
            print 'start_robot'
            client.send('start_robot ')
            print client.recv(flags=zmq.NOBLOCK)
        except:
            print 'socket error!'
    def stopGoal (self):
        return
        try:
            print 'stop_robot'
            client.send('stop_robot ')
            print client.recv(flags=zmq.NOBLOCK)
        except:
            print 'socket error!'

    def ScanHandler(self, scan):
        if self.scan != None:
            self.scene.removeItem(self.scan)
            self.scan = None

        self.scan = LaserScan(scan, self.resolution )
        self.scene.addItem(self.scan)
        
        self.robot.setPos(scan.robot.position.x / self.resolution, scan.robot.position.y / self.resolution);
        self.robot.setRotation( -scan.robot.orentation.yaw / 3.141592653*180.0);
        self.robot.update()
        if self.focusOnRobot:
            self.centerOn(self.robot)
        if self.showHistoryRobot:
            point = QtCore.QPointF (scan.robot.position.x / self.resolution - 1, \
                                    scan.robot.position.y / self.resolution - 1);
            
            self.historyRobotItemList.append( \
                        self.scene.addRect( \
                                            point.x(),point.y(), 2, 2,
                                            QtGui.QPen(Qt.blue),QtGui.QBrush(Qt.blue)\
                                           ) );

    def PlanHandler(self, plan):
        if self.path != None:
            self.scene.removeItem(self.path)
            self.path = None
        plan = plan.path
        if len(plan) == 0:
            return

        path = QtGui.QPainterPath()
        path.moveTo( plan[0].position.x/self.resolution, plan[0].position.y/self.resolution )
        for i in plan[1:]:
            path.lineTo( i.position.x/self.resolution,i.position.y/self.resolution )
        #for i in plan[1:]:
        #    path.addEllipse( i[0]/0.1 - 0.5, i[1]/0.1 - 0.5, 1, 1)
        self.path = self.scene.addPath(path)
        self.path.setZValue(10)
    
