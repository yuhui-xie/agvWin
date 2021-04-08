import sys, platform, ctypes ,time, array, threading, struct
import copy, os, math

from stat import *
[sys.path.append( i ) for i in ['..', '../msg', './msg','./gui']]
import Node, stdmsg

if platform.platform().find('Windows') >= 0:
    if sys.version.find('64 bit') >= 0:
        SDL = ctypes.cdll.LoadLibrary('x64_SDL2.dll')
    else:
        SDL = ctypes.cdll.LoadLibrary('x86_SDL2.dll')

    class Joystick:
        joystick = None
        def __init__ (self, index = 0):
            SDL.SDL_InitSubSystem(ctypes.c_uint(0x00000200))
    
            num_of_joystick = SDL.SDL_NumJoysticks()
            if num_of_joystick <= 0:
                print 'no joystick found!'
            else:
                self.joystick = ctypes.c_void_p( SDL.SDL_JoystickOpen (ctypes.c_int( index ) ) )
                name_of_joystick = ctypes.c_char_p( SDL.SDL_JoystickName(self.joystick) ).value
                print num_of_joystick, 'joystick found! And', name_of_joystick, 'is opened'
                
            SDL.SDL_JoystickEventState(ctypes.c_int(-1))
            self.axis_num = SDL.SDL_JoystickNumAxes(self.joystick)  
            #self.button_num = SDL.SDL_JoystickNumButton(self.joystick)    
        
    
        def __del__ (self):
            SDL.SDL_JoystickClose(self.joystick)
            
        def update(self):
            SDL.SDL_JoystickUpdate()
            
        def axis (self, index):      
            return ctypes.c_short( SDL.SDL_JoystickGetAxis(self.joystick,ctypes.c_int(index)) ).value / 32768.0
            
        def button (self, index):
            return ctypes.c_uint8( SDL.SDL_JoystickGetButton ( self.joystick, ctypes.c_int(index) ) ).value
else:
    import fcntl, os
    class js_event(ctypes.Structure):
        _fields_=[('time',ctypes.c_uint),
                  ('value',ctypes.c_int16),
                  ('type',ctypes.c_uint8),
                  ('number',ctypes.c_int8)]
              
    class Joystick:
        _axis ={}
        _button = {}
        def __init__(self, index):
            self.joy = os.open('/dev/input/js%d'%index, os.O_RDONLY)
            #buf = array.array('h', [0])
            #fcntl.ioctl(joy, 0x80016a11, buf)
            fcntl.fcntl(self.joy, fcntl.F_SETFL)
            #fcntl.fcntl(joy, os.O_NONBLOCK)
            
        def update(self):
            se = os.read(self.joy, ctypes.sizeof(js_event))
            e = js_event()
            ctypes.memmove(ctypes.addressof(e), se, ctypes.sizeof(e))
            if(e.type == 1):
                self._button[e.number] = e.value
            else:
                self._axis[e.number] = e.value / 32768.0
            
        def axis(self, index):
            if index in self._axis:
                return self._axis[index]

        def button(self, index):
            if index in self._button:
                return self._button[index]

try:joy = Joystick( 0 )                
except:joy = None

def create_thread(task):
    if callable(task):
        thread = threading.Thread(target = task)
        thread.setDaemon(True)
        thread.start()
        return thread
    else:
        raise 'task must be callable'
def say( file ):
    import winsound

    file = 'wav/'+file + ".wav"
    winsound.PlaySound(file, winsound.SND_NODEFAULT)
    
class Explorer:
    def __init__(self, joy_index = 0):
        self.is_running = True
        try:
            self.rpc = Node.RPC()
            self.rpc.connect("tcp://127.0.0.1:9000")
            self.joy = joy
            self.joy.update()
        except:
            print 'no joystick found'
            raw_input()
            sys.exit()
        create_thread( self.run )    
    def run(self):
        os.system('explore.exe')
    def check(self):
        time.sleep(1)
        while True:
            time.sleep(0.1)
            self.joy.update()
            #print self.joy.button(0)
            if self.joy.button(0):
                break
        print "save map"
        cal = stdmsg.String()
        cal.str = 'end'
        ret = self.rpc.call('maker', cal)
        cal.ParseFromString( ret )
        self.exit()
    def exit(self):
        t = 0
        while time.time() - t > 10:
            try:
                t = os.stat('map.png')[ST_MTIME]
            except:
                t = 0
            time.sleep(1)
        print 'exit'
        os.system("taskkill /f /t /im explore.exe")

    
from ConfigParser import SafeConfigParser
parser = SafeConfigParser()
try:
    parser.read('gui.ini')
    HOST = parser.get('url', 'host')
    PORT_RPC = parser.get('url', 'rpc')
    PORT_SUB = parser.get('url', 'sub')
except :
    HOST = "127.0.0.1"
    PORT_RPC = '9000'
    PORT_SUB = '9001'''
class Navigator:
    def __init__(self,  ):
        
        self.joy = joy
        create_thread( self.run )
        time.sleep(5)
        
        print 'time is up'
        self.connect_host()
    def connect_host(self):      
        self.rpc = Node.RPC()
        self.nh = Node.Node("tcp://*:5165")
        
        self.nh.subscrible('laser',self.ScanHandler)

        print 'connect to', HOST
        self.nh.subscrible('scan',self.ScanHandler)
        self.nh.subscrible('global_plan',self.PlanHandler)  
        
        self.rpc.connect("tcp://"+HOST+":"+PORT_RPC)
        self.nh.connect( "tcp://"+HOST+":"+PORT_SUB)
       
    def check(self):
        while True:
            self.nh.run(0.1)

    def run (self):
        os.system('navigator.exe')
    def set_path(self, pose_list):
        path = stdmsg.Global_Plan()
        for pose in pose_list:
            msgpose = path.path.add()
            msgpose.position.x = pose[0]
            msgpose.position.y = pose[1]
            msgpose.orentation.yaw = pose[2]
        ret = self.rpc.call('set_path', path)
        path.ParseFromString( ret )

    @Node.msg_callback(stdmsg.Laser_Scan)
    def ScanHandler(self, scan):
        x = scan.robot.position.x
        y = scan.robot.position.y
        t = scan.robot.orentation.yaw
        print x, y, t
        if self.joy:
            self.joy.update()
            if self.joy.button(1):
                say( 'recorded' )
                file = open('path.txt', 'a')
                file.write( '111 %f %f %f\n'%(x,y,t) )
                file.close()


    @Node.msg_callback(stdmsg.Global_Plan)
    def PlanHandler(self, plan):
        pass


say( 'startup' )

if joy:
    say('guide')
    t = time.time()
    while time.time() - t < 0:
        time.sleep(0.1)
        joy.update()
        if joy.button(2):
            d = Explorer()
            d.check()
            say( 'completed')
            break

    
say( 'navigator' )
n = Navigator()
n.check()