#!/usr/bin/env python

 ############################################################
 #  Created on: 2013.04                                     #
 #  Author: LIAO Wenlong                                    #
 #  Email:  Volans.liao@gmail.com                           #
 #  This is part of the Localize and Navigation Toolkit     #
 ############################################################

import sys, platform, ctypes ,time, array


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
        

if __name__== '__main__':
    import time
    joy = Joystick(0)
    while True:
        joy.update()
        print joy._axis
        #print joy.axis(0), joy.button(0)
        #print ctypes.c_char_p( SDL.SDL_GetError() ).value
        #time.sleep(0.1)
        
        
