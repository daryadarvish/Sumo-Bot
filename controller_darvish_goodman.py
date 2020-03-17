# -*- coding: utf-8 -*-
"""
Spyder Editor

This is a temporary script file.
"""


''' @file main.py
There must be a docstring at the beginning of a Python
source file with an @file [filename] tag in it! '''
import utime

class Controller:
    ''' This implements a closed-loop proportional controller on the DC motor
    system using the position sensor and motor driver to improve accuracy,
    speed, and efficiency.'''
    def __init__ (self, setup_gain, setup_point, current_time):
        '''Initializes the set-point and proportional gain of the system.'''
        
        self.point = setup_point
        self.gain  = setup_gain
        self.time_list = []
        self.pos_list = []    
        self.pos = 0
        self.current_time = utime.ticks_ms()
        self.error = 0
        self.integral_gain = 0.01
        self.esum_max = 2500
        self.esum =  0
        
    def go (self, position):
        ''' Runs the control algorithm and produces a duty cycle. '''
        #self.pos_list.append(position)
        #self.time_list.append(utime.ticks_ms()-self.current_time)
        #print('{:},{:}\r'.format(utime.ticks_ms()-self.current_time, position)
        self.pos = position
        self.error = self.point - position
        self.esum = self.esum + self.error
        if self.esum > self.esum_max:
            self.esum = self.esum_max
        duty_cycle = self.error*self.gain + self.esum*self.integral_gain
        #if duty_cycle < 0:
        #    duty_cycle = abs(self.duty_cycle)
        return duty_cycle
    
    def set_point (self, setup_point):
        ''' Allows the user to change the set point of the system.'''
        self.point = setup_point
        return 'Setting desired point to: ' + str(setup_point)
    
    def set_gain (self, setup_gain):
        '''Allows the user to change the proportional gain.'''
        
        self.gain = setup_gain
        return 'Setting desired proportional gain to: ' + str(setup_gain)
    
#    def print_stats (self):
 #       """ prints time and position data to be read through serial port"""
  #      for i in range(len(self.time_list)):
   #         print('{:},{:}\r'.format(self.time_list[i], self.pos_list[i]))
            
    def reset_stats (self):
        """resets the lists holding position and time data"""
        self.time_list = []
        self.pos_list = []
        
    
    def get_pos (self):
        return self.pos
    
    def get_current_time(self):
        return self.current_time