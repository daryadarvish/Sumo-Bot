# -*- coding: utf-8 -*-
"""
Created on Fri Feb 28 11:26:44 2020

@author: melab15
"""

import pyb
import time

class LineTracker:
    ''' This class implements a motor driver for the
    ME405 board. '''
    def __init__ (self, pin_1, pin_2, pin_3):
        ''' Creates a motor driver by initializing GPIO
        pins and turning the motor off for safety. '''
        #print('Creating a motor driver')
        self.pin1 = pyb.Pin (pin_1, pyb.Pin.IN)
        self.pin2 = pyb.Pin (pin_2, pyb.Pin.IN)
        self.pin3 = pyb.Pin (pin_3, pyb.Pin.IN)
        
    def read(self):
        return (str(self.pin1.value()) + str(self.pin2.value()) +str(self.pin3.value()))
    
    def read_L(self):
        return self.pin1.value()

    def read_C(self):
        return self.pin2.value()
        
    def read_R(self):
        return self.pin3.value()


    
    
if __name__ == '__main__':
    yo = LineTracker(pyb.Pin.board.PB8, pyb.Pin.board.PB9, pyb.Pin.board.PB3)
    while True:
        time.sleep(1)
        print(yo.read())

