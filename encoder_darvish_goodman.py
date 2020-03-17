# -*- coding: utf-8 -*-
"""
Spyder Editor

This is a temporary script file.
"""

''' @file main.py
There must be a docstring at the beginning of a Python
source file with an @file [filename] tag in it! '''

import pyb

class Encoder():
    ''' This class implements a timer module that reads the positional output of
    a motor connected to the ME 405 board. 
    '''
    def __init__ (self, timer_channel, pin1, pin2):
        ''' Initializes the timer channels and pins that the motor's
        encoders are connected to.
        '''
        self.Pin1 = pyb.Pin (pin1, pyb.Pin.IN)
        self.Pin2 = pyb.Pin (pin2, pyb.Pin.IN)
        self.cosmo = pyb.Timer (timer_channel, period = 0xFFFF, prescaler = 0)
        self.cosmo_ch1 = self.cosmo.channel (1, pyb.Timer.ENC_A, pin=self.Pin1)
        self.cosmo_ch2 = self.cosmo.channel (2, pyb.Timer.ENC_B, pin=self.Pin2)
        self.position = 0
        self.position_old = 0

       # self.PinC6 = pyb.Pin (pyb.Pin.board.PC6, pyb.Pin.IN)
       # self.PinC7 = pyb.Pin (pyb.Pin.board.PC7, pyb.Pin.IN)
       # self.wanda = pyb.Timer (8, period = 0xFFFF, prescalar = 0)
       # self.t8ch1 = self.wanda.channel (1, pyb.Timer.ENC_A, pin=self.PinC6)
       # self.t8ch1 = self.wanda.channel (2, pyb.Timer.ENC_B, pin=self.PinC7)
    def not_moving(self):
        if -20 < self.position_old - self.position < 20:
            return True
        
    def read(self):
        temp_2 = self.cosmo.counter() - self.position_old
        self.position_old = self.cosmo.counter()
        if temp_2 > 32767:
            temp_2 -= 65536
        elif temp_2 < -32767:
            temp_2 += 65536
        self.position += temp_2
        return self.position
    
    def show_position(self):
        return self.position
        
    def zero(self):
        self.position = 0
   
  
def main():
    wanda = Encoder(8, pyb.Pin.board.PC6, pyb.Pin.board.PC7)
    go = True
    while go:
        print (wanda.read())
            
        
if __name__ == '__main__':
    main()