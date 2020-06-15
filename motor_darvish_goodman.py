""" @file motor_darvish_goodman.py
    This file contains the main program that creates a motor 
    driver for the sumo-bot.

    author: Darya Darvish

"""



import pyb

class MotorDriver:
    """This class implements a motor driver for the
    ME405 board."""
    def __init__ (self, timer_channel, pin_1, pin_2, enable_pin):
        ''' Creates a motor driver by initializing GPIO
        pins and turning the motor off for safety. '''
        #print('Creating a motor driver')
        self.motor_enable = pyb.Pin (enable_pin, pyb.Pin.OUT_PP)
        self.motor_enable.high()
        self.in1A = pyb.Pin (pin_1, pyb.Pin.OUT_PP)
        self.in1A.low ()
        self.in2A = pyb.Pin (pin_2, pyb.Pin.OUT_PP)
        self.in2A.low ()
        self.tim = pyb.Timer (timer_channel, freq=20000)
        self.ch2 = self.tim.channel (2, pyb.Timer.PWM, pin=self.in2A)
        self.ch1 = self.tim.channel (1, pyb.Timer.PWM, pin=self.in1A)

        
    def set_duty_cycle (self, level):
        ''' This method sets the duty cycle to be sent
        to the motor to the given level. Positive values
        cause torque in one direction, negative values
        in the opposite direction.
        @param level A signed integer holding the duty
        cycle of the voltage sent to the motor '''
        #print ('Setting duty cycle to ' + str (level))
    
        if level < 0:
            self.ch1.pulse_width_percent(0)
            self.ch2.pulse_width_percent(abs(level))
        elif level >= 0:
            self.ch2.pulse_width_percent(0)
            self.ch1.pulse_width_percent(level)
