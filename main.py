""" @file main.py
    This file contains the main program that controls our sumo bot.
    A series of task are defined and executed in order of priority and
    timing requirements.

    author: Darya Darvish

"""

import pyb
import controller_darvish_goodman
import encoder_Roder_darvish_goodman
import motor_darvish_goodman
from micropython import const, alloc_emergencoder_Ry_exception_buf
from task_share import Queue, Share
import ultrasonic
import linetracker
import gc
import utime
import time
import cotask
import task_share
alloc_emergencoder_Ry_exception_buf (100)
  
def interrupt(pokemon):
    '''Interrupt suus_sensorutine that allows the IR sensor to read the
    infrared signal from the remoteeee controller.'''
    if not ir_time_queue.full() and not ir_full_flag.get():
        ir_time_queue.put(pokemon.counter(), in_ISR=True)
    else:
        ir_full_flag.put(1)

def motor_task_1 ():
    ''' Initializes left and right motor.'''
    motor_L = motor_darvish_goodman.MotorDriver(3, pyb.Pin.board.PB4, pyb.Pin.board.PB5, pyb.Pin.board.PA10)
    encoder_L = encoder_Roder_darvish_goodman.Encoder(8, pyb.Pin.board.PC6, pyb.Pin.board.PC7)
    controller_L = controller_darvish_goodman.Controller(0.3, None, utime.ticks_ms())
    
    motor_R = motor_darvish_goodman.MotorDriver(5, pyb.Pin.board.PA0, pyb.Pin.board.PA1, pyb.Pin.board.PC1)
    encoder_R = encoder_Roder_darvish_goodman.Encoder(4, pyb.Pin.board.PB6, pyb.Pin.board.PB7)
    controller_R = controller_darvish_goodman.Controller(0.3, None, utime.ticks_ms())
    
    while True:
        if master_go.get():                
            if retreat.get():
                motor_R.set_duty_cycle(+70)
                motor_L.set_duty_cycle(-70)
                
                if not Line_L.get() and not Line_C.get() and not Line_R.get():
                    ready_left.put(0)
                    ready_right.put(0)
                    time_elapsed.put(time.time())
                    left_pos.put(encoder_L.read()-200)
                    right_pos.put(encoder_R.read()+200)
                    retreat.put(0)
                    retreating.put(1)
                
                elif not Line_L.get() and Line_R.get():
                    ready_left.put(0)
                    ready_right.put(0)
                    time_elapsed.put(time.time())
                    left_pos.put(encoder_L.read()-450)
                    right_pos.put(encoder_R.read()-130)
                    retreat.put(0)
                    retreating.put(1)
                
                elif not Line_R.get() and Line_L.get():
                    ready_left.put(0)
                    ready_right.put(0)
                    time_elapsed.put(time.time())
                    right_pos.put(encoder_R.read()+450)
                    left_pos.put(encoder_L.read()+130)
                    retreat.put(0)
                    retreating.put(1)
                
                else:
                    ready_left.put(0)
                    ready_right.put(0)
                    time_elapsed.put(time.time())
                    left_pos.put(encoder_L.read()-200)
                    right_pos.put(encoder_R.read()+200)
                    retreat.put(0)
                    retreating.put(1)
            
            elif attack.get() and not retreat.get():
                ready_left.put(0)
                ready_right.put(0)
                time_elapsed.put(time.time())
                left_pos.put(encoder_L.read()+125)
                right_pos.put(encoder_R.read()-125)        
            
            controller_L.set_point(left_pos.get())
            controller_R.set_point(right_pos.get())
            posL = encoder_L.read()
            posR = encoder_R.read()
            motor_L.set_duty_cycle(controller_L.go(posL))
            motor_R.set_duty_cycle(controller_R.go(posR))

            # if the wheel is in the desired position, then set the ready flags to true
            if -35 < left_pos.get()-posL < 35:
                ready_left.put(1)
            if -35 < right_pos.get()-posR < 35:
                ready_right.put(1)

        else:
            motor_L.set_duty_cycle(0)
            motor_R.set_duty_cycle(0)
        yield (0)
        
def motion_control ():
    while True:
        if master_go.get():
            if scan.get() and ready_left.get() and ready_right.get():
                if scan_count.get() == 1:
                    yield(0)
                    left_pos.put(left_pos.get() + 125)
                    ready_left.put(0)
                    time_elapsed.put(time.time())
                elif scan_count.get() == 2:
                    yield(0)
                    left_pos.put(left_pos.get() + 100)
                    right_pos.put(right_pos.get() - 100)
                    ready_right.put(0)
                    ready_left.put(0)
                    time_elapsed.put(time.time())
                elif scan_count.get() == 3:
                    yield(0)
                    right_pos.put(right_pos.get() - 125)
                    ready_right.put(0)
                    time_elapsed.put(time.time())
                elif scan_count.get() == 4:
                    yield(0)
                    left_pos.put(left_pos.get() + 100)
                    right_pos.put(right_pos.get() - 100)
                    ready_right.put(0)
                    ready_left.put(0)
                    time_elapsed.put(time.time())
                scan_count.put(scan_count.get()+1)
                if scan_count.get() > 4:
                    scan_count.put(1)
        elif retreating.get() and ready_left.get() or ready_right.get():
                retreating.put(0)
        yield(0)
        
def perception ():
    #initialize US sensor
    us_sensor = ultrasonic.UltraSonic("PA9", "PB10")
    line_sensor = linetracker.LineTracker(pyb.Pin.board.PB8, pyb.Pin.board.PB9, pyb.Pin.board.PB3)

    while True:
        if master_go.get():
            enemy_position.put(us_sensor.distance_cm())
            Line_L.put(line_sensor.read_L())
            Line_C.put(line_sensor.read_C())
            Line_R.put(line_sensor.read_R())
            if retreating.get():
                #Wait for retreat to finish.
                yield(0)
            if Line_L.get() and Line_C.get() and Line_R.get():
                #No line has been sensed.
                if enemy_position.get() > 15:
                    #Checks if enemy is near or far.
                    scan.put(1)
                    attack.put(0)
                else:
                    attack.put(1)
                    scan.put(0)
                    retreat.put(0)
            else:
                #Retreat if line has been sensed.
                scan.put(0)
                retreat.put(1)
        yield(0)

def ir_sensor_task ():        
    while True:
        if ir_full_flag.get():
            # Tell us we have data to analyze
            delta_t = []
            first_val = ir_time_queue.get()
            #compute differnce in time
            while not ir_time_queue.empty():
                second_val = ir_time_queue.get()
                if second_val-first_val < 0:
                    delta_t.append(second_val-first_val + 65536)
                else:    
                    delta_t.append(second_val - first_val)
                first_val = second_val
            if not (8500 < delta_t[0] < 9500) or delta_t[3] > 1000:
                ir_full_flag.put(False)
                continue
            delta_t = delta_t[3:]
            bool_arr = []
            #convert time difference into 1 and 0

            i = 1
            while i < 33:
                if -200 < (delta_t[2*i-1]-delta_t[(2*i)-2]) < 200:
                    bool_arr.append(0)
                else:
                    bool_arr.append(1)
                i += 1
            
            bool_arr.reverse()
            code = ""
            for item in bool_arr:
                code = code + str(item)
            
            #ADDR = code[24:32]
            #if ADDR != "00000000":
                #while not time_queue.empty():
                #    time_queue.get()
            #    full_flag.put(False)
            #    continue
            #nADDR = code[16:24]
            #nCMD = code[0:8]
           
            CMD = code[8:16]

            if int(CMD, 2) == 22:
                master_go.put(0)
            elif int(CMD, 2) == 12:
                master_go.put(1)
            
            ir_full_flag.put(0)

            
            #print("---------------- New Packet ----------------")
            #print("  RAW:  0b" + nCMD + CMD + nADDR + ADDR + "\n")
            #print(" ADDR:  0b" + ADDR)
            #print("nADDR:  0b" + nADDR)
            #print("  CMD:  0b" + CMD)
            #print(" nCMD:  0b" + nCMD + "\n")
            #print("Address (DECIMAL): " + str(int(ADDR, 2)))
            #print("Command (DECIMAL): " + str(int(CMD, 2)))
        yield(0)

# =============================================================================

if __name__ == "__main__":

    print ('\033[2JTesting scheduler in cotask.py\n')
    # set up interrupts and timer for ir sensor
    charmander = pyb.Pin (pyb.Pin.board.PA8, pyb.Pin.IN)
    pokemon = pyb.Timer(1, period = 0xFFFF, prescaler = 79)
    channel_1 = pokemon.channel(1, mode = pyb.Timer.IC, polarity = pyb.Timer.BOTH, 
                              pin=charmander)
    channel_1.callback(interrupt)
    ir_time_queue = Queue("I", 68)
    ir_full_flag = Share("i")
    ir_full_flag.put(0, in_ISR = True)
    
    #setup master controller share
    master_go = Share("i")
    master_go.put(0, in_ISR = False)
    
    #Robot States
    attack = task_share.Share ('i', thread_protect = False, name = "attack")
    attack.put(0)
    
    scan = task_share.Share ('i', thread_protect = False, name = "scan")
    scan.put(0)
    
    retreat = task_share.Share ('i', thread_protect = False, name = "retreat")
    retreat.put(0)
    
    retreating = task_share.Share ('i', thread_protect = False, name = "retreat")
    retreating.put(0)
                               
    #setup motion control shares
    ready_left = task_share.Share ('i', thread_protect = False, name = "right_flag")
    ready_left.put(1)
    
    ready_right = task_share.Share ('i', thread_protect = False, name = "right_flag")
    ready_right.put(1)
    
    right_pos = task_share.Share ('i', thread_protect = False, name = "right_pos")
    right_pos.put(0)
    
    left_pos = task_share.Share ('i', thread_protect = False, name = "left_pos")
    left_pos.put(0)
    
    scan_count = task_share.Share ("f", thread_protect = False, name = "scan_count")
    scan_count.put(1)

    #setup enemy distance Share
    enemy_position = task_share.Share ('f', thread_protect = False, name = "enemy_pos")
    enemy_position.put(0)
    
    time_elapsed = task_share.Share ('f', thread_protect = False, name = "time_elapsed")
    time_elapsed.put(0)    
    
    #setup perception shares
    Line_L = task_share.Share ('i', thread_protect = False, name = "L")
    Line_L.put(1)
    Line_C = task_share.Share ('i', thread_protect = False, name = "C")
    Line_C.put(1)
    Line_R = task_share.Share ('i', thread_protect = False, name = "R")
    Line_R.put(1)
    
    #setup task share and tasks
    
    q0 = task_share.Queue ('B', 6, thread_protect = False, overwrite = False,
                           name = "Queue_0")
    
    task1 = cotask.Task (motor_task_1, name = 'right_motor', priority = 1, 
                         period = 20, profile = True, trace = False)
    
    task2 = cotask.Task (motion_control, name = 'motion', priority = 2,
                        period = 21, profile = True, trace = False)
    
    task3 = cotask.Task (ir_sensor_task, name = 'ir_sensor', priority = 2,
                        period = 25, profile = True, trace = False)
    
    task4 = cotask.Task (perception, name = 'perception', priority = 1,
                        period = 20, profile = True, trace = False)

    cotask.task_list.append (task1)
    cotask.task_list.append (task2)
    cotask.task_list.append (task3)
    cotask.task_list.append (task4)

    gc.collect ()

    # Run the scheduler with the chosen scheduling algorithm. Quit if any 
    # character is sent through the serial port
    vcp = pyb.USB_VCP ()
    while not vcp.any ():
        cotask.task_list.pri_sched ()

    # Empty the comm port buffer of the character(s) just pressed
    vcp.read ()
