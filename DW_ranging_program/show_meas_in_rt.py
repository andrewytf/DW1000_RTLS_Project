# -*- coding: utf-8 -*-
"""
Created on Fri Nov 18 10:07:20 2016

@author: jostp
"""

import serial
import re
import numpy as np
import matplotlib.pyplot as plt
import time

NUM_OF_MEASUREMENTS = 200   # number of values to record
MAX_DISTANCE = 1.5            # max expected distance in m

dist_vec = np.zeros((NUM_OF_MEASUREMENTS,1)) # measured distances
toc_vec = np.zeros((NUM_OF_MEASUREMENTS,1))  # time of DWT communication (one distance measurement)

ser = serial.Serial()
ser.baudrate = 115200
ser.port = 'COM6'

if not ser.is_open:
    ser.open()

meas_cnt = 0

plt.figure(1)
plt.cla()
plt.ylim((0, MAX_DISTANCE))
plt.xlim((0, NUM_OF_MEASUREMENTS))
plt.title('Prikaz meritev')
plt.xlabel('Indeks meritve')
plt.ylabel('Razdalja [m]')
plt.grid();

while True:
    
    ser.reset_input_buffer() # zato da vzamemo samo zadnjo vrstico (zadnji podatek)
    
    while not ser.in_waiting > 0: # wait for new line
        pass        
        
    start_time = time.time()
    s = ser.readline() # read line of data from serial connection
    #print(s)
    
    # process line
    if 'DIST:' in str(s):
        result_dist = re.search('DIST: (.*) m,', str(s))
        result_toc = re.search('TOC: (.*) s', str(s))
        dist_vec[meas_cnt] = float(result_dist.group(1))
        toc_vec[meas_cnt] = float(result_toc.group(1))
        
        print('Computed distance: %.2f' % dist_vec[meas_cnt-1])
        meas_cnt += 1
    
    # break out of loop, if all measurements acquired
    if meas_cnt >= NUM_OF_MEASUREMENTS:
        break
    
    # draw latest measurement
    plt.scatter(meas_cnt, dist_vec[meas_cnt-1])
    plt.pause(0.05)
    
    
    print('Time of iteration: %.5f' % (time.time() - start_time))
    

    