#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
 This script creates a pseudo serial ports using pseudoterminals
 It can be used to run sensor data from GPS or Vectornav
Authors: Jagatpreet Singh, Kris Dorsey, Arun Anbu
"""

import os, pty
from serial import Serial
import threading
import time 
import argparse

class SerialEmulator:
    
    def __init__(self,file,sample_time, loop_type):
        self.sample_time = sample_time  
        self.file = file 
        self.driver = None
        self.driven = None
        self.loop_type = loop_type
        
    def write_file_to_pt(self):
        f = open(self.file, 'r') 
        Lines = f.readlines()
        for line in Lines:
            write_string = line.rstrip() + '\r\n'
            write_bytes = str.encode(write_string, encoding='utf-8')
            os.write(self.driver, write_bytes)
            time.sleep(self.sample_time)
        f.close()
        print("Sensor emulator has reached the end of the file")
    
    def emulate_device(self):
        """Start the emulator"""
        self.driver,self.driven = pty.openpty() #open the pseudoterminal
        print("The Pseudo device address: %s"%os.ttyname(self.driven))
        try:
            self.write_file_to_pt()
            while self.loop_type == 'yes':
                print("Restarting...\n")
                self.write_file_to_pt()
            self.stop_simulator()
                
        except KeyboardInterrupt:
            self.stop_simulator()
            pass
    
    def start_emulator(self):
        self.emulate_device()

    def stop_simulator(self):
        os.close(self.driver)
        os.close(self.driven)
        print("Terminated")
        
    
if __name__=='__main__':
    parser = argparse.ArgumentParser(description='Command line options for Serial emulator.\
                                     Press Ctrl-C to stop execution')
    parser.add_argument('-f','--file', required=True, type=str, dest='file',
                    help='data file to simulate device')   

    parser.add_argument('-V','--VN_string', default = b'$VNWRG,07,200*XX', type=str, dest='VN_string',
                    help='Write register string to pass to VN')

    parser.add_argument('-dev','--device_type', default = 'gps', type=str, dest='device_type',
                    help="Device type should be 'gps' or 'imu' ")
    
    parser.add_argument('-l','--loop', default = 'yes', type=str, dest='loop_behavior',
                    help="This should be 'yes' for looping ")
    
    parser.add_argument('-r','--rate', default = 1, type=float, dest='sample_rate',
                    help="This should be a float at the desired rate ")
    
    
    args = parser.parse_args()
    sample_time = 1/float(args.sample_rate)
    if (args.device_type == 'gps') or args.device_type == 'imu':
        print("Starting", args.device_type, "emulator with sample rate:", str(args.sample_rate), "Hz")
    elif (args.device_type == 'imu'):
        VN_string = args.VN_string.decode('utf-8')
        VN_list = VN_string.split(",")
        if (VN_list[0] == "$VNWRG" and VN_list[1] == "07"):
            sample_rate = VN_list[2].split('*')
            sample_rate = sample_rate[0] 
            sample_time = 1/float(sample_rate)
        else:
            print("This is not the correct string to change the sample rate.")
        print("Starting", args.device_type, "emulator with sample rate:", str(args.sample_rate), "Hz")

    else: 
        print("Device type string must be 'gps' or 'imu'. Setting sample time to default 1 second")
    
    if sample_time > 0: 
        se = SerialEmulator(args.file, sample_time, args.loop_behavior)
        se.start_emulator()    
