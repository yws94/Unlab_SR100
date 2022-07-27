import csv
import os, sys
import random
import signal
import socket
import threading
import time
import math

import numpy.matlib
from numpy.linalg import inv
from math import *
from numpy.core.fromnumeric import transpose

import numpy as np
import pyvisa as visa
from colorama import Fore

import serial
import serial.tools.list_ports
import serial.serialutil
import datetime

# from ucitool.base_uci.helpers.uci_helper import *

# ---------------------------------TEST RUN CONFIGS---------------------------------------------------------------------
TEST_OBJECT_FW = False  # make this false for test object, True for release FW(RC)

Rx_DEVICE_COM_PORT = 'com10' #responder COM Port
Tx1_DEVICE_COM_PORT = 'com11' #initiator1 COM Port
Tx2_DEVICE_COM_PORT = 'com32' #initiator2 COM Port
Tx3_DEVICE_COM_PORT = 'com34' #initiator3 COM Port 
Tx4_DEVICE_COM_PORT = 'com35' #initiator4 COM Port

# ----------------------------------------------------------------------------------------------------------------------
# --------------------------------------NO EDITS BELOW------------------------------------------------------------------
# ----------------------------------------------------------------------------------------------------------------------

def serial_tx(my_port, command):
    b_command = command.encode()
    my_port.write(b_command)

def serial_rx(my_port):
    line = my_port.read_until()
    line = line.strip()
    return line.decode("utf-8")

def serial_trx(my_port, command):
    serial_tx(my_port, command)
    return (serial_rx(my_port))

def save_csv(csvf, row):
    with open(csvf, "a", newline="") as F:
        w = csv.writer(F)
        w.writerow(row)

class UWB_SR100_Ranging_test():
    def __init__(self):
        
        self.reset_delay = 3 # delay setting(sec)
        self.ranging_stop_delay = 1 # delay setting(Sec)
        self.delay = 1 # delay setting(sec)
        
        ## set baudrate ##
        self.scpi_rx = serial.Serial(Rx_DEVICE_COM_PORT, baudrate=230400, timeout=6) 
        self.scpi_tx1 = serial.Serial(Tx1_DEVICE_COM_PORT, baudrate=230400, timeout=6)
        
        ##State Vectors 
        self.X = np.array([[0.0001],[0.0001]])
        ##State Trasition Matrixs
        self.F = np.eye(2)
        ##Error Covariance Matrix
        self.errorCov = 100
        self.P = np.eye(2) * self.errorCov

        #Process Noise Covariance About 
        self.Q = np.diag([0.01,0.01]) # Q = De

        #Measurement Noise
        self.R = np.diag([0.1, 0.083]) # R = D
        
    def ekf_update(self, Measurement):
            #1. Time Update("Predict")
        #1.1 Project the state ahead
        self.pXk = self.F @ self.X
        
        #1.2 Project the error covariance ahead
        self.pPk = (self.F @ self.P @ np.transpose(self.F)) + self.Q 

        #sub. Cala Hk Matrix
        self.X2Y2 = pow(self.pXk[0,0],2) + pow(self.pXk[1,0],2)
        self.dRxk = self.pXk[0,0] / sqrt(self.X2Y2)
        self.dRyk = self.pXk[1,0] / sqrt(self.X2Y2)
        if self.pXk[0,0] > 0 :
            self.dPDxk = (-self.pXk[1,0])/(self.X2Y2)
            self.dPDyk = (self.pXk[0,0])/(self.X2Y2)
        elif self.pXk[0,0] < 0:
            self.dPDxk = (self.pXk[1,0])/(self.X2Y2)
            self.dPDyk = (-self.pXk[0,0])/(self.X2Y2)
        self.Hk = np.array([[self.dRxk, self.dRyk],[self.dPDxk, self.dPDyk]])
            
        #2. Measurement Update("Correct")
        #2.1 Compute the Kalman gain Kk
        self.Ksk = self.Hk @ self.pPk @ np.transpose(self.Hk) + self.R
        self.Kk = self.pPk @ np.transpose(self.Hk) @ inv(self.Ksk)
        #sub. Calc Sk(Yk - Y(k-1))
        self.YkHR = sqrt((self.pXk[0]**2) + (self.pXk[1]**2))
        self.YkHPD = math.atan(self.pXk[1,0]/self.pXk[0,0])
        if(self.YkHPD<0):
            self.YkHPD += math.pi
        print(self.YkHPD)
        self.Sk = Measurement - np.array([[self.YkHR],[self.YkHPD]])

        #2.2 Update estimate with measurement
        self.X = self.pXk + np.dot(self.Kk,self.Sk)
        #2.3 Update the error covariance
        self.P = self.pPk - (self.Kk @ self.Hk @ self.pPk)
            
    def Ranging(self):
        ## Reset all ##
        state_ntf_tx = serial_trx(self.scpi_tx1, "RST\r\n") # 'reset' command
        print(state_ntf_tx)
        state_ntf_rx = serial_trx(self.scpi_rx, "RST\r\n")
        print(state_ntf_rx)
        time.sleep(self.reset_delay)

        state_ntf_tx = serial_rx(self.scpi_tx1)
        # print(state_ntf_tx)
        state_ntf_tx = serial_rx(self.scpi_rx)
        # print(state_ntf_tx)
        
        state_ntf_rx = serial_trx(self.scpi_rx, "UWB ANTPAIR 1\r\n") # Set PDOA offset to responder
        # print(state_ntf_rx)
        state_ntf_rx = serial_trx(self.scpi_rx, "UWB PDOAOFFSET -60\r\n") # Input offset value
        # print(state_ntf_rx)


        ## Session #1 Ranging start ##
        state_ntf_tx = serial_trx(self.scpi_tx1, "UWB MTINIT1 ON\r\n") # Initiator of Session #1 start Command
        # print(state_ntf_tx) 
        state_ntf_rx = serial_trx(self.scpi_rx, "UWB MTRESP1 ON\r\n") # Responder of Session #1 start Command
        # print(state_ntf_rx)
        state_ntf_tx = serial_rx(self.scpi_tx1)
        # print(state_ntf_tx)
        state_ntf_tx = serial_rx(self.scpi_rx)
        # print(state_ntf_tx)
        time.sleep(self.delay)

        self.scpi_tx1.close()
        self.scpi_rx.close()
        
        while 1: # Number of result Data
            self.scpi_rx = serial.Serial(Rx_DEVICE_COM_PORT, baudrate=230400, timeout=6)
            self.scpi_ret = serial_rx(self.scpi_rx)
            # print(Fore.GREEN,scpi_ret,Fore.RESET)
            try:
                ## Data Parsing ##
                result = self.scpi_ret.split(',')
                session_id = result[0]
                distance = result[1]
                aoa_azimuth = result[2]
                aoa_elevation = result[3]
                pdoa_azimuth = result[4]
                pdoa_elevation = result[5].replace("\r\n", '')
            except:
                pass
            
            ## convert types for dist and angle to float
            dist = float(distance)/100
            angle = math.pi * (float(aoa_azimuth)+90)/180
            
            ## calculate position of TAGs
            x = dist * math.cos(angle)
            y = dist * math.sin(angle)
            x_ref = str(x)
            y_ref = str(y)
            
            s_dist = str(dist)

            meas = np.array([[dist],[angle]])
            self.ekf_update(meas)

            x_pos = self.X[0,0]
            y_pos = self.X[1,0]
            
            print(Fore.GREEN, x_pos, y_pos, x_ref, y_ref, s_dist, aoa_azimuth, Fore.RESET)
            # print(Fore.GREEN, x_ref, y_ref, scpi_ret,Fore.RESET)
            
            ## save data(.csv file) ##
            save_csv(ranging_result_csvF, [session_id, s_dist, x_pos, y_pos, x_ref, y_ref,aoa_azimuth, aoa_elevation, pdoa_azimuth, pdoa_elevation])
            # save_csv(ranging_result_csvF, [session_id, distance, x_ref, y_ref, aoa_azimuth, aoa_elevation, pdoa_azimuth, pdoa_elevation])
            
            self.scpi_rx.close()
        
        self.scpi_tx1 = serial.Serial(Tx1_DEVICE_COM_PORT, baudrate=230400, timeout=6)
        self.scpi_rx = serial.Serial(Rx_DEVICE_COM_PORT, baudrate=230400, timeout=6)

        state_ntf_rx = serial_trx(self.scpi_rx, "UWB RNG STOP\r\n") #Reset Command
        print(state_ntf_rx)
        state_ntf_tx = serial_trx(self.scpi_tx1, "UWB RNG STOP\r\n") #Reset Command
        print(state_ntf_tx)
        time.sleep(self.ranging_stop_delay)
        state_ntf_tx = serial_rx(self.scpi_tx1)
        print(state_ntf_tx)
        state_ntf_tx = serial_rx(self.scpi_rx)
        print(state_ntf_tx)

        self.scpi_rx.close()
        self.scpi_tx1.close()


if __name__ == "__main__":
    
    now = datetime.datetime.now()
    nowDatetime = now.strftime('%Y_%m_%d_%H_%M_%S')
    
    ranging_result_csvF = 'results/UWB_SR100_ranging_test_result-%s.csv' %nowDatetime
    save_csv(ranging_result_csvF, ['Session_ID','Distance','pos_X','pos_Y','ref_X','ref_Y','AoA_azimuth','AoA_elevation','PDoA_azimuth','PDoA_elevation'])
    
    ekf = UWB_SR100_Ranging_test()
    ekf.Ranging()
    