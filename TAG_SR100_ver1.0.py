import os, sys
import csv
import random
import signal
import socket
import threading
import time
import datetime

import numpy.matlib
import numpy as np
from numpy.linalg import inv
from numpy.core.fromnumeric import transpose

import math
from math import *

import pyvisa as visa
from colorama import Fore

import serial
import serial.tools.list_ports
import serial.serialutil

import matplotlib.pyplot as plt

# from ucitool.base_uci.helpers.uci_helper import *

# ---------------------------------TEST RUN CONFIGS---------------------------------------------------------------------
TEST_OBJECT_FW = False  # make this false for test object, True for release FW(RC)

Tx1_DEVICE_COM_PORT = 'com11' #initiator1 COM Port

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


def UWB_SR100_TAG():

    reset_delay = 3 # delay setting(sec)
    delay = 1 # delay setting(sec)
    
    ## set baudrate ##
    scpi_tx1 = serial.Serial(Tx1_DEVICE_COM_PORT, baudrate=230400, timeout=6)

    ## Reset all ##
    state_ntf_tx = serial_trx(scpi_tx1, "RST\r\n") 
    print(state_ntf_tx)
    time.sleep(reset_delay)

    state_ntf_tx = serial_rx(scpi_tx1)
    # print(state_ntf_tx)

    ## Session #1 Ranging start ##
    state_ntf_tx = serial_trx(scpi_tx1, "UWB MTINIT1 ON\r\n") # Initiator of Session #1 start Command
    # print(state_ntf_tx) 
    state_ntf_tx = serial_rx(scpi_tx1)
    # print(state_ntf_tx)

if __name__ == "__main__":
    
    UWB_SR100_TAG()
    