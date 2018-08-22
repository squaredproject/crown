# This is serialRouter.py
import serial
import select
from sys import platform
from threading import Thread, Lock
import logging
import time
import Queue


if platform == "linux" or platform == "linux2":
    tty = '/dev/ttyUSB0'
elif platform == "darwin":
    tty = '/dev/tty.usbserial'
    
ser = None
baudrate = 115200
timeout = 0.1

listeners = {}         # hash of registered listeners 
listenerId = 0         # id to be given to next registered listener 
listenerMutex = Lock() # mutex for hashlist

serialThread = None

running = True

writeQueue = Queue.Queue()


def init():  
    global ser
    ser = serial.Serial(None, baudrate, timeout=timeout)
    ser.nonblocking()
    serialThread = Thread(target=run)
    serialThread.start()
    
def shutdown():
    global running
    running = False
    if (serialThread) :
        serialThread.join(1)

def run():
    serial_open()
    while( running ):
        try: 
            avail_read,avail_write,avail_error=select([ser, writeQueue], [], [], timeout)
            if ser in avail_read:
                if checkForResponse(): 
                    routeResponse()
            
            
            while(writeQueue.qsize() > 0) :  # XXX do this in a select - can look for writes and reads simultaneously
                item = writeQueue.get()
                ser.write(item)
                

        except serial.SerialException:
            time.sleep(1)
            serial_open()
            
def write(bytes):
    writeQueue.put(bytes)
        
def serial_open():
    try: 
        if ser.is_open:
            ser.close()
            time.sleep(1)
        ser.port = tty
        ser.open()            
    except serial.SerialException: # maybe serialUtil.SerialException?
        logging.error("Failed to open serial port at " + str(tty));

# called by listener thread...
def registerListener(callback, args):
    global listenerId
    global listeners
    listenerMutex.acquire()
    callbackId = listenerId
    listenerId = listenerId + 1
    listeners[callbackId] = {'callback': callback, 'args': args}
    listenerMutex.release()
    
    return callbackId
    
def freeListener(callbackId):
    global listeners
    listenerMutex.acquire()
    del listeners[callbackId]
    listenerMutex.release()
    
    
    

# command interface - private functions
MAX_COMMAND_LEN = 256
command_len = 0
command = []


bInCommand = False
COMMAND_START_CHAR = '<'
COMMAND_END_CHAR = '>'  


def checkForResponse():
    global bInCommand
    global command_len
    global command
    command_finished = False
    if command_len >= MAX_COMMAND_LEN:
        command = []
    c = ser.read(1) 
    while(c):
        command_len = command_len + 1
        if ((not bInCommand) and c == COMMAND_START_CHAR):
            bInCommand = True
            command = []
            command.append(c)
        elif (bInCommand and c != COMMAND_END_CHAR):
            command.append(c)
        elif (bInCommand and c == COMMAND_END_CHAR):
            command_finished = True
            bInCommand = False
            
        if not command_finished and command_len >= max_command_len:
            bInCommand = False
        if command_finished:
            break
        else:
            c = ser.read(1)
        
            
    return command_finished
            
            
def routeResponse():
    listenerMutex.acquire()
    bareCommand = command[1:-1] # strip '<' ..'>'
    for key in listeners :
        listener = listeners[key]
        if listener["callback"](listener["args"], bareCommand):  
            break
            
    listenerMutex.release()
    command = []
    
  

            