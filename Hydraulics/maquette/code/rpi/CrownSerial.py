# This is serialRouter.py
import serial
import select
from sys import platform
from threading import Thread, Lock
import logging
import time
import queue


if platform == "linux" or platform == "linux2":
    tty = "/dev/ttyACM0"
elif platform == "darwin":
    tty = "/dev/tty.usbserial"

ser = None
baudrate = 115200
timeout = 0.1

listeners = {}  # hash of registered listeners
listenerId = 0  # id to be given to next registered listener
listenerMutex = Lock()  # mutex for hashlist

serialThread = None

running = True

writeQueue = queue.Queue()


def init():
    global ser
    ser = serial.Serial(None, baudrate, timeout=timeout)
    ser.nonblocking()
    serialThread = Thread(target=run)
    serialThread.start()


def shutdown():
    global running
    running = False
    if serialThread:
        serialThread.join(1)


def run():
    serial_open()
    while running:
        try:
            # avail_read,avail_write,avail_error=select.select([ser, writeQueue], [], [], timeout)
            avail_read, avail_write, avail_error = select.select([ser], [], [], timeout)
            if ser in avail_read:
                resp = checkForResponse()
                if resp:
                    routeResponse(resp)

            while (
                writeQueue.qsize() > 0
            ):  # XXX do this in a select - can look for writes and reads simultaneously
                item = writeQueue.get()
                ser.write(bytes(item, "utf-8"))

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
    except serial.SerialException:  # maybe serialUtil.SerialException?
        print("Failed to open serial port at " + str(tty))


# called by listener thread...
def registerListener(callback, args, bare_response=True):
    global listenerId
    global listeners
    listenerMutex.acquire()
    callbackId = listenerId
    listenerId = listenerId + 1
    listeners[callbackId] = {"callback": callback, "args": args, "bare_response" : bare_response}
    listenerMutex.release()

    return callbackId


def freeListener(callbackId):
    global listeners
    listenerMutex.acquire()
    del listeners[callbackId]
    listenerMutex.release()


# command interface - private functions
MAX_COMMAND_LEN = 2000
command_len = 0
command = []


bInCommand = False
COMMAND_START_CHAR = b"<"
COMMAND_END_CHAR = b">"


def checkForResponse():
    global bInCommand
    global command_len
    global command
    command_finished = False
    if command_len >= MAX_COMMAND_LEN:
        command = []
    c = ser.read(1)
    while c:
        command_len = command_len + 1
        if (not bInCommand) and c == COMMAND_START_CHAR:
            bInCommand = True
            command = []
            command.append(c)
        elif bInCommand:
            command.append(c)
            if c == COMMAND_END_CHAR:
                command_finished = True
                command_len = 0
                bInCommand = False

        if not command_finished and command_len >= MAX_COMMAND_LEN:
            bInCommand = False
            command_len = 0
            print("Exceeded maximum size of command, resetting")
        if command_finished:
            break
        else:
            c = ser.read(1)

    if command_finished:
        return b"".join(command).decode("utf-8")
    else:
        return None


def routeResponse(response):
    listenerMutex.acquire()
    bareCommand = response[1:-1]  # strip '<' ..'>'
    for key in listeners:
        listener = listeners[key]
        if listener["bare_response"]:
            consume = listener["callback"](*listener["args"], bareCommand)
        else:
            consume = listener["callback"](*listener["args"], response) 
        if consume:
            break

    listenerMutex.release()
