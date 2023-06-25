import serial
import select
from sys import platform
from threading import Thread, Lock
import logging
import time
from multiprocessing import Queue
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

writeQueue = Queue()


def init():
    global ser
    ser = serial.Serial(None, baudrate, timeout=timeout)
    ser.nonblocking()
    serialThread = Thread(target=run)
    serialThread.start()


def shutdown():
    global running
    running = False
    print("SHUTDOWN called, setting running to FALSE")
    if serialThread:
        serialThread.join(1)


def run():
    global running
    serial_open()
    while running:
        try:
            # avail_read,avail_write,avail_error=select.select([ser, writeQueue], [], [], timeout)
            avail_read, avail_write, avail_error = select.select([ser], [], [], timeout)
            if ser in avail_read:
                resp = checkForResponse()
                if resp:
                    logging.debug(f"SERIAL - have response {resp}")
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
    logging.debug(f"Register listener - args are {args}")
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
command = []
in_command = False
COMMAND_START_CHAR = b"<"
COMMAND_END_CHAR = b">"


def checkForResponse():
    global command
    global in_command
    command_finished = False
    c = ser.read(1)
    while c:
        logging.debug(f"Serial char {c}")
        if (not in_command) and c == COMMAND_START_CHAR:
            in_command = True
            command = []
            command.append(c)
        elif in_command:
            command.append(c)
            if c == COMMAND_END_CHAR:
                command_finished = True
                in_command = False

        if not command_finished and len(command) >= MAX_COMMAND_LEN:
            in_command = False
            logging.error(f"Serial: Exceeded maximum size of command {command}, resetting")
            command = []
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
    try: 
        bareCommand = response[1:-1]  # strip '<' ..'>'
        for key in listeners:
            listener = listeners[key]
            if listener["bare_response"]:
                logging.debug("Calling callback with bare command")
                logging.debug(f"Callback is {listener['callback']}")
                logging.debug(f"Args are {listener['args']}")
                consume = listener["callback"](*listener["args"], bareCommand)
            else:
                logging.debug("Calling callback with full response")
                consume = listener["callback"](*listener["args"], response) 
            if consume:
                break
    except Exception as e:
        logging.error(f"Serial: Exception in callback {e}")
    finally:
        listenerMutex.release()
