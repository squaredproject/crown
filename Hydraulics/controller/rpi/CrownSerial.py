import serial
import select
from sys import platform
from threading import Thread, Lock
import logging
import time
import multiprocessing
import multiprocessing.queues
import socket
import os
# import queue

if platform == "linux" or platform == "linux2":
    tty = "/dev/ttyACM0"
elif platform == "darwin":
    tty = "/dev/tty.usbserial"

ser = None
baudrate = 115200
timeout = 0.01

listeners = {}  # hash of registered listeners
listenerId = 0  # id to be given to next registered listener
listenerMutex = None  # mutex for hashlist

serialThread = None

running = True

# PollableQueue class thanks to O'Reilly
class PollableQueue(multiprocessing.queues.Queue):
    def __init__(self):
        print("POLLABLE QUEUE CTOR")
        super().__init__(ctx=multiprocessing.get_context())
        # Create a pair of connected sockets
        if os.name == 'posix':
            self._putsocket, self._getsocket = socket.socketpair()
        else:
            # Compatibility on non-POSIX systems
            server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            server.bind(('127.0.0.1', 0))
            server.listen(1)
            self._putsocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self._putsocket.connect(server.getsockname())
            self._getsocket, _ = server.accept()
            server.close()
        print(f"putsocket is {self._putsocket}, getsocket is {self._getsocket}")

    def fileno(self):
        return self._getsocket.fileno()

    def put(self, item):
        super().put(item)
        self._putsocket.send(b'x')

    def get(self):
        self._getsocket.recv(1)
        return super().get()


writeQueue = None

def init():
    print("SER INIT")
    global ser
    global writeQueue
    global listenerMutex
    global running
    ser = serial.Serial(None, baudrate, timeout=timeout)
    ser.nonblocking()
    writeQueue = multiprocessing.Queue()
    # print(f"writequeue put socket is {writeQueue._putsocket}")
    print(f"writequeue is {writeQueue}")
    listenerMutex = Lock()
    running = True
    serialThread = Thread(target=run, args=(writeQueue, listenerMutex, running))
    serialThread.start()


def shutdown():
    global running
    running = False
    print("SHUTDOWN called, setting running to FALSE")
    if serialThread:
        serialThread.join(1)


def run(writeQueue, listenerMutex, running):
    logger = logging.getLogger("Serial")
    handler = logging.handlers.RotatingFileHandler(filename = "/var/log/crown/serial.log", maxBytes=100000)
    formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
    handler.setFormatter(formatter)
    logger.addHandler(handler)
    logger.setLevel(logging.INFO)
    
    serial_open()
    while running:
        try:
            # avail_read,avail_write,avail_error=select.select([ser, writeQueue], [], [], timeout)
            avail_read, avail_write, avail_error = select.select([ser], [], [], timeout)
            if ser in avail_read:
                resp = checkForResponse(logger)
                if resp:
                    logger.debug(f"Have response {resp}")
                    routeResponse(resp, logger)
            # if writeQueue in avail_read:
            while (
                writeQueue.qsize() > 0
            ):
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
    logging.getLogger("Crown").debug(f"Register listener - args are {args}")
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


def checkForResponse(logger):
    global command
    global in_command
    command_finished = False
    c = ser.read(1)
    while c:
        logger.debug(f"Serial char {c}")
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
            logger.error(f"Exceeded maximum size of command {command}, resetting")
            command = []
        if command_finished:
            break
        else:
            c = ser.read(1)

    if command_finished:
        retVal = None
        try: 
            retVal = b"".join(command).decode("utf-8")
        except:
            logger.error(f"Could not decode {command}")
        return retVal
    else:
        return None


def routeResponse(response, logger):
    listenerMutex.acquire()
    try: 
        bareCommand = response[1:-1]  # strip '<' ..'>'
        for key in listeners:
            listener = listeners[key]
            if listener["bare_response"]:
                logger.debug("Calling callback with bare command")
                logger.debug(f"Callback is {listener['callback']}")
                logger.debug(f"Args are {listener['args']}")
                consume = listener["callback"](*listener["args"], bareCommand)
            else:
                logger.debug("Calling callback with full response")
                consume = listener["callback"](*listener["args"], response) 
            if consume:
                break
    except Exception as e:
        logger.error(f"Serial: Exception in callback {e}")
    finally:
        listenerMutex.release()
