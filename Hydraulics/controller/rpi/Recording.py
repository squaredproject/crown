# Crown Recording Module
# Records data coming in from the maquette
# and the conductor to a file
#
# This module lives on the main arms-control pi, and
# works in conjunction with the controller Arduino Mega
# (which monitors the 485 port for communication from the
# conductor) and the maquette system (which connects
# to a listener in self module, and sends maquette position
# information).
#
# Proper multiplexing between maquette and driver is not
# enforced by self module; the various producers of positional
# information should understand whether they are supposed to
# be on or off, and act accordingly.


import shutil
import tempfile
import glob
from threading import Thread
import time

from multiprocessing import Process, Queue
import queue

from pathlib import Path

listener = None
recording_towers = None

# globals - gTowerRange?


class Recorder:
    def __init__(self):
        self._is_recording = False
        self._is_paused = False
        self._recording_file = None
        self._filename = None
        self._recording_path = "~/crown/recordings"
        self._tmp_filepath = Path(self._recording_path, "tmp.playback")
        self._last_frame_time = None

        Path(_recordingPath).mkdir(parents=True, exist_ok=True)

    def start_recording(self, filename=None):
        if self._is_recording:
            print("Recording already started")

        now = time.time()

        if filename is None:
            self._filename = "auto" + now.strftime("%m_%d_%Y:%H_%M_%S") + ".playback"
        else:
            self._filename = filename + ".playback"

        self._recording_file = open(self._tmp_filepath, "w+")
        self._is_recording = True
        self._is_paused = False
        self._last_frame_time = None

    def pause_recording(self):
        if not self._is_recording:
            print(f"Not currently recording, so could not pause")
            return
        self._is_paused = True

    def resume_recording(self):
        if not (self._is_recording and this._is_paused):
            return
        self._is_paused = False

    def abort_recording(self):
        if not self._is_recording:
            return
        close(self._recording_file)
        os.remove(self._tmp_filepath)
        self._is_recording = False

    def close_recording(self):
        if not self._is_recording:
            return
        close(self._recording_file)
        os.rename(self._tmp_filepath, Path(self._recording_path, self._filename))
        self._is_recording = False

    def record_frame(self, frame: String):
        if not self._is_recording:
            return
        if self._is_paused:
            return
        new_reference_time = time.perf_counter()
        if self._last_frame_time == None:
            timedelta_ms = 0
        else:
            timedelta_ms = (new_reference_time - self._last_reference_time) * 1000
        self._last_frame_time = new_reference_time

        self._recording_file.write(f"{frame} + : + {timedelta_ms:3f}\n")


class CrownRecorder:
    def __init__(self):
        self.recording_state = "not recording"
        self.data_queue = Queue()
        self.command_queue = Queue()
        self.process = Process(target=self._run, args=(data_queue, command_queue))
        self.maquettePositionHandler = MaquettePositionHandler(self)
        self.conductorPositionHandler = ConductorPositionHandler(self)
        self.process.start()

    # Recorder processs
    def _run(data_queue, command_queue):
        running = True
        recorder = Recorder()
        while running:
            # read with timeout on data queue
            try:
                recorder.record_frame(data_queue.get(timeout=0.05))
            except queue.Empty:
                pass

            # read non-blocking on command queue
            command = None
            try:
                command = command_queue.get(block=False)
            except queue.Empty:
                pass

            # run command if there is one
            if command != None:
                if command[0] == "shutdown":
                    recorder.close_recording()
                    running = False
                else:
                    _run_command(command)

        def _run_command(command):
            command_func = getattr(recorder, command[0])
            if command_func is not None:
                command_func(recorder, *command[1:])

    # API functions
    def pause_recording(self):
        self.command_queue.put(["pause_recording"])
        self.recording_state = "paused"

    def resume_recording(self):
        self.command_queue.put(["resume_recording"])
        self.recording_state = "recording"

    def start_recording(self, name: String):
        self.command_queue.put(["start_recording", name])
        self.recording_state = "recording"

    def close_recording(self):
        self.command_queue.put(["close_recording"])
        self.recording_state = "not recording"

    def abort_recording(self):
        self.command_queue.put(["abort_recording"])
        self.recording_state = "not recording"

    def record_frame(self, frame):
        self.command_queue.put(["record_frame", frame])

    def shutdown(self):
        self.command_queue.put(["shutdown"])
        self.process.join()

    def recording_state(self):
        return self.recording_state


class FrameState:
    NO_FRAME = "No Frame"
    ACCUMULATING = "Accumulating"


class CrownProtocol:
    FRAME_START = b"00T00000"
    FRAME_STOP = b"00T00001"


class MaquettePositionHandler:
    def __init__(self, recorder: CrownRecorder, port=None):
        self.msg_pipe = Pipe()
        self.maquette_position_gatherer = Process(
            target=self.run, args=(recorder, port, self.msg_pipe)
        )
        self.maquette_position_gatherer.start()

    def run(recorder, port, pipe):
        PACKET_LEN = 8
        listener_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        listener_socker.bind((socket.gethostname(), port))
        listener_socket.listen()
        running = False
        frame = []
        while running:
            readable, writeable, error = select.select(
                [listener_socket, pipe], [], [listener_socket], timeout=0.1
            )
            if pipe in readable:  # Signal shutdown
                running = False
                break

            if listener_socket in readable:  # Connection ready
                (client_socket, address) = listener_socket.accept()
                while True:
                    readable, writeable, error = select.select(
                        [listener_socket, pipe], [], [listener_socket], timeout=0.1
                    )
                    if pipe in readable:
                        running = False
                        break
                    if client_socket in error:
                        break

                    if client_socket in readable:
                        data = clientsocket.recv(self.PACKET_LEN)
                        if data == CrownProtocol.FRAME_START:
                            state = FrameState.ACCUMULATING
                        elif data == CrownProtocol.FRAME_STOP:
                            state = FrameState.NO_FRAME
                            recorder.record_frame(frame)
                            frame = []
                        else:  # an actual frame
                            if state == FrameState.ACCUMULATING:
                                frame.append(data.decode("UTF-8"))

                client_socket.close()

        listener_socket.close()

    def shutdown(self):
        self.msg_pipe.write("shutdown")
        self.maquette_position_gatherer.join()


class ConductorPositionHandler:
    def __init__(self, recorder):
        self._callback_id = CrownSerial.registerListener(self.handle_serial_data, self)
        self.frame_state = FrameState.NO_FRAME
        self._frame = []

    def filter_function(self, data):
        if data == CrownProtocol.FRAME_START:
            self._frame_state = FrameState.ACCUMULATING
        elif data == CrownProtocol.FRAME_STOP:
            self._frame_state = FrameState.NO_FRAME
            return True
        else:
            if self._state == FrameState.ACCUMULATING:
                if (
                    data[3] == "T" or data[3] == "F"
                ):  # we're only looking for certain types of data. XXX really, I should make this all canonical form
                    self._frame.append(data.decode("utf-8"))
        return False

    # Callback happens on the serial thread
    def handle_serial_data(
        data, self
    ):  # XXX - Yes, that's right. The arguments are data and *then* self
        if self.filter_function(data):
            recorder.record_frame(self._frame.join(":"))
            self._frame = []

        return False  # never consume this data

    def shutdown(self):
        CrownSerial.freeListener(self._callback_id)


# So.
# I now have the modules that run on the newpi. I need modules that
# - run on the new mega (485 listener, 485 switch)
# - run on the old mega (send maquette position on serial)
# - run on the old pi (listen for serial and send via ethernet, the reverse
#    of these last couple of functions.
# - I also need to delineate frames... This is a change pretty much everywhere.

# - run on the new pi (send maquette position down to mega)  Missed this one, didn't I?
# -- XXX haven't dealt with the fact that I might not receive all the bytes on the wire...

class MaquettePositionSender:
    def __init__(self):
        self.socket = None
        self._callback_id = CrownSerial.registerListener(self.handle_serial_data, self)
        self.frame_state = FrameState.NO_FRAME
        self._frame = []

    def setup_socket(self):
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.connect(CONTROL_PI_ADDR, CONTROL_PI_MAQUETTE_PORT)
        except OSError, TimeoutError:
            self.socket = None

    def socket_send(self):
        if self.socket == None:
            setup_socket()

        if self.socket == None:
            return

        try:
            self.socket.sendall(frame.join()  # XXX this may be a little harder than that
        except OSError, TimeoutError:
            self.socket.close()
            self.setup_socket()

    def filter_function(self, data):
        if data == CrownProtocol.FRAME_START:
            self._frame_state = FrameState.ACCUMULATING
        elif data == CrownProtocol.FRAME_STOP:
            self._frame_state = FrameState.NO_FRAME
            return True
        else:
            if self._state == FrameState.ACCUMULATING:
                if (
                    data[3] == "T" or data[3] == "F"
                ):  # we're only looking for certain types of data. XXX really, I should make this all canonical form
                    self._frame.append(data.decode("utf-8"))
        return False

    # Callback happens on the serial thread
    def handle_serial_data(
        data, self
    ):  # XXX - Yes, that's right. The arguments are data and *then* self
        if self.filter_function(data):
            recorder.record_frame(self._frame.join(":"))
            self._frame = []

        return False  # never consume this data

    def shutdown(self):
        CrownSerial.freeListener(self._callback_id)

# I will write the python ones first, since I've been writing in Python


# XXX - So I am going to assume that there is a calling module responsible for
# filtering data and packing things up into a frame. I am *not* handling that from
# the recording module

# Maquette recorder is going to take position information coming out of the maquette,
# assemble a frame in the correct format, and call self function. This happens over
# ethernet - which means I can't call self function directly. Okay, so the recorder
# is taking things out of a queue, which I need to be maintaing. Anyone can put things
# into the queue. Which says that it runs in a different thread or process. Probably
# a different process.
# And then there need to be rpc calls to it to do the above functionality. Okay fine,
# now we've got an interface on top of self.

# Conductor recorder is hanging off the serial interface (also in a different process)
# Same requirements
# Both of the gatherers run regardless of whether we're actually recording.
# Ah, and self is why I have a filter on the serialListener. Great.


def startRecording(towers):
    global recordingFile
    global recordingTowers
    if recordingFile:
        recordingFile.close()
    tmpRecordingFile = tempfile.TemporaryFile()
    recordingTowers = towers
    if listener:
        listener.shutdown()
    listener = SerialListener(filterPositions, positionHandler)


def filterPositions(serialString):
    # position string looks.. how? XXX TODO
    if serialString.startswith("<!mT"):
        return True

    return False  # do not block others (Or maybe I do want to block them) XXX


def positionHandler(serialString):
    # parse out position
    try:
        position = json.loads(serialString)
    except ValueError:
        return  # log debug

    for towerId in towers:
        if towerId in gTowerRange:
            towerPos = position[towerId]
    if tmpRecordingFile:
        tmpRecordingFile.write(towerId + "," + towerPos)
        tmpRecordingFile.write("\n")


def stopRecording():
    global listener
    if listener:
        listener.shutdown()
        listener = None


def isRecording():
    if listener:
        return True
    else:
        return False


def nameRecording(filename):
    ret = False

    global tmpRecordingFile
    if not tmpRecordingFile:
        return False

    try:
        with open(recordingPath + filename + ".rec", "w+") as newfile:
            shutil.copyfileobj(tmpRecordingFile, newFile)

    except IOError:
        logging.error("IO Error transferring temporary file to permanent storage")
        ret = False

    close(tmpRecordingfile)
    tmpRecordingFile = None

    return ret


def getClips():
    return getListOfRecordings()  # take off the *.rec!


def getListOfRecordings():
    return glob.glob(recordingPath + "*.rec")


# Create playback from recording clips... XXX todo


playbackThread = None
bPlaying = False
playbackFileName = None
PLAYBACK_DONE = 1
PLAYBACK_TERMINATED = 2
PLAYBACK_ERROR = 3


def playbackStop():
    global bPlaying
    global playbackThread

    if not playbackThread:
        return

    bPlaying = False
    playbackFileName = None
    playbackThread.join()
    playbackThread = False


def playbackStart(name, callback=None, speed=0.1):
    global playbackThread
    global bPlaying
    global playbackFileName

    if playbackThread:
        playbackStop()

    bPlaying = True
    playbackFileName = name
    playbackThread = Thread(target=playback_thread, args=(name, callback, speed))
    playbackThread.start()


def playback_thread(name, callback, speed):
    endState = PLAYBACK_DONE
    try:
        with open(name + ".rec") as playbackFile:
            for line in recording:
                if bPlaying:
                    try:
                        SerialWriter.sendCommand(parsePlayback(line))
                    except:
                        logging.error("Trouble sending playback command")
                    time.sleep(speed)
                else:
                    endState = PLAYBACK_TERMINATED
                    break

    except IOError:
        logger.error("IO Error reading from recorded file" + name)
        endState = PLAYBACK_ERROR

    if callback:
        callback(endState)


STATE_PLAYING = 1
STATE_IDLE = 0


def getPlaybackState():
    if bPlaying:
        state = STATE_PLAYING
    else:
        state = STATE_IDLE

    return {"status": state, "playbackFile": playbackFileName}


def parsePlayback(line):
    """Parse a line of playback and send it to the maquette, to be sent on to the
    sculpture.
    Format of the playback file is : tower, [xx,yy,zz]"""

    parts = line.split(",")
    if len(parts) < 2:
        return
    try:
        towerId = int(parts[0])
        if towerId not in gTowerRange:
            raise ValueError
        locations = json.loads(parts[1])
        if not isinstance(locations, list):
            raise ValueError
        if len(locations) < 3:
            raise ValueError

        return (
            "<"
            + towerId
            + COMMAND_SET_POSITION
            + parts[0]
            + "_"
            + parts[1]
            + "_"
            + parts[2]
            + ">"
        )

    except ValueError:
        logger.error(
            "Error parsing playback, file" + playbackFileName + ", line " + line
        )
        raise

    # XXX parse the line
    # XXX swizzle to serial command format
    # return
    pass  # XXX for now...
