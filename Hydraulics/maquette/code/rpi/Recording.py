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


class Recorder:
    def __init__(self):
        self._is_recording = False
        self._is_paused = False
        self._recording_file = None
        self._filename = None
        self._recording_path = "~/crown/recordings"
        self._tmp_filepath = Path(self._recording_path, "tmp.playback")
        self._last_frame_time = None

        Path(self._recording_path).mkdir(parents=True, exist_ok=True)

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

    def record_frame(self, frame: str):
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

# For recording I need to be able to get the state of recording
# from the recording process - if we're not recording, I don't want
# to fork off all the data
# Or do I have the recording state at a higher level? On the main thread?
# I need to think about that a little bit

class CrownRecorder:
    def __init__(self, data_queue):
        self.recording_state = "not recording"
        self.data_queue = data_queue
        self.command_queue = Queue()
        self.process = Process(target=self._run, args=(data_queue, self.command_queue,))
        # self.maquettePositionHandler = MaquettePositionReceiver(self)
        # self.conductorPositionHandler = ConductorPositionHandler(self)
        self.process.start()

    # Recorder processs
    def _run(self, data_queue, command_queue):
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

    def start_recording(self, name: str):
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
