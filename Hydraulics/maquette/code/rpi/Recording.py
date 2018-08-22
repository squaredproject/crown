# recording module
import shutil
import tempfile
import glob
from threading import Thread
import time

listener = None
tmpRecordingFile = None
recordingPath = "/usr/lib/crown/"
recordingTowers = None

# globals - gTowerRange?

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
        
    return False # do not block others (Or maybe I do want to block them) XXX
    
def positionHandler(serialString):
    # parse out position
    try:  
        position = json.loads(serialString)
    except ValueError:
        return # log debug
        
    for towerId in towers:
        if towerId in gTowerRange:
            towerPos = position[towerId]
    if (tmpRecordingFile):
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
        with open(recordingPath + filename + '.rec', 'w+') as newfile:
            shutil.copyfileobj(recordingFile, newFile)
          
    except IOError:
        logging.error("IO Error transferring temporary file to permanent storage")
        ret =  False
        
    close(tmpRecordingfile)
    tmpRecordingFile = None
    
    return ret
    
def getClips:
    return getListOfRecordings() # take off the *.rec!
    
def getListOfRecordings():
    return glob.glob(recordingPath + '*.rec')
    

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
    
    if (callback):
        callback(endState)

STATE_PLAYING = 1
STATE_IDLE    = 0   
     
def getPlaybackState():
    if bPlaying:
        state = STATE_PLAYING
    else:
        state = STATE_IDLE
    
    return {"status": state, "playbackFile": playbackFileName}
    
    
        
def parsePlayback(line):
    """ Parse a line of playback and send it to the maquette, to be sent on to the
        sculpture.
        Format of the playback file is : tower, [xx,yy,zz] """
        
    parts = line.split(",")
    if (len(parts) < 2) 
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
            
        return ("<" + towerId + COMMAND_SET_POSITION + parts[0] + "_" + parts[1] + "_" + parts[2] + ">")
        
        
    except ValueError:
        logger.error("Error parsing playback, file" + playbackFileName + ", line " + line)
        raise
        
    # XXX parse the line
    # XXX swizzle to serial command format
    # return
    pass # XXX for now...
    
    