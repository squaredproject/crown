# main crown 
import logging
from sys import platform
import time
from BaseHTTPServer import HTTPServer, BaseHTTPRequestHandler
from SocketServer import ThreadingMixIn
import threading
import urlparse
import traceback
import cgi
import json
import CrownSerial
import CrownSerialMock

test = True
if test:
    serial = CrownSerialMock
else:
    serial = CrownSerial

# XXX - note that you can select on Queues. select.select() works just fine...
# change the code to use select!!

class AsyncRequester:
    """ Handles aggregating multiple requests. (Since we're using a CAN bus under the
    covers, the data packets received are very small. We deal with this by making a 
    lot of requests for small amounts of data) """
    
    def __init__(self, calldata, timeout=5):
        self.calldata = calldata
        self.timeout = timeout
        self.dataReady = False
                          
        self.listener = serial.registerListener(AsyncRequester._asyncCallback, self)
        # self.SerialListener(self.asyncFilter, self.asyncCallback, self)
        
    def run(self):
        """ Make all requests, wait for responses. Timeout if responses have not been 
        received by the specified timeout """
        print("RUN asynch requester!\n");
        try:
            endTime = time.time() + self.timeout
            while (not self.dataReady and time.time() < endTime):
                self._request()
                print("requesting...\n")
                time.sleep(0.15)
        except Exception as e:
            print("Exception!\n")
            print e
            traceback.print_exc()
        
        serial.freeListener(self.listener)
        
        if self.dataReady:
            print ("Have data, sending to callback\n")
            return True
        else:
            print ("No data returned, aborting\n")
            return False
            
        
        
    def _request(self):
        """ Re-request anything that we haven't gotten a response for """
        for call in self.calldata:
            if not call['response']:
                call["call"](call['callArgs'])
                
        
    def _asyncCallback(myself, message): # callback... can't call through class so I fake it...
        try: 
            print("Asynccallback called!")
            myself.dataReady = True
            for call in myself.calldata:
                print("checking on message " + message)
                print("check against")
                print(call)
                if not call['response']:
                    filteredResponse = call['filter'](message, call['filterArgs'])
                    if filteredResponse:
                        print("async callback accepts " + message)
                        call['response'] = filteredResponse[:] # copying 
                    else:
                        myself.dataReady = False  
            # if there are no responses left, trigger wakeup of main thread.   # XXX - use select 
        except Exception as e:
            myself.dataReady = False
            logging.warn("Exception on async serial callback")
            traceback.print_exc()
                  
        return False   
        
              
gTowerRange = [1,2,3,4]

# Nomenclature - 'get' is a compound call. 
# 'Request' a low level call - only a single call on the CAN bus

def getSculptureState(towers): 
    """Get the state of one or more towers
       Tower state is defined as the current joint position, the joint limits for each joint
        and the general state. It does not include PID values or current drive
        Returns list of 
        {"towerId":towerId, "joints":[{"id":xx, "position":xx, 
                                       "center":xx, "rightLimit":xx, leftLimit":xx
                                       "homed":true|false, "enabled":true|false} ...],
                            "running":true|false,
                            "error":true:false}"""   
        
    calls = []
    for towerId in towers: # XXX check what happens if towers is an integer TODO
        if towerId not in gTowerRange:
            continue
            
        calls.append({'call'   : requestTowerPosition, 'callArgs' : [towerId], 'response' : None,
                      'filter' : towerPositionRequestFilter, 'filterArgs' : [towerId]})        
        calls.append({'call'   : requestJointLimits, 'callArgs' : [towerId, 0], 'response' : None, 
                      'filter' : jointLimitsRequestFilter, 'filterArgs' : [towerId, 0]})
        calls.append({'call'   : requestJointLimits, 'callArgs' : [towerId, 1], 'response' : None, 
                      'filter' : jointLimitsRequestFilter, 'filterArgs' : [towerId, 1]})
        calls.append({'call'   : requestJointLimits, 'callArgs' : [towerId, 2], 'response' : None, 
                      'filter' : jointLimitsRequestFilter, 'filterArgs' : [towerId, 2]})
        calls.append({'call'   : requestTowerGeneralStatus, 'callArgs' : [towerId], 'response' : None, 
                      'filter' : generalStatusRequestFilter, 'filterArgs' : [towerId]})
                      
              
    requester = AsyncRequester(calls)
    results = requester.run()
    if (results):
        resultList = {}
        for towerId in towers: # set up template
            if towerId not in gTowerRange:
                continue
            jointList = []
            for jointId in range(0,3):
                jointList.append({'id':jointId, 'position':[],
                                  'max': 0, 'min': 0, 'center':0, 
                                  'enabled' : True, 
                                  'homed'   : True})
            towerObj = {'tower': towerId, 'joints':jointList, 'running':True, 'error':False}
            print("adding towerId {} to resultList".format(towerId))
            resultList[towerId]=towerObj
        
        for call in calls: 
            # the format of the individual responses is json string data 
            resultObj = json.loads(call['response'])  # XXX handle misformatted json
            towerId = call['callArgs'][0]
            print("towerIdx is {}".format(towerId))
            print("resultObj is {}".format(resultObj))
            print("resultList is {}".format(resultList))
            print("sizeof resultList is {}".format(len(resultList)))
            if call['call'] == requestTowerPosition:
                print("resultlist towerId is {}".format(resultList[towerId]))
                print("joints 0 is {}".format(resultList[towerId]['joints'][0]))
                resultList[towerId]['joints'][0]['position'] = resultObj[0]
                resultList[towerId]['joints'][1]['position'] = resultObj[1]
                resultList[towerId]['joints'][2]['position'] = resultObj[2]
            if call['call'] == requestTowerGeneralStatus:
                resultList[towerId]['running'] = resultObj['running']
                resultList[towerId]['error']   = resultObj['error']
                resultList[towerId]['joints'][0]['enabled'] = resultObj['enabled'][0]
                resultList[towerId]['joints'][0]['homed']   = resultObj['homed'][0]
                resultList[towerId]['joints'][1]['enabled'] = resultObj['enabled'][1]
                resultList[towerId]['joints'][1]['homed']   = resultObj['homed'][1]
                resultList[towerId]['joints'][2]['enabled'] = resultObj['enabled'][2]
                resultList[towerId]['joints'][2]['homed']   = resultObj['homed'][2]
            if call['call'] == requestJointLimits:
                jointIdx = call['callArgs'][1] # joints 0 based. because we hate life XXX I don't have to propagate that
                resultList[towerId]['joints'][jointIdx]['center']  = resultObj['center']
                resultList[towerId]['joints'][jointIdx]['min']     = resultObj['min']
                resultList[towerId]['joints'][jointIdx]['max']     = resultObj['max']
                
        return (200,json.dumps(resultList))
    else:
        return 501
#
#def getTowerState(towerId):     # subsumed by getSculptureState      
#    calls = [{'call'   : requestTowerGeneralStatus, 'callArgs' : [towerId], 'response' : None, 
#               'filter' : requestGeneralStatusFilter, 'filterArgs' : [towerId]}]    
#                
#     requester = AsyncRequester(calls)
#     results = requester.run()
#     if (results):
#         return(200, json.dumps(results[4:]))
#     else:
#         return(501, None)

def getTowerPosition(towerId):
    """Get the current position of the specified tower
       Returns [xx,xx,xx] """
    calls = [{'call'   : requestTowerPosition, 'callArgs' : [towerId], 'response' : None, 
              'filter' : towerPositionRequestFilter, 'filterArgs' : [towerId]}]    
    
    requester = AsyncRequester(calls)
    results = requester.run()
    if (results):
        return(200, calls[0]["response"])
    else:
        return(500)
        
def getTowerLimits(towerId):
    """ Get min, max, and center positions for all joints.
        Returns [{"min": xxx, "max":xxx, "center":xxx}] """
    calls = []
    for jointId in range(1,4):
        calls.append({'call'   : requestJointLimits, 'callArgs' : [towerId, jointId], 'response' : None, 
                      'filter' : jointLimitsRequestFilter, 'filterArgs' : [towerId, 1]})
    requester = AsyncRequester(calls)
    results = requester.run()
    if (results):
        limits = []
        for call in calls:
            limits.append(json.loads(call["response"]))
        return(200, json.dumps(limits))
                
    else:
        return(500)
        
        
def getJointLimits(towerId, jointId):
    """ Get min, max, and center positions for all joints.
        Returns [{"min": xxx, "max":xxx, "center":xxx}] """
    calls = []
    calls.append({'call'   : requestJointLimits, 'callArgs' : [towerId, jointId], 'response' : None, 
                  'filter' : jointLimitsRequestFilter, 'filterArgs' : [towerId, jointId]})
    requester = AsyncRequester(calls)
    results = requester.run()
    if (results):
        limits = []
        for call in calls:
            limits.append(json.loads(call["response"]))
        print("limits: ", json.dumps(limits));
        return(200, json.dumps(limits))
                
    else:
        return(500)
        
def getDriveValues(towerId):
    """ Get current drive values for all joints
        Returns [xx,xx,xx] """
    calls = [{'call'   : requestValveState, 'callArgs' : [towerId], 'response' : None, 
              'filter' : valveDriveRequestFilter, 'filterArgs' : [towerId]}]    
    
    requester = AsyncRequester(calls)
    results = requester.run()
    if (results):
        return(200, calls[0]["response"])
    else:
        return(500)  
              
def getPIDValues(towerId):
    """ Get current PI values for all joints
        Returns {"p":[xx,xx,xx],"i":[xx,xx,xx]}"""
        
    calls = [{'call'   : requestPIDState, 'callArgs' : [towerId], 'response' : None, 
              'filter' : PIDValuesRequestFilter, 'filterArgs' : [towerId]}]    
    
    requester = AsyncRequester(calls)
    results = requester.run()
    if (results):
        return(200, calls[0]["response"])
    else:
        return(500) 
                       
def getMaquetteStatus():
    """Get the status of the maquette. Includes mode, position, limits"""
    calls = [{'call'   : requestMaquetteStatus, 'callArgs' : None, 'response' : None,
              'filter' : maquetteStatusFilter, 'filterArgs' : None}]
              
    requester = AsyncRequester(calls)
    results = requester.run()
    if (results):
        return(200, json.dumps(results[4:]))
    else:
        return(500)
        
def getMaquetteCalibration():
    calls = [{'call'   : requestMaquetteCalibration, 'callArgs' : None, 'response' : None,
              'filter' : maquetteCalibrationFilter, 'filterArgs' : None}]
              
    requester = AsyncRequester(calls)
    results = requester.run()
    if (results):
        return(200, json.dumps(results[4:]))
    else:
        return(500)
    

   

GENERAL_STATUS_REQUEST = 's'
JOINT_STATUS_REQUEST   = 'j'
JOINT_LIMITS_REQUEST   = 'l'
HOME_COMMAND           = 'H'
TOWER_POSITION_REQUEST = 'T'
VALVE_DRIVE_REQUEST    = 'v'
SET_HOME_SPEED_COMMAND = 'h'
SET_TARGETS_COMMAND    = 't'
SET_LIMITS_COMMAND     = 'L'
SET_MIN_COMMAND        = 'm'
SET_MAX_COMMAND        = 'M'
SET_CENTER_COMMAND     = 'C'
SET_I_VALUE_COMMAND    = 'I'
SET_P_VALUE_COMMAND    = 'P'
PID_VALUES_REQUEST     = 'p'
NEUTER_COMMAND         = 'n'
EXTENDED_STATUS_REQUEST = 'e'
MAQUETTE_CALIBRATION_REQUEST = 'c'
SET_RUN_STATE_COMMAND  = 'r'
SET_HOME_STATE_COMMAND = 'w'
SET_MAQUETTE_MODE_COMMAND = 'M'
SET_MAQUETTE_TOWER_CENTER_COMMAND = 'C'

def validateResponseHeader(message, towerId, command):
    if message[0] != '!':
        return False
        
    if message[1] != command:
        return False
        
    try: 
        if towerId != None and int(message[2]) != towerId:
            return False
    except ValueError:
        return False
        
    return True
   
def generalStatusRequestFilter(message, args):
    ret = validateResponseHeader(message, args[0], GENERAL_STATUS_REQUEST)
    if (ret):
        return message[3:]
    else:
        return None
        
    
def jointStatusRequestFilter(message, args):
    ret = validateResponseHeader(message, args[0], JOINT_STATUS_REQUEST)
    try: 
        if (ret and (args[1] == int(message[3]))): # checking third position in header against joint id
            return message[4:]
    except ValueError:
        pass
        
    return None
        
def jointLimitsRequestFilter(message, args):
    ret = validateResponseHeader(message, args[0], JOINT_LIMITS_REQUEST)
    try: 
        if (ret and (args[1] == int(message[3]))): # checking third position in header against joint id
            return message[4:]
    except ValueError:
        pass
        
    return None
    
def towerPositionRequestFilter(message, args):
    ret = validateResponseHeader(message, args[0], TOWER_POSITION_REQUEST)
    if (ret):
        return message[3:]
    else:
        return None
    
def valveDriveRequestFilter(message, args):
    ret = validateResponseHeader(message, args[0], VALVE_DRIVE_REQUEST)
    if (ret):
        return message[3:]
    else:
        return None
        
def PIDValuesRequestFilter(message, args):
    ret = validateResponseHeader(message, args[0], PID_VALUES_REQUEST)
    if (ret):
        return message[3:]
    else:
        return None

def maquetteCalibrationFilter(message, args):
    ret = validateResponseHeader(message, args[0], MAQUETTE_CALIBRATION_REQUEST)
    if (ret):
        return message[3:]
    else:
        return None
        
# things one can request...

def requestTowerGeneralStatus(args):
    if (isinstance(args, list)):
        towerId = args[0]
    else:
        towerId = args
        
    serial.write("<" + str(towerId) + GENERAL_STATUS_REQUEST + ">")

def requestJointStatus(args):
    if (isinstance(args, list)):
        towerId = args[0]
        jointId = args[1]
    else:
        return # log error !! todo
        
    serial.write("<" + str(towerId) + GENERAL_STATUS_REQUEST + ">")

def requestJointLimits(args, jointId = None):
    if (isinstance(args, list)):
        towerId = args[0]
        jointId = args[1]
    else:
        return # log error !! todo

    serial.write("<" + str(towerId) + str(jointId) + JOINT_LIMITS_REQUEST + ">")
    
def requestTowerPosition(args):
    if (isinstance(args, list)):
        towerId = args[0]
    else:
        towerId = args
        
    serial.write("<" + str(towerId) + TOWER_POSITION_REQUEST + ">") 

def requestValveState(args):
    if (isinstance(args, list)):
        towerId = args[0]
    else:
        towerId = args     
    serial.write("<" + str(towerId) + VALVE_DRIVE_REQUEST + ">")
    
def requestPIDState(args):
    if (isinstance(args, list)):
        towerId = args[0]
    else:
        towerId = args   
    serial.write("<" + str(towerId) + PID_VALUES_REQUEST + ">")
    
def requestExtendedStatus(args):
    if (isinstance(args, list)):
        towerId = args[0]
    else:
        towerId = args   
    serial.write("<" + str(towerId) + EXTENDED_STATUS_REQUEST + ">")
    
def requestMaquetteCalibration(args):
    if (isinstance(args, list)):
        towerId = args[0]
    else:
        towerId = args
        
    serial.write("<" + str(towerId) + MAQUETTE_CALIBRATION_REQUEST + ">")
    
## Will I do an ack on these commands? No. These are 485 commands

def commandHome(towerId, jointId):
    serial.write("<" + str(towerId) + str(jointId) + HOME_COMMAND + ">")
    
def commandSetTargets(towerId, j1, j2, j3):
    serial.write("<" + str(towerId) + SET_TARGETS_COMMAND + str(j1) +  ">")
    serial.write("<" + str(towerId) + SET_TARGETS_COMMAND + str(j2) +  ">")
    serial.write("<" + str(towerId) + SET_TARGETS_COMMAND + str(j3) +  ">")

    
def commandSetHomeSpeed(towerId, jointId, homeSpeed):
    serial.write("<" + str(towerId) + str(jointId) + SET_HOME_SPEED_COMMAND + str(homespeed))
    
def commandNeuterValve(towerId, jointId):
    serial.write("<" + str(towerId) + str(jointId) + NEUTER_COMMAND + ">")
    
def commandSetRunState(towerId, onOff):
    if onOff:
        onOffVal = "1"
    else:
        onOffVal = "0"
    serial.write("<" + str(towerId) + SET_RUN_STATE_COMMAND + onOffVal + ">") 
    
def commandSetCenters(towerId):
    serial.write("<" + str(towerId) + str(1) + SET_CENTER_COMMAND + ">")
    serial.write("<" + str(towerId) + str(2) + SET_CENTER_COMMAND + ">")
    serial.write("<" + str(towerId) + str(3) + SET_CENTER_COMMAND + ">")

    
def commandSetLimits(towerId, jointId, min, max):
    serial.write("<" + str(towerId) + str(jointId) + SET_MIN_COMMAND + str(min) + ">")
    serial.write("<" + str(towerId) + str(jointId) + SET_MAX_COMMAND + str(max) + ">")

def commandSetTowerHomed(towerId):
    print("setting homed")
    serial.write("<" + str(towerId) + str(1) + SET_HOME_STATE_COMMAND + "1>")
    serial.write("<" + str(towerId) + str(2) + SET_HOME_STATE_COMMAND + "1>")
    serial.write("<" + str(towerId) + str(3) + SET_HOME_STATE_COMMAND + "1>")
    print("end homed")

def commandSetMaquetteMode(mode):
    if mode == "pose":
        modeInt = "2"
    elif mode == "off":
        modeInt = "0"
    elif mode == "immediate":
        modeInt = "1"
    else:
        raise Exception
    serial.write("<m" + SET_MAQUETTE_MODE_COMMAND + modeInt + ">")    
    
def commandCalibrateMaquette(towerId):
    serial.write("<m" + SET_MAQUETTE_TOWER_CENTER_COMMAND + towerId + ">")    
  

    


class Handler(BaseHTTPRequestHandler):
    def sendResponse(self, resp, mimetype="application/json"):
#         print("attempting to send response");
#         self.send_response(resp[0])
#         self.send_header('Content-type', mimetype)
#         self.send_header('Access-Control-Allow-Origin', '*')
#         self.end_headers()
#         self.wfile.write(resp[1])
#         return
        
        if (  isinstance(resp, list) 
           or isinstance(resp, tuple)):
            print("sending list or tuple")
            print("code is {}".format(resp[0]))
            self.send_response(resp[0])
            self.send_header('Content-type', mimetype)
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            if resp[1]:
                self.wfile.write(resp[1])
                
        else:
            print("sending single response!")
            self.send_response(resp)
            self.end_headers()  
            
    def sendError(self, errcode):
        self.end_headers()
        self.send_response(errcode)
             
                
    def getFile(self, path, mimeType):
        try:
            print ("attempting to open", path)
            f=open("." + path, "rb")
            self.send_response(200)
            self.send_header('Content-type', mimeType)
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            self.wfile.write(f.read())
            f.close()
        except Exception:
            self.sendError(404)

        
    def do_GET(self):
        parsed_path = urlparse.urlparse(self.path)
        paths = parsed_path.path[1:].split('/')
        print paths
        path_len = len(paths)
        
        if (paths[0] == ""):
            self.getFile("/control_panel.html", 'text/html');
    
        
            
        elif paths[0] == "crown":
            if paths[1] == "sculpture":
                if len(paths) < 3:
                    # /crown/sculpture. Get sculpture state
                    # option for extended state? ?extended=true
                    self.sendResponse(getSculptureState(range(1,4))) 
                elif paths[2] == "towers": 
                    # /crown/sculpture/towers/[id]
                    if len(paths) < 4:
                        self.send_response(404)
                        return
                    try:
                        towerNum = int(paths[3])
                    except ValueError:
                        self.send_response(404)
                        return
                        
                    if towerNum not in gTowerRange:
                        self.send_response(404)
                        return
                      
                    if len(paths) < 5:
                        # get tower state
                        self.sendResponse(getSculptureState([towerNum]))
                    elif paths[4] == "position":
                        # get tower position
                        self.sendResponse(getTowerPosition(towerNum))
                    elif paths[4] == "limits":
                        # get joint limits
                        self.sendResponse(getTowerLimits(towerNum)) 
                    elif paths[4] == "homingStatus":
                        self.sendResponse([200, HomingStatusHandler.getHomingStatus(towerNum)])
                    elif paths[4] == "pid":
                        self.sendResponse(getPIDValues(towerNum))
                    elif paths[4] == "valves":
                        self.sendResponse(getDriveValues(towerNum))
                        
                    elif len(paths) == 7:
                        jointNum = int(paths[5])
                        if paths[6] == "limits":
                            self.sendResponse(getJointLimits(towerNum, jointNum));
                            return;

                    else:
                        self.send_response(404)
                        return                                                   
                else:
                    self.send_response(404)
                    return                    
            elif paths[1] == "savedParameters": # get saved pid and limits
                pass
            elif paths[1] == "maquette": # get maquette status
                if (len(paths) == 2):
                    # get maquette status
                    self.sendResponse(getMaquetteStatus())
                elif len(paths) == 3:
                    if paths[2] == "calibration":
                        self.sendResponse(getMaquetteCalibration())
            elif paths[1] == "playback": # get playback status (playing, not playing, which playlist)
                self.sendResponse(getPlaybackState())
            elif paths[1] == "record": # get recording state (recording, not recording, which tower to record)
                self.sendResponse(getRecordingState())
            elif paths[1] == "playlists": # get list of playlists
                self.sendResponse(getPlaylists())
            elif paths[1] == "clips": # get list of clips
                self.sendResponse(getClips())
        elif paths[0] == "admin":
            self.getFile("/control_panel.html", 'text/html');
        elif paths[0] == "img" or paths[0] == "js" or paths[0] == "style":
            print("GET path is {}\n".format(self.path))
           
            if self.path.endswith(".png"):
                self.getFile(self.path, 'image/png')
            elif self.path.endswith(".gif"):
                self.getFile(self.path,'image/gif')
            elif self.path.endswith(".jpeg"):
                self.getFile(self.path, 'image/jpeg')
            elif self.path.endswith(".html"):
                self.getFile(self.path, 'text/html')
            elif self.path.endswith(".js"):
                self.getFile(self.path, 'application/javascript')
            elif self.path.endswith(".css"):
                self.getFile(self.path,'text/css')
            else:
                self.sendResponse(404)
        else: # for now, just 404
            self.sendResponse(404)
            

    def getPlaybackState():
        return [200, getPlaybackStatue()]
                      
    def getRecordingState():
        return [200, {"recordingState":Recording.getStatus(), "towers":Recording.getTowers()}]
        
    def getPlaylists():
        return [200, Playback.getPlaylists()]
        
    def getClips():
        return [200, Playback.getClips()]
        
    def do_OPTIONS(self):
        print "OPTIONS"
        self.send_response(200, "ok")
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Access-Control-Allow-Methods', 'GET, POST, PUT, OPTIONS')
        self.send_header("Access-Control-Allow-Headers", "X-Requested-With")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")


        
    def do_PUT(self):
        ctype, pdict = cgi.parse_header(self.headers.getheader('content-type'))
        if ctype == 'multipart/form-data':
            postvars = cgi.parse_multipart(self.rfile, pdict)
        elif ctype == 'application/x-www-form-urlencoded':
            length = int(self.headers.getheader('content-length'))
            postvars = cgi.parse_qs(self.rfile.read(length), keep_blank_values=1)
        else:
            postvars = {}
            
        parsed_path = urlparse.urlparse(self.path)
        paths = parsed_path.path.split('/')
        if len(paths) < 2:
            self.send_response(404)
            return 
            
        if paths[0] == "crown":
            if paths[1] == "sculpture":
                if len(paths) < 5 or paths[2] != "towers":
                    self.send_response(404)
                    return     
                try:                            
                    towerNum = int(paths[3])
                except ValueError:
                    self.send_response(404)
                    return
                                     
                if not towerNum in gTowerRange:
                    self.send_response(404)
                    return
                    
                if paths[5] == "targets": # set  target position
                    self.sendResponse(commandSetTargets(towerId, postvars["joint1"], postvars["joint2"], postvars["joint3"]))
                    return
                elif paths[5] == "joints":
                    if len(paths) < 6:
                        self.send_response(404)
                        return
                    self.sendResponse(commandSetLimits(towerId, jointId, postvars["min"], postvars["max"], postvars["center"]))
                    return
                elif paths[5] == "pid":
                    self.sendResponse(setTowerPID([[postvars["joint1_P"], postvars["joint1_1"]],
                                              [postvars["joint2_P"], postvars["joint2_1"]],
                                              [postvars["joint3_P"], postvars["joint3_1"]]]))
                elif paths[5] == "debug":
                    pass
            elif paths[1] == "savedParameters": # set saved limits and pid values
                pass
            elif paths[1] == "maquette": # set maquette limits and mode
                pass
            elif paths[1] == "playback": #get playback status (playing, not playing, which playlist)
                pass
            elif paths[1] == "record": # get recording state (recording, not recording, which tower to record)
                pass
            elif paths[1] == "playlists": # modify playlists
                pass
            
        return
        
        # XXX TODO - decide if the towers are indexed based on 1 or 0
        # I seem to be leaning on 1 for the ids at least
        # yeah, the API uses a 1's based index.
            
    def do_POST(self): # homing, playback, record, 
    
        parsed_path = urlparse.urlparse(self.path)
        paths = parsed_path.path.split('/')
        if len(paths) < 2:
            self.send_response(404)
            return
# 
#         ctype, pdict = cgi.parse_header(self.headers.getheader('content-type'))
#         if ctype == 'multipart/form-data':
#             postvars = cgi.parse_multipart(self.rfile, pdict)
#         elif ctype == 'application/x-www-form-urlencoded':
#             length = int(self.headers.getheader('content-length'))
#             postvars = cgi.parse_qs(self.rfile.read(length), keep_blank_values=1)
#         else:
#             postvars = {}
#             
        print paths
        paths = paths[1:]
        if paths[0] == "crown":
            if paths[1] == "sculpture":
                if (   (len(paths) >= 7) 
                   and (paths[2] == "towers")
                   and (paths[4] == "joints")):
                    try:
                        towerId = int(paths[3])
                        jointId = int(paths[5])
                        if (towerId not in gTowerRange) or (jointId not in range(1,4)):
                            raise ValueError 
                    except ValueError:
                        self.send_response(404)
                        return

                    if (paths[6] == "home"):
                        self.sendResponse(commandHome(towerId, jointId))
                        return
                    elif (paths[6] == "limits"):
                        commandSetLimits(towerId, jointId, -250, 300);
                        self.send_response(200);
                        return
                elif (len(paths) == 5) :
                    try: 
                        print("paths 3 is ", paths[3])
                        print("paths 4 is ", paths[4])
                        towerId = int(paths[3])
                        if (paths[4] == "center"):
                            commandSetCenters(towerId)
                            self.send_response(200);
                        elif (paths[4] == "running"):
                            try:
                                print("running?")
                                commandSetTowerHomed(towerId)
                                commandSetRunState(towerId, True)
                                print("sending 200")
                                self.send_response(200);
                                return
                            except Exception as e:
                                traceback.print_exc()
                                raise e

                        else:
                            self.send_response(404)
                              
                    except Exception:
                        self.send_response(404)
                        return;
                    
                else:
                    self.send_response(404)
                    return  
            elif paths[1] == "maquette":
                if paths[2] == "mode":
                    commandSetMaquetteMode(paths[3])
                    self.end_response(200)
                elif paths[2] == "towers":
                    if paths[4] == "calibrate":
                        commandCalibrateMaquette(int(paths[3]))
                    
        else:
            self.send_response(404)
            return  
                              
        return
    def do_DELETE(self): # playlists, recordings
        return
        
class HomingStatusHandler():
    homingStatus = []
    
    def init():
        for i in range(0,4):
            homingStatus.append(None)
        serial.registerListener(HomingStatusHandler.homingCallback, None)  # XXX - note that this adds a dependency on initing crown serial before creating this object!
            
    def setHomingStatus(homingMessage):
        towerId = homingMessage[2]
        try:
            homingObject = json.loads(homingMessage[4:])
            HomwingStatusHandlr.homingStatus[towerId] = homingObject
        except ValueError:
            logging.error("malformed json in homing message")
            
    def getHomingStatus(towerId):
        if (towerId not in gTowerRange):
            return None
        return HomingStatusHandler.homingStatus[towerId]
        
    def homingCallback(message, args):
        if (validateRequestHeader(message, None, HOME_COMMAND)) :
            HomingStatusHandler.setHomingStatus(message)
            return True
        else:
            return False



class ThreadedHTTPServer(ThreadingMixIn, HTTPServer):
    """Handle requests in a separate thread."""

    

if platform == "linux" or platform == "linux2":
    logfile = '/var/log/crown/crown.log'
elif platform == "darwin":
    logfile = 'crown.log'

if __name__ == "__main__":
    logging.basicConfig(filename=logfile,level=logging.DEBUG)
    
    serial.init()
    
    homingHandler = HomingStatusHandler()
    
    time.sleep(1) # why am I doing this?
    
    server = ThreadedHTTPServer(('localhost', 5050), Handler)
    print 'Starting server, use <Ctrl-C> to stop'
    try:
        server.serve_forever()
    except KeyboardInterrupt:   
        pass
        
    serial.shutdown()
    
    