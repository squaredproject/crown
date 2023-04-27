# Test version of CrownSerial...

import logging
import random 
from threading import Thread, Lock
import time
import queue


ser = None
baudrate = 115200
timeout = 0.1

listeners = {}         # hash of registered listeners 
listenerId = 0         # id to be given to next registered listener 
listenerMutex = Lock() # mutex for hashlist

serialThread = None
responderThread = None

running = True

writeQueue = queue.Queue()  # things to be written to the 'serial' port, sent to us
readQueue  = queue.Queue()  # things that the 'serial' port reads
responseBuffer = [] # 'serial' port responses

def init():  
    global serialThread
    global responderThread
    serialThread = Thread(target=run)
    serialThread.start()
    responderThread = Thread(target=respond)
    responderThread.start()
    
def shutdown():
    global running
    running = False
    if (serialThread) :
        serialThread.join(1)
    if responderThread :
        responderThread.join(1)

def run():
    while( running ):
        time.sleep(0.1)
        while(writeQueue.qsize() > 0) : 
            print ("Got an item off the write queue!")
            item = writeQueue.get()
            print(f"Adding item {item} to read queue!")
            readQueue.put(item)
            
        time.sleep(0.02) # XXX I should be using select and a select timeout...

            

def respond():
    while( running ):
        time.sleep(0.1)
        while(readQueue.qsize() > 0) : 
            print ("responder gets item!!")
            item = readQueue.get()
            processRequest(item)
            
        curTime = time.time()
        markedForDeletion = []
        for response in responseBuffer :
            if (curTime > response["time"]):
                routeResponse(response["response"]) # XXX - ideally, the response should be handled by the intermediary, not the response generator, but <shrug> This is test code
                markedForDeletion.append(response)
        for deleteme in markedForDeletion:
            responseBuffer.remove(deleteme)
    


def write(bytes):
    writeQueue.put(bytes)
       
def processRequest(item):
    if (len(item) < 4 ):
        return
        
    print("processing " + item)
    responseString = None
        
    try:
        towerId = item[1] 
        try:
            jointId = int(item[2])
            commandId = item[3]
        except ValueError:
            jointId = None
            commandId = item[2]
            
        print ("command is " + commandId)
        if commandId == 's': # tower status
            responseString = "<!s" + towerId + "{\"homed\":[true, true, false], \"switches\": [4, 0, 1], \"enabled\": [true, true, true], \"running\": true,\"error\": 0}>"
        elif commandId == 'j': # joint status
            responseString = "<!j" + towerId + str(jointId) + "{\"jointId\":" + str(jointId) + ", \"pos\":300, \"target\": 250, \"valve\":78, \"homed\":true, \"enabled\":true, \"switches\":0x02}>"
        elif commandId == 'l': # joint limits
            responseString = "<!l" + towerId + str(jointId) + "{\"min\":-250,\"max\":300, \"center\": 4027}>"
        elif commandId == 'T': #position
            responseString = "<!T" + towerId + "[13,-34,200]>"
        elif commandId == 'v': # valves
            responseString = "<!v" + towerId + "[13,63,93]>"
        elif commandId == 'p': #pid values
            responseString = "<!p" + towerId + "{\"p\": [100,80,120], \"i\": [2, 4, 5.6]}>"
        else:   
            responseString = None
        
        if ( responseString ) :
            print("adding response...")
            curTime = time.time()
            expTime = curTime + random.randint(1,100)/1000
            print("expiry time " + str(expTime) + ", current time " + str(curTime))
            responseBuffer.append({"time": expTime, "response":responseString})
    except Exception as e:
        print("Exception in serial responder test code")
        print(e)
    

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
    print(listeners)
    del listeners[callbackId]
    listenerMutex.release()

            
def routeResponse(response):
    print("Attempting to route response " + response)
    listenerMutex.acquire()
    for key in listeners :
        listener = listeners[key]
        if listener["callback"](listener["args"], response[1:-1]):  # strip '<'..'>' 
            break
            
    listenerMutex.release()
    
  
    
 
