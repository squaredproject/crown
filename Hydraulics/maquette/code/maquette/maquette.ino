#include <mcp_can_dfs.h>
//#include <mcp_can.h>

#include <SPI.h>
#include "UART1.h"
#include "UARTbaudrates.h"

#include "CAN_message.h"
#include "mcp_can_wrapper.h"


// Basic Architecture:
// This embedded processor handles low level communication
// to the sculpture towers, and to the maquette. It is controlled by a 
// higher level processor (in this particular case, a Raspberry PI) that 
// it communicates with over serial. The higher level processor sets
// state, requests human-visible information, and is responsible for saving
// and resetting any calibration values
//
// One of the oddities in the architecture is that we communicate to the 
// towers both over RS485 and CAN. This is because the tower controllers were
// originally built using RS485, but when we brought Crown out to BurningMan in
// 2018, we wanted to to query for status as well as send commands. However, RS485
// is not a particularly friendly for two way communication so we added CAN
// for status and left the original RS485 code alone. This also allowed us
// to fall back to an older control model if/when the maquette controls weren't
// ready.
//
// The sculpture is designed to be operated in one of two states - Pose mode or 
// Slave mode. In Pose mode, hitting the button in the center of the maquette 
// causes the sculpture to take the position of the maquette. In Slave mode, the
// position of the sculpture is controlled by position information coming from
// the raspberry pi.
// 
// There is an additional (possibly dangerous) mode called Free Play, where the
// sculpture attempts to immediately reflect any changes to the maquette.
// 
// Please note that as of this writing (2/6/23) there is no provision made to
// keep the towers from hitting each other, assuming their ranges allow them to
// do so.

// CSW, 2/6/23
 

#define NUM_TOWERS 4
#define NUM_JOINTS 3
#define MIN_POT 384
#define MAX_POT 640
#define POT_RANGE 1024
#define SCULPTURE_RANGE 8192

#ifndef TRUE 
#define TRUE 1
#endif 

#ifndef FALSE
#define FALSE 0
#endif

#define MAIN_LOOP_TIMEOUT_MILLIS 20
#define TF_STRING(x) ((x) ? "true" : "false")
#define SETTLE_TIME 2

//const int SPI_CS_PIN = 53;


typedef enum {
   MODE_OFF = 0,
   MODE_FREE_PLAY = 1,
   MODE_POSE = 2,
   MODE_SLAVE = 3,
} Mode;
Mode mode = MODE_OFF;

int towerValid[NUM_TOWERS] = {TRUE, TRUE, TRUE, TRUE};
int jointValid[NUM_TOWERS][NUM_JOINTS] = {{TRUE, TRUE, TRUE},
                                          {TRUE, TRUE, TRUE},
                                          {FALSE, FALSE, FALSE},
                                          {TRUE, TRUE, TRUE}};

// XXX - this is the joint range for the MAQUETTE, not the tower.
// XXX so where is the tower center position?
uint16_t jointRange[NUM_TOWERS][NUM_JOINTS][2] = {{{-300, 200}, {-532, 328}, {-275, 276}},
                                                  {{-300, 520}, {-532, 288}, {-275, 296}},
                                                  {{-200, 200}, {-200, 200}, {-200, 200}},
                                                  {{-300, 520}, {-532, 288}, {-275, 296}}};
//-300, -532, -275
// 520, 288, 296

/*uint16_t jointRange[NUM_TOWERS][NUM_JOINTS][2] = {{{-200, 200}, {-200, 200}, {-200, 200}},
                                                  {{-200, 200}, {-200, 200}, {-200, 200}},
                                                  {{-200, 200}, {-200, 200}, {-200, 200}},
                                                  {{-200, 200}, {-200, 200}, {-200, 200}}};
*/

// Joint center is the raw value at the center of the joint
// XXX - note again that this is the range for the MAQUETTE not CROWN
uint16_t jointCenter[NUM_TOWERS][NUM_JOINTS] = {{POT_RANGE/2,POT_RANGE/2,POT_RANGE/2}, {504, 571, 500}, {POT_RANGE/2,POT_RANGE/2,POT_RANGE/2}, {504, 571, 500 }};
uint8_t centered[NUM_TOWERS] = {FALSE, FALSE, FALSE, FALSE};
float canonicalJointPosition[NUM_TOWERS][NUM_JOINTS] = {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}};

int modelPosition[NUM_TOWERS][NUM_JOINTS]       = {{0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}};
// XXX - below - when we're holding position?? XXX
// XXX - it appears to be set, but never actually read. Not sure what I was thinking. XXX
int staticModelPosition[NUM_TOWERS][NUM_JOINTS] = {{0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}};

int pinMapping[NUM_TOWERS][NUM_JOINTS] = { {A0, A1, A3},
                                          {A4, A5, A7},
                                          {A8, A9, A11},
                                          {A12, A13, A15} };

// 'Pose' mode sets the towers to the current maquette state, and holds the pose for
// some number of seconds. It is triggered by a button press
int posePosition[NUM_TOWERS][NUM_JOINTS] = { {0,0,0},
                                             {0,0,0},
                                             {0,0,0},
                                             {0,0,0}};
uint8_t poseDataValid = FALSE; 
#define POSE_STATE_WAITING 0
#define POSE_STATE_HOLD 1
uint8_t poseState = POSE_STATE_WAITING;

// In 'Slave' mode, we're reading poses from some external source (such as 
// a pose list on the rpi) and adjusting the towers to match the current
// pose.
int slavePosition[NUM_TOWERS][NUM_JOINTS] = { {0,0,0},
                                              {0,0,0},
                                              {0,0,0},
                                              {0,0,0} };

uint8_t slaveValidBitmask = 0x00; 


typedef struct _ButtonState {
  uint8_t externalState;
  uint8_t internalState;
  uint8_t justPressed;
  uint8_t justReleased;
  int settleTime;
  int pin;
} ButtonState;

enum {
  BTN_MODE_0,
  BTN_MODE_1,
  BTN_MODE_2,
  BTN_MODE_3,
  BTN_MODE_4,
  BTN_POSE = 4
};

#define POSE_BUTTON_LED 13
#define POSE_BUTTON     12
#define POSE_LENGTH_MILLIS 10000



#define NBUTTONS 1
uint8_t ButtonArray[NBUTTONS] = {POSE_BUTTON};

static ButtonState buttonState[NBUTTONS];

static const char *modeStr[] = {"OFF", "IMMEDIATE", "POSE", "PLAYBACK"};

int modeIsLocal = TRUE; // could control via serial, or via local button                                          
                                          

static int16_t maquetteToModel(uint16_t pos, int i, int j);
static int16_t clamp(int16_t value, int16_t max, int16_t min);
static void targetSculpture(int positionArray[NUM_TOWERS][NUM_JOINTS]);
static void broadcastModelPosition();
static void setModeFromSwitch();
static int slaveDataValid();
static void readAllButtons();
static void InitButtons();
static ButtonState *readButtonState(int button);
static void readSerialCommand();
static void readCAN();
static uint8_t accumulateCommandString(uint8_t c);
static void parseSerialCommand();
static uint8_t isCentered();
static uint16_t readJointPos(int towerIdx, int jointIdx);

static void Write485(char *buf);

static void CAN_init();
static void CAN_sendData(unsigned char *data, unsigned char len);
static void CAN_receive();

static void HandleCANPosition(uint8_t towerId, uint8_t *buf);
static void HandleCANJointLimits(uint8_t towerId, uint8_t jointId, uint8_t *buf);
static void HandleCANGeneralStatus(uint8_t towerId, uint8_t *buf);
static void HandleCANValveStatus(uint8_t towerId, uint8_t *buf);
static void HandleCANJointStatus(uint8_t towerId, uint8_t *buf);
static void HandleCANHomingStatus(uint8_t towerId, uint8_t *buf);
static void HandleCANExtendedStatus(uint8_t towerId, uint8_t *buf);
static void HandleCANPIDValues(uint8_t towerId, uint8_t *buf);
static void HandleCANIntegrators(uint8_t towerId, uint8_t *buf);

#define BLINK_TIME 400
static int blinkOnOff = 1;
static unsigned long blinkTransitionTime = 0;
static void blinkPoseButton();
static void getPoseData();
static void initPoseMode();


void log_info(char *str);
void log_error(char *str);
void log_debug(char *str);

#define DBG_LEVEL_DEBUG 3
#define DBG_LEVEL_INFO 2
#define DBG_LEVEL_ERROR 1
static const char *debugString[] = {"OFF", "ERROR", "INFO", "DEBUG"};

int debug = 1;  // debug on/off
//#define SIMULATE_JOINTS

void log_info(char *str) 
{
  if (debug >= DBG_LEVEL_INFO) {
     Serial.print("INFO:" );
     Serial.println(str);
  }
}

void log_error(char *str) 
{
  if (debug >= DBG_LEVEL_ERROR) {
     Serial.print("ERROR:" );
     Serial.println(str);    
  }
}

void log_debug(char *str) 
{
  if (debug >= DBG_LEVEL_DEBUG) {
     Serial.print("DEBUG:" );
     Serial.println(str);    
  }
}

// 485 bus here...
#define NETWORKFILE   &uart1file

void setup(){
  Serial.begin(115200);
  UART1_Init(UART_115200);  // Network Baud out to nodes
  delay(100);
  InitButtons();
  InitLEDs();
  CAN_init();
  Serial.println("Starting maquette...");
  fprintf(NETWORKFILE, "Starting maquette...");
  //pinMode(35, INPUT_PULLUP);
}

int timerIdx = 0;
int secIdx = 0;
unsigned long mainloopTimeout = 0;
unsigned long poseTimeout = 0;

void loop() {
    char outBuf[4096];
    unsigned long curTime = millis();


//    secIdx++;
    
    readAllButtons();
    readSerialCommand();
    readCAN();
    
    if (curTime > mainloopTimeout) {
      mainloopTimeout = curTime + MAIN_LOOP_TIMEOUT_MILLIS;

      timerIdx++;
      secIdx++;


      if (timerIdx > 5) { // only run main control loop every 10th of a second
          timerIdx = 0; 
          

       // if (UART1_data_in_ring_buf()) {
       //   Serial.print("data on 485\n");
       //  }
  
        for (int i=0; i<NUM_TOWERS; i++) {
            for (int j=0; j<NUM_JOINTS; j++) {
                // For each joint, read the value from the maquette, and check whether that value
                // has changed more than the accepted delta from the previous position
                uint16_t potValue  = readJointPos(i, j);
                canonicalJointPosition[i][j] = maquetteRawToCanonical(potValue, i, j);
                modelPosition[i][j] = maquetteToModel(potValue, i, j);
/*                if (j == 1) {
                  char buf[512];
                  sprintf(buf, "pot val: %d, center: %d, model: %d\n", potValue, jointCenter[0][1], modelPosition[0][1]);
                  Serial.print(buf);
                }
*/
                if ( towerValid[i] && jointValid[i][j] && 
                   ((modelPosition[i][j] - 20 > staticModelPosition[i][j]) || 
                    (modelPosition[i][j] + 20 < staticModelPosition[i][j]))) {
                    //char buf[256];
                    //sprintf(buf, "Have movement on tower %d joint %d, old %d, new %d, potValue %d \n", i, j, staticModelPosition[i][j], modelPosition[i][j], potValue);
                    //Serial.print(buf);
                    staticModelPosition[i][j] = modelPosition[i][j];
                }
            }
        }
   
    
        // What I do here depends on what mode I'm in.
        switch (mode) {
            case MODE_OFF:
                break;
            case MODE_FREE_PLAY:
                if (isCentered()) {
                    targetSculpture(modelPosition);
                }
                break;
            case MODE_POSE:
                getPoseData(); // Get pose data, if ready.  
                if (poseDataValid && isCentered() ) {
                    targetSculpture(posePosition);
                    if (curTime > poseTimeout) {
                      poseState = POSE_STATE_WAITING;
                    } else {
                      blinkPoseButton();                      
                    }
                }
                break;
            case MODE_SLAVE:
                if (slaveDataValid()  && isCentered()) {
                    targetSculpture(slavePosition);
                }
                break;
            default:
                break;
        }
        if (debug) {
          //broadcastModelPosition();
        }
     }
   }
}

static uint8_t isCentered(void)
{
  uint8_t centerVal = TRUE;
  for (int i=0; i<NUM_TOWERS; i++){
    centerVal = centerVal && centered[i];
  }

  return centerVal;
}


static void setPoseButtonLED(int onOff)
{
    digitalWrite(POSE_BUTTON_LED, onOff?HIGH:LOW);
}

static void blinkPoseButton() 
{
   unsigned long curTime = millis();
   if (curTime > blinkTransitionTime) {
      blinkOnOff = 1 - blinkOnOff;
      setPoseButtonLED(blinkOnOff);
      blinkTransitionTime = curTime + BLINK_TIME;
   }
}

static void getPoseData() {
  if (poseState == POSE_STATE_WAITING) {
    // check for button press
    if (buttonState[0].justReleased) { // XXX NB - note that pose button is currently first in array. Should be a better thing than hard coding like this
      // get pose data
      for (int i=0; i<4; i++) {
        for (int j=0; j<3; j++) {
          posePosition[i][j] = modelPosition[i][j];
        }
      }
      poseDataValid = TRUE;
      
      // change state, set timeout
      poseState = POSE_STATE_HOLD; 
      poseTimeout = millis() + POSE_LENGTH_MILLIS;
    }
  } 
  
  // reset button state - throw away any presses that 
  // happen when state is not WAITING.
  buttonState[0].justReleased = FALSE;
}


static int slaveDataValid() {
  return slaveValidBitmask == 0x0F;
}

/*
static void setModeFromSwitch() {
  int newMode;
  if (buttonState[BTN_MODE_0].externalState) {
    newMode = MODE_OFF;
  } else if (buttonState[BTN_MODE_1].externalState) {
    newMode = MODE_FREE_PLAY;
  } else if (buttonState[BTN_MODE_2].externalState) {
    newMode = MODE_POSE;
  } else if (buttonState[BTN_MODE_3].externalState) {
    newMode = MODE_SLAVE;
  } else {
    return; // This is likely a debug state - no mode set externally
  }

  if (mode != newMode) {
    if (debug) {
      char strBuf[256];
      sprintf(strBuf, "Changing mode from %s to %s\n", modeStr[mode], modeStr[newMode]);
      Serial.print(strBuf);
    }
    
    if (newMode == MODE_POSE) {
      poseValidBitmask = 0x00;
    } else if (newMode == MODE_SLAVE) {
      slaveValidBitmask = 0x00;
    }
  }
  mode = newMode;
  
  return;
}
*/

static uint16_t simulatedJointPos = 100;
static uint16_t readJointPos(int towerIdx, int jointIdx) {
#ifdef SIMULATE_JOINTS
  simulatedJointPos++;
  if (simulatedJointPos > 300) {
    simulatedJointPos = 0;
  }
  return simulatedJointPos;
#else
  return analogRead(pinMapping[towerIdx][jointIdx]);
 #endif
}

static void InitLEDs() {
  pinMode(POSE_BUTTON_LED, OUTPUT);
}


static void InitButtons() {
  for (int i=0; i<NBUTTONS; i++) {
    pinMode(ButtonArray[i], INPUT_PULLUP);
    buttonState[i].pin = ButtonArray[i];
    buttonState[i].externalState = 0;
    buttonState[i].internalState = 0;
    buttonState[i].settleTime = 0;
    buttonState[i].justPressed = FALSE;
    buttonState[i].justReleased = FALSE;
  }
}

static void readAllButtons() {
  for (int i=0; i<NBUTTONS; i++) {
    readButtonState(i);
  }
}

static ButtonState *readButtonState(int button)
{
  if (button >= NBUTTONS) {
    return NULL;
  }
  
  ButtonState *state = &buttonState[button];
  int curState = digitalRead(state->pin);
  curState = 1-curState; // invert meaning, since we use a pull up

  // if the state is not the currently reported state of the button,
  // change either the externally reported state, or update
  // our settling counter
  if (curState != state->externalState) {
    if (curState == state->internalState) {
      state->settleTime++;
    } else {
      state->settleTime = 0;
      state->internalState = curState;
    }
    
    if (state->settleTime > SETTLE_TIME) {
      state->externalState = curState;
      state->settleTime = 0;
      if (curState) {
        state->justPressed = TRUE;
        state->justReleased = FALSE;
      } else {
        state->justPressed = FALSE;
        state->justReleased = TRUE;
      }
      if (debug) {
        char strBuf[256];
        sprintf(strBuf, "%s button %d\n", state->justPressed ? "PRESSED" : "RELEASED", button);
        Serial.print(strBuf);
      }
    }
  }
  return state;
}



/* Translate between potentiometer reading [0, 1204) 
   to model position (-200 to +200). Note that not all
   values read by the potentiometer are valid - we clamp the 
   value between MAX_POT and MIN_POT */


// At the risk of adding yet more complexity, I would consider
// going through an intermediary translation into canonical joint coordinates [-1, 1]
// This makes it a little easier to visualize what is going on, and is the
// standard way to represent this type of positional information.
// jointCanonical = (potVal - potValMin) / (potValMax - potValMin)
// And then - jointHydraulicsPos = hydraulicsValMin + jointCanonical*(hydraulicsValMax - hydraulicsValMin)

static float maquetteRawToCanonical(uint16_t potValue, int towerId, int jointId) {
   int16_t center = jointCenter[towerId][jointId];
   int16_t min = jointRange[towerId][jointId][0];
   int16_t max = jointRange[towerId][jointId][0];
   float canonicalValue = potValue < center ? (potValue - center)/(center - min) : (potValue - center)/(max - center);

   return clamp(canonicalValue, -1.0, 1.0);
}

static int16_t maquetteCanonicalToSculpture(int towerId, int jointId) {
   return (int16_t)(canonicalJointPosition[towerId][jointId]*SCULPTURE_RANGE/2) + SCULPTURE_RANGE/2; 
}


static int16_t maquetteToModel(uint16_t potValue, int towerId, int jointId) {
// Potentiometer has a range of POT_RANGE (1K)  XXX check this!!
// Sculpture has a range of SCULPTURE_RANGE (8K)

// Assuming the zeros are lined up, that means that the translation is
   long potValueCentered = ((long)potValue) - jointCenter[towerId][jointId];
   int16_t val = (int16_t)((potValueCentered * SCULPTURE_RANGE)/POT_RANGE);

   return clamp(val, jointRange[towerId][jointId][0], jointRange[towerId][jointId][1]);
}


static int16_t clamp(int16_t value, int16_t min, int16_t max) {
  if (value > max) 
    return max;
  if (value < min)
    return min;
  return value;
}

// Send target position to sculpture
static void targetSculpture(int positionArray[NUM_TOWERS][NUM_JOINTS]) {
    char modelString[256];
    char smallModelString[64];
    char *ptr = modelString;

    if (!isCentered()) {
      log_error("Target sculpture called on uncalibrated sculpture. Center first!!");
      return;
    }

    for (int i=0; i<NUM_TOWERS; i++){
        if (!towerValid[i]) {
          continue;
        }
        for (int j=0; j<NUM_JOINTS; j++) {
            if (!jointValid[i][j]) {
              continue;
            }
            sprintf(ptr, "<%d%dt%d>", i+1, j+1, positionArray[i][j]);
            ptr += strlen(ptr);
            
/*            if (i==0 && j==1) {
              sprintf(smallModelString, "<%d%dt%d>\r\n", i+1, j+1, positionArray[i][j]);
              Serial.print(smallModelString);
            }
*/
        }
    }
    sprintf(ptr, "\r\n");
    fprintf(NETWORKFILE, modelString);

    if (debug) {
      debug_info(modelString);
    }
}

void debug_info(char *debugStr)  // XXX add formatting
{
    char buf[512];
    sprintf(buf, "<!md{\"level\":\"info\", \"str\":\"%s\"}>\n", debugStr);
    Serial.print(buf);
}


// Let the rest of the world know what we're doing
// Here I'm going to work in JSON, since that's easiest for me to parse
static void broadcastModelPosition() {
    char modelString[256];
    char *ptr = modelString;
    sprintf(ptr, "[");
    ptr += strlen(ptr);
    for (int i=0; i<NUM_TOWERS; i++){
        sprintf(ptr, "{'tower': %d, 'joints':[", i);
        ptr += strlen(ptr);
        for (int j=0; j<NUM_JOINTS; j++) {
            sprintf(ptr, "%.2f,", canonicalJointPosition[i][j]);
            ptr += strlen(ptr);
        }
        ptr -= 1; // cut trailing ','
        sprintf(ptr, "]},");
        ptr += strlen(ptr);
    }
    ptr -= 1; // cut trailing '\n'
    sprintf(ptr, "]\n");
    Serial.print(modelString); 
}


#define MAX_COMMAND_LENGTH 32
static uint8_t cmd_str[MAX_COMMAND_LENGTH];
static uint8_t cmd_len = 0;

static void readSerialCommand()
{
  //Serial.println("Sanity test...");
  while(Serial.available() > 0) {
    int haveCommand = FALSE;
    haveCommand = accumulateCommandString(Serial.read());
    if (haveCommand) {
       log_debug("Have command");
      parseSerialCommand();
    }
  }
}




static uint8_t accumulateCommandString(uint8_t c)
{
  /* catch beginning of this string */
  if (c == '<') { // this will catch re-starts and stalls as well as valid commands.
     cmd_len = 0;
     cmd_str[cmd_len++] = c;
     return 0;
  }
  
  if (cmd_len != 0) {   // string in progress, accumulate next char
    if (cmd_len < MAX_COMMAND_LENGTH - 1) 
      cmd_str[cmd_len++] = c;
      
    if (c == '>') {
        //char buf[128];
        //memcpy(buf, cmd_str, cmd_len);
        //buf[cmd_len] = '\0';
        cmd_str[cmd_len] = '\0';
        cmd_len = 0;
        //putstr(buf);
        return 1;
    }
  }
  return 0;
}



static void parseSerialCommand() 
{
  int8_t towerId=0;    /* tower we're working with, if there is one */
  uint8_t c;            /* next char to parse */
  int len = strlen(cmd_str);
#if 0
  int8_t jointId=0;    /* joint we're working with, if necessary */
  int val;
  char *limitstr[3] = {NULL, NULL, NULL};
  uint16_t jtarget[3];  /* joint target data */
  uint16_t jcenter[3];  /* joint center data. Raw units from the potentiometer */
  int maxVal;
  int minVal;
  int centerVal;
  int idx = 0;
  int ntokens = 0;
  char strBuf[1024];
#endif

  log_debug("Received command");

  if (len < 3 ) {
    log_error("Command too short");
    return;
  }

  /* first character after the '<' is either a command to a tower, or an 'm', signifying that this
     is a command to the maquette itself */
  c = cmd_str[1];  
  if (c == 'm') {
    parseMaquetteCommand(cmd_str+2, len-2); // skip '<m'
  } else {  // parse tower command here
    //Sanity check - first character should be the tower id
    towerId = c - '0';
    if (towerId < 0 || towerId > 4) {
        log_error("Invalid tower id");
    } else {
        parseTowerCommand(cmd_str, len);
    }
  }
}

static void initPoseMode()
{
  mode = MODE_POSE;
  poseState = POSE_STATE_WAITING;
  poseDataValid = FALSE;
  blinkTransitionTime = 0;
  blinkOnOff = 0;
}

static void parseMaquetteCommand(char *buf, int len) {
    int maxVal;
    int minVal;
    int centerVal;
    char c;
    char *ptr = buf;
    char *bufEnd = buf + len;
    uint8_t val;
    char outBuf[2*1024];
    uint8_t towerId;
    uint8_t jointId;
    uint16_t jtarget[3];  /* joint target data */
    uint16_t jcenter[3];  /* joint center data. Raw units from the potentiometer */
    int ntokens;
    
    if (bufEnd <= buf) return;
    
    c = *ptr++;
    log_debug("maquette command!!!");
    //Serial.println("maquette command!!!");
    //Serial.println(c);
    
    switch(c) {
    case 'L':  // set limits for a tower on maquette 
      /* next character should be a digit - tower 
         Note that this clause will catch EOL */
      c =*ptr++;
      towerId = c -'0';
      if (towerId < 1 || towerId > 4) {
        log_error("cannot parse command string, unknown tower");
        return;
      } 
      towerId--; // 0 based indexing

      c = *ptr++;
      jointId = c - '0';
      if (jointId < 1 || jointId > 3) {
        Serial.println("cannot parse command string, unknown joint");
        return;
      }
      jointId--; // 0 based indexing
      
      ntokens = sscanf (ptr,"%d,%d>",&minVal, &maxVal);

      if (ntokens < 2) {
        log_error("not enough values in command");
        return;
      }  
      
      jointRange[towerId][jointId][0] = minVal;
      jointRange[towerId][jointId][1] = maxVal;

      if (debug) {
        sprintf(outBuf, "Maquette Tower %d joint %d set to min: %d, max: %d,\n", towerId+1, jointId+1, minVal, maxVal);
        log_debug(outBuf);
      }
      break; 
    case 'D':  // set debug level
      Serial.println("Setting debug level"); 
      c = *ptr++;
      if (((c - '0') >= 0) && ((c - '0') <= 3)) {
        debug = c - '0';
        sprintf(outBuf, "Setting debug level to %s\n", debugString[debug]);
        Serial.println(outBuf);
      } else {
        sprintf(outBuf, "Unknown debug value %d\n", c - '0');
        Serial.println(outBuf);
      }
      break; 
    case 'M':  // set mode
      c =*ptr++;
      val = c - '0';
      sprintf(outBuf, "Setting mode to %d", val);
      log_debug(outBuf);
      modeIsLocal = FALSE;
      switch(val) {
        case 0:
            mode = MODE_OFF;
            break;
        case 1:
            mode = MODE_FREE_PLAY;
            break;
        case 2:
            initPoseMode();
           break;
        case 3:
            mode = MODE_SLAVE;
            break;
        default:
            modeIsLocal = TRUE;
            log_debug("Restoring mode control to switch");
            break;
      }

      if (mode != MODE_SLAVE) {
        slaveValidBitmask = 0x00;
      }

      if (!modeIsLocal) {
        if (debug ) {
          sprintf(outBuf, "Mode set to %s\n", modeStr[mode]);
          log_debug(outBuf);
        }     
      }
      break; 
    case 's': // Get status
        sprintf(outBuf, "<!ms{\"mode\": %d, \"poseState\": %d, \"towerState\": [", mode, poseState);
        for (int i=0; i<4; i++) {
            char towerStr[1024];
            sprintf(towerStr, "{\"tower\": %d, \"jointPos\" : [%d, %d, %d], \"jointCenter\": [%d, %d, %d], \"jointLimits\": [[%d,%d],[%d,%d],[%d,%d]], \"towerValid\":%s, \"jointValid\" : [%s, %s, %s]},",
                                i, modelPosition[i][0], modelPosition[i][1], modelPosition[i][2],
                                jointCenter[i][0], jointCenter[i][1], jointCenter[i][2],
                                jointRange[i][0][0],jointRange[i][0][1], jointRange[i][1][0], jointRange[i][1][1], jointRange[i][2][0], jointRange[i][2][1],
                                (towerValid[i] ? "true" : "false"),
                                (jointValid[i][0] ? "true" : "false"),
                                (jointValid[i][1] ? "true" : "false"),
                                (jointValid[i][2] ? "true" : "false"));
                                
            strcat(outBuf, towerStr);
        }
        outBuf[strlen(outBuf)-1] = '\0'; // remove trailing ','
        strcat(outBuf, "]}>");
        Serial.println(outBuf);
        break;
    case 'P':  // set slave (pose) data
        ntokens = sscanf (ptr,"t%hhu:%hu,%hu,%hu", 
                          &towerId, &jtarget[0], &jtarget[1], &jtarget[2]); 
        if (ntokens != 4 || towerId > 4 || towerId < 1) {
          log_error("Invalid tower");
          break;
        }
        towerId--; // 0 based idx
        for (int i=0; i<3; i++) {
          slavePosition[towerId][i] = clamp(jtarget[i], jointRange[towerId][i][0], jointRange[towerId][i][1]);
          sprintf(outBuf, "Clamping %d between %d and %d, result %d\n", jtarget[i], jointRange[towerId][i][0], 
                                                                                    jointRange[towerId][i][1], 
                                                                                    slavePosition[towerId][i]);
          log_debug(outBuf);
        } 
        slaveValidBitmask |= (1 << towerId);

        if (debug) {
           sprintf(outBuf, "Set pose data for tower %d to %d, %d, %d\n", towerId+1, slavePosition[towerId][0], 
                                                                                    slavePosition[towerId][1],
                                                                                    slavePosition[towerId][2]);
           log_debug(outBuf);
           sprintf(outBuf, "Slave bitmask is %x\n", slaveValidBitmask);
           log_debug(outBuf);
        }
        break;  
    case 'C': 
        // Set center of joint (calibrate)
        // C<towerId>:<joint_center1>,<joint_center2>,<joint_center3>
        // There are two ways to use this command - either by sending just the tower id and no joint parameters
        // or by sending the tower id and all three joint parameters. In the first case, the joint
        // centers will be set to the current position of the joints
        
        ntokens = sscanf(ptr, "%hhu:%hu,%hu,%hu", &towerId, &jcenter[0], &jcenter[1], &jcenter[2]);  
        
        if (ntokens != 1 && ntokens != 4) {
          log_error("Invalid number of center tokens. Require 3 or none");
          break; 
        }

        if (towerId > 4 || towerId < 1) {
          log_error("Invalid tower");
          break;
        }

        towerId--; // 0 based indexing from now on

        if (ntokens == 1) {
          // take the current center of the joint as the center
          for (int i=0; i<3; i++) {
            jcenter[i] = readJointPos(towerId, i); 
          }
        } else if (ntokens == 4) {
          if (jcenter[0] > POT_RANGE || jcenter[0] < 0) {
              log_error("Joint 1 center out of range\n");
              break;
          } else if (jcenter[1] > POT_RANGE || jcenter[1] < 0) {
              log_error("Joint 2 center out of range\n");
              break;
          } else if (jcenter[2] > POT_RANGE || jcenter[2] < 0) {
              log_error("Joint 3 center out of range\n");
              break;
          } 
       } 
       
       for (int i=0; i<NUM_JOINTS; i++) {
          jointCenter[towerId][i] = jcenter[i];   
       }
       centered[towerId] = TRUE;
       Serial.println(maquetteToModel(jcenter[0], towerId, 0));
       
       break;
    case 'c': // Get calibration status
      sprintf(outBuf, "<!c[%d, %d, %d, %d]>", centered[0], centered[1], centered[2], centered[3]);
      Serial.println(outBuf);
      break;
    case 'E': // enable/disable tower
      ntokens = sscanf(ptr, "%hhu:%hhu", &towerId, &val);
      if (ntokens != 2) {
          log_error("Invalid number of tokens. Require 2");
          break;
      }
      if (towerId > NUM_TOWERS || towerId < 1) {
          log_error("Invalid tower");
          break;
      }
      if (val != 0 && val != 1) {
          log_error("value must be 0 or 1 (on/off)");
          break;      
      }
      towerValid[towerId-1] = val;
      break;
       
    case 'e': // enable/disable joint
      ntokens = sscanf(ptr, "%hhu:%hhu:%hhu", &towerId, &jointId, &val); 
      if (ntokens != 3 ) {
          log_error("1 Invalid number of tokens. Require 3");
          break;
      }


      if (towerId > NUM_TOWERS || towerId < 1) {
          log_error("Invalid tower");
          log_error(towerId);
          log_error(jointId);
          log_error(val);
          break;
      }
      if (jointId > NUM_JOINTS || jointId < 1) {
          log_error("Invalid joint");
          break;
      }
      if (val != 0 && val != 1) {
          log_error("value must be 0 or 1 (on/off)");
          break;      
      }
      jointValid[towerId-1][jointId-1] = val;
      break;
        
    case 'S': // simulation. Get the position that we would be sending the towers
       sprintf(outBuf, "[[%d,%d,%d], [%d,%d,%d],[%d,%d,%d],[%d,%d,%d]]", 
            modelPosition[0,0], modelPosition[0,1], modelPosition[0,2],
            modelPosition[1,0], modelPosition[1,1], modelPosition[1,2],
            modelPosition[2,0], modelPosition[2,1], modelPosition[2,2],
            modelPosition[3,0], modelPosition[3,1], modelPosition[3,2]);
       Serial.println(outBuf);
       break;

    case 'A': // analog read of the joints. Sanity check
      sprintf(outBuf, "[[%d,%d,%d], [%d,%d,%d],[%d,%d,%d],[%d,%d,%d]]",
        readJointPos(0,0), readJointPos(0,1), readJointPos(0,2),
        readJointPos(1,0), readJointPos(1,1), readJointPos(1,2),
        readJointPos(2,0), readJointPos(2,1), readJointPos(2,2),
        readJointPos(3,0), readJointPos(3,1), readJointPos(3,2));
        
      Serial.println(outBuf);
      break;
       
    default:
      break; 
    }
} 

static void parseTowerCommand(char *buf, int len) {
    // For the most part, we don't parse tower commands. We just send them on
    uint8_t towerId = buf[1] - '0';
    uint8_t jointId;
    char secondChar = buf[2];
    char command;
    int ret;
    char outBuf[64];
    
    
    if (secondChar <= '3' && secondChar >= '0') {
        // joint id. Next is command
        if (len < 4) {
            log_error("Tower command too short!");
            return;
        }
        jointId = secondChar-'0';
        command = buf[3];
    } else {
        command = buf[2];
    }
    
    switch(command){
    case 's':
        // Get tower status
        log_debug("Get tower status");
        ret = CAN_RequestGeneralStatus(towerId);
        sprintf(outBuf, "Get tower status returns %d\n", ret);
        Serial.print(outBuf);
        break;
    case 'j':
        // Get joint status
        CAN_RequestJointStatus(towerId, jointId);
        break;
    case 'l':
        CAN_RequestJointLimits(towerId, jointId);
        break;
    case 'H':
        // Home joint. pass through
        Serial.print("Writing home request to 485\n");
        Write485(buf);
        break;
    case 'T':
        CAN_RequestPosition(towerId);
        break;
    case 'h':
        // set homing speed. Pass through
        Write485(buf);
        break;
    case 't':
        // Set tower position. Pass through
        Write485(buf); // XXX - probably not allowed in slave mode or playback XXX
        break;
    case 'L':
        // Set tower limits. Pass through
        Write485(buf);
        break;
    case 'I':
        // Set I values. Pass through
        Write485(buf);
        break;
    case 'i':
        // CAN Request integrator values.
        CAN_RequestIntegrators(towerId);
        break;
    case 'P':
        // Set P values. Pass through
        Write485(buf);
        break;
    case 'p':
        // Request PID values
        CAN_RequestPIDValues(towerId);
        break;
    case 'e':
        // Get extended status
        CAN_RequestExtendedStatus(towerId);
        break;
    case 'm':
        // set min values. Pass through
        Write485(buf);
        break;
    case 'M':
        // set max values. Pass through
        Write485(buf);
        break;
    case 'C':
        // set center values. Pass through
        Write485(buf);
        break;
    case 'w':
        // Manually home joint. Pass through
        Write485(buf);
        break;
    case 'r':
        // Manually force joint into running mode
        Write485(buf);
        break; 
    case 'N':
        // neuter
        Write485(buf);
        break;
    case 'D':
        // set joint limits
        Write485(buf);
        break;
    case 'x':
        // set dead band
        Write485(buf);
        break;
    case 'z':
        // clear error condition
        Write485(buf);
        break;
    default:
      Serial.println("Unknown command to maquette");
    }
}
        
static void Write485(char *buf)
{
    Serial.print("writing to 485: ");
    Serial.println(buf);
    fprintf(NETWORKFILE, buf);
}




// CAN stuff here...
// Build an ID or PGN

// CAN transmission id. This message is used for bus arbitration. Depending on the 
// application, it may also be used to identify the sender - that seems to be commonly
// done, but is not strictly part of the CAN protocol
// 
// Note also that the 'dominant' bit - ie, the one that gets priority in bus arbitration -
// is a 0. So 0x0001 gets priority over 0x00010. Or - low numbers rule.

long unsigned int txID = 0x1881ABBA;// This format is typical of a 29 bit identifier.. 
                                    // the most significant digit is never greater than one.
                                    // Note that this is a compatiblity issue for CAN 
                                    // controllers - see 
                                    // https://www.kvaser.com/about-can/the-can-protocol/can-messages-33/
unsigned char stmp[8] = {0x0E, 0x00, 0xFF, 0x22, 0xE9, 0xFA, 0xDD, 0x51};

//Construct a MCP_CAN Object and set Chip Select to 53.

//MCP_CAN CAN(SPI_CS_PIN);                            


static void CAN_init()
{
    while (CAN_OK != mcp_can_begin(CAN_500KBPS, SPI_CS_PIN))              // init can bus : baudrate = 500K
    {
        Serial.println("CAN BUS Module Failed to Initialized");
        Serial.println("Retrying....");
        delay(200); 
    }

    
    while (CAN_OK != mcp_can_set_mode(CAN_WRAPPER_MODE_NORMAL)) {
        Serial.println("CAN Bus could not set mode back to Normal");
        Serial.println("Retrying....");
        delay(200);
    }

    Serial.println("CAN BUS Shield init ok!");
    CAN_setTxId(MAQUETTE_TX_ID); //0x15FFFFFF //MAQUETTE_TX_ID
}

static void readCAN()
{
/* NB - errors don't clear by themselves. if you want to check the
 *  error for debug purposes, you need to clear it or you end up 
 *  doing nothing other than checking for errors...
 */

 /*
  if (mcp_can_check_error()) {  // first let's check for errors...
    Serial.println("Error on CAN bus!");
    
  } else 
*/
  
  if (CAN_MSGAVAIL == mcp_can_check_receive()) {        
    Serial.println("DATA on CAN!!");
    CAN_receive();
  } 
}


static void CAN_receive()
{
    uint8_t buf[8];
    uint8_t len = 0;
    uint32_t txId;
    uint8_t jointId;
    uint8_t command;
    uint8_t towerId;
    char outBuf[64];
    
    mcp_can_receive(&txId, &len, buf); 
    if (len < 3) {
        sprintf(outBuf,"Invalid length of CAN message, %d", len);
        log_error(outBuf);
        return;
    }

    txId = txId & 0x1FFFFFFF;
    
    Serial.print("Received message on CAN bus, length: ");
    Serial.println(len);
    //Serial.print(", contents: ");
    //buf[7] = '\0'; // for this test we're getting back a string... I think. Anyway, terminate it so we don't crash by accident
    //Serial.print((char*)buf);
    
    // Determine which tower this came from
    switch( txId ){
    case TOWER_1_TX_ID:
        towerId = 1;
        break;
    case TOWER_2_TX_ID:
        towerId = 2;
        break;
    case TOWER_3_TX_ID:
        towerId = 3;
        break;
    case TOWER_4_TX_ID:
        towerId = 4;
        break;
    default:
        sprintf(outBuf, "Unknown CAN message: txId %ul", txId);
        log_error(outBuf);
        return;
    }
    
    // Depending on the message type, do the appropriate thing
    switch(buf[CAN_CMD_BYTE]){
    case CAN_MSG_POSITION:
        HandleCANPosition(towerId, buf);
        break;
    case CAN_MSG_JOINT_LIMITS:
        jointId = (buf[CAN_TOWER_JOINT_BYTE] >> CAN_JOINT_SHIFT) & CAN_JOINT_MASK;
        HandleCANJointLimits(towerId, jointId, buf);
        break;
    case CAN_MSG_GENERAL_STATUS:
        HandleCANGeneralStatus(towerId, buf);
        break;
    case CAN_MSG_VALVES:
        HandleCANValveStatus(towerId, buf);
        break;
    case CAN_MSG_JOINT_STATUS:
        jointId = (buf[CAN_TOWER_JOINT_BYTE] >> CAN_JOINT_SHIFT) & CAN_JOINT_MASK;
        HandleCANJointStatus(towerId, jointId,  buf);
        break;
    case CAN_MSG_HOMING_STATUS:
        jointId = (buf[CAN_TOWER_JOINT_BYTE] >> CAN_JOINT_SHIFT) & CAN_JOINT_MASK;
        HandleCANHomingStatus(towerId, jointId, buf);
        break;
    case CAN_MSG_HOMING_RESULT:
        jointId = (buf[CAN_TOWER_JOINT_BYTE] >> CAN_JOINT_SHIFT) & CAN_JOINT_MASK;
        HandleCANHomingResult(towerId, jointId, buf);
        break;
    case CAN_MSG_EXTENDED_STATUS:
        HandleCANExtendedStatus(towerId, buf);
        break;
    case CAN_MSG_PID_VALUES:
        HandleCANPIDValues(towerId, buf);
        break;
    case CAN_MSG_INTEGRATORS:
        HandleCANIntegrators(towerId, buf);
        break;
    /*
    case CAN_MSG_ACK:
        CAN_HandleAck(towerId, buf);
        break;
    */ 
    default:
        { 
          char buf[256];
          sprintf(buf, "Invalid CAN message, type %d\n", buf[CAN_CMD_BYTE]);
          log_error(buf);
        }
        return;
    }

}



// Translate CAN messages into a friendlier format, send them on... 

static void HandleCANPosition(uint8_t towerId, uint8_t *buf) 
{
    uint8_t serialBuf[256];
    CAN_Position pos;
    
    CAN_BufferToPosition(buf, &pos);
    sprintf(serialBuf, "<!T%d[%d,%d,%d]>", towerId, pos.joint1Pos, pos.joint2Pos, pos.joint3Pos);
    Serial.println((char *)serialBuf);
}

static void HandleCANJointLimits(uint8_t towerId, uint8_t jointId, uint8_t *buf)
{
    uint8_t serialBuf[256];
    CAN_Limits limits;
    
    CAN_BufferToJointLimits(buf, &limits);
    sprintf(serialBuf, "<!l%d%d{\"min\":%d,\"max\":%d, \"center\": %d}>", 
                         towerId, jointId, 
                         limits.minPos, limits.maxPos, limits.centerPos);
    Serial.println((char *)serialBuf);
}

static void HandleCANGeneralStatus(uint8_t towerId, uint8_t *buf) 
{
    uint8_t serialBuf[512];
    CAN_StatusStruct canStatus;

    CAN_BufferToStatus(buf, &canStatus);
    
    sprintf(serialBuf, "<!x%d{\"homed\":[%s,%s,%s], \"switches\": [%d, %d, %d], \"enabled\": [%s, %s, %s], \"running\": %s,\"error\": %d}>", 
                              towerId,
                              TF_STRING(canStatus.homed[0]), 
                              TF_STRING(canStatus.homed[1]), 
                              TF_STRING(canStatus.homed[2]),
                              canStatus.sw[0], canStatus.sw[1], canStatus.sw[2],
                              TF_STRING(canStatus.jointEnable[0]), 
                              TF_STRING(canStatus.jointEnable[1]), 
                              TF_STRING(canStatus.jointEnable[2]),
                              TF_STRING(canStatus.runState),
                              canStatus.state),
    Serial.println((char *)serialBuf);  
}

static void HandleCANValveStatus(uint8_t towerId, uint8_t *buf)
{
    uint8_t serialBuf[512];
    CAN_Valves valves;

    CAN_BufferToValves(buf, &valves);
    sprintf(serialBuf, "<!v%d[%d, %d, %d]>", towerId, 
                                             valves.driveLevel[0],
                                             valves.driveLevel[1],
                                             valves.driveLevel[2]);

    Serial.println((char *)serialBuf);
  
}

static void HandleCANJointStatus(uint8_t towerId, uint8_t jointId, uint8_t *buf)
{
    uint8_t serialBuf[1024];
    CAN_JointStatus joint; 

    CAN_BufferToJointStatus(buf, &joint);

    sprintf(serialBuf, "<!j%d%d{\"jointId\":%d, \"pos\":%d, \"target\":%d, \"valve\":%d, \"homed\":%s, \"enabled\":%s, \"switches\":0x%x}>",
            towerId, jointId, 
            jointId,
            joint.pos,
            joint.target,
            joint.valve,
            TF_STRING(joint.homed),
            TF_STRING(joint.enabled),
            joint.switches);
    Serial.println((char *)serialBuf);
    
}

static void HandleCANHomingStatus(uint8_t towerId, uint8_t jointId, uint8_t *buf)
{
    uint8_t serialBuf[512];
    CAN_Homing homing; 

    CAN_BufferToHomingStatus(buf, &homing);

    sprintf(serialBuf, "<!H%d%d{\"sw\":%d, \"pos\":%d, \"mask\":%d, \"stalls\":%d}>", 
                      towerId, jointId, 
                      homing.switches, homing.pos, homing.target, homing.stalls);
    Serial.println((char *)serialBuf);
                      
}

static const char * homingReason(uint8_t detail)
{
  char *ret = "Unknown";
  switch(detail) {
  case HOMING_RESULT_OK:
    ret = "OK";
    break;
  case HOMING_RESULT_STALL:
    ret = "Stall";
    break;
  case HOMING_RESULT_CENTER_NOT_FOUND:
    ret = "Center not found";
    break;
  case HOMING_RESULT_ESTOP:
    ret = "ESTOP pressed";
    break;
  case HOMING_RESULT_CLEAR:
    ret = "CLEAR pressed";
    break;
   default:
    break;
  }
  return ret;
}

static void HandleCANHomingResult(uint8_t towerId, uint8_t jointId, uint8_t *buf)
{
    uint8_t serialBuf[512];
    CAN_Homing_Result homing; 

    CAN_BufferToHomingResult(buf, &homing);

    sprintf(serialBuf, "<!H%d%d{\"homed\":%s, \"detail\":\"%s\", \"target\":%d}>", 
                      towerId, jointId, 
                      TF_STRING(homing.success), 
                      homingReason(homing.detail),
                      homing.target);
    Serial.println((char *)serialBuf);             
}



static void HandleCANExtendedStatus(uint8_t towerId, uint8_t *buf)
{
    uint8_t serialBuf[512];
    CAN_ExtendedStatus status;

    CAN_BufferToExtendedStatus(buf, &status);

    sprintf(serialBuf, "<!e%d{\"addr\":%d,\"debug\":%d, \"h1\":%d, \"h2\":%d, \"h3\":%d}>",
              towerId,
              status.debugLevel, 
              status.homeSpeed[0], status.homeSpeed[1], status.homeSpeed[2]);
    Serial.println((char *)serialBuf);       
}


static void HandleCANPIDValues(uint8_t towerId, uint8_t *buf) 
{
    uint8_t serialBuf[512];
    CAN_PIDValues pids;

    CAN_BufferToPIDValues(buf, &pids);

    sprintf(serialBuf, "<!p%d[{\"p\":%d,\"i\":%d},{\"p\":%d,\"i\":%d}, {\"p\":%d,\"i\":%d}]>",
              towerId,
              pids.Kp[0], pids.Ki[0], 
              pids.Kp[1], pids.Ki[1], 
              pids.Kp[2], pids.Ki[2]);
              
    Serial.println((char *)serialBuf);       
}

static void HandleCANIntegrators(uint8_t towerId, uint8_t *buf)
{
    uint8_t serialBuf[512];
    CAN_Integrators integrators;

    CAN_BufferToIntegrators(buf, &integrators);

    sprintf(serialBuf, "<!i%d[%d, %d, %d]>",
              towerId,
              integrators.i1,
              integrators.i2,
              integrators.i3);
                
    Serial.println((char *)serialBuf); 
}


static void CAN_sendData(unsigned char *data, unsigned char len)
{   
    byte ret;
    if (len > 8) {
      log_error("Bad CAN data len");
      return;
    }

    ret = mcp_can_check_error();
    if (ret != CAN_OK) {
      Serial.println("Error on CAN Bus!! not attempting a send!!");
      return;
    }
    
    log_debug("sending CAN data");

    // send the data:  id = 0x00, Extended Frame, data len = 8, stmp: data buf
    // Extended Frame = 1.
   
    ret = mcp_can_send(0x1000, 1, len, data);
//    ret = mcp_can_send(MAQUETTE_TX_ID, 1, len, data);
    if (ret != CAN_OK) {
      Serial.print("Failed sending CAN data: ");
      Serial.println(ret);
    }
  
    //delay(25);    // send data every 25mS
}



