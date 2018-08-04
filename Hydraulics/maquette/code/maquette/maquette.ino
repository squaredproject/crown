#include <mcp_can_dfs.h>
#include <mcp_can.h>

#include <SPI.h>

// basic arduino code
// Weirdly with arduino ide all the standard headers
// appear to get added for you. I'm not sure how I feel about that...

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

#define SETTLE_TIME 2

const int SPI_CS_PIN = 53;



typedef enum {
   MODE_OFF = 0,
   MODE_FREE_PLAY,
   MODE_POSE,
   MODE_SLAVE
} Mode;

Mode mode = MODE_OFF;

uint16_t jointRange[NUM_TOWERS][NUM_JOINTS][2] = {{{-300, 200}, {-532, 328}, {-275, 276}},
                                                  {{-200, 200}, {-200, 200}, {-200, 200}},
                                                  {{-200, 200}, {-200, 200}, {-200, 200}},
                                                  {{-200, 200}, {-200, 200}, {-200, 200}}};

/*uint16_t jointRange[NUM_TOWERS][NUM_JOINTS][2] = {{{-200, 200}, {-200, 200}, {-200, 200}},
                                                  {{-200, 200}, {-200, 200}, {-200, 200}},
                                                  {{-200, 200}, {-200, 200}, {-200, 200}},
                                                  {{-200, 200}, {-200, 200}, {-200, 200}}};
*/
uint16_t jointCenter[NUM_TOWERS][NUM_JOINTS] = {{POT_RANGE/2,POT_RANGE/2,POT_RANGE/2}, {POT_RANGE/2,POT_RANGE/2,POT_RANGE/2}, {POT_RANGE/2,POT_RANGE/2,POT_RANGE/2}, {POT_RANGE/2,POT_RANGE/2,POT_RANGE/2}};
                               
// XXX - above. At some point I use real values. Not just yet though.

int modelPosition[NUM_TOWERS][NUM_JOINTS];
int pinMapping[NUM_TOWERS][NUM_JOINTS] = { {A0, A1, A2},
                                          {A3, A4, A5},
                                          {A6, A7, A8},
                                          {A9, A10, A11} };
                                          
int posePosition[NUM_TOWERS][NUM_JOINTS] = { {0,0,0},
                                             {0,0,0},
                                             {0,0,0} };
uint8_t poseValidBitmask = 0x00; 

int slavePosition[NUM_TOWERS][NUM_JOINTS] = { {0,0,0},
                                             {0,0,0},
                                             {0,0,0} };

uint8_t slaveValidBitmask = 0x00; 

int debug = 1;  // debug off


typedef struct _ButtonState {
  uint8_t externalState;
  uint8_t internalState;
  uint8_t justPressed;
  uint8_t justReleased;
  int settleTime;
  int pin;
} ButtonState;

enum {
  BTN_TOWER_1 = 0,
  BTN_TOWER_2,
  BTN_TOWER_3,
  BTN_TOWER_4,
  BTN_MODE_0,
  BTN_MODE_1,
  BTN_MODE_2,
  BTN_MODE_3,
  BTN_MODE_4,
  BTN_POSE
};

#define NBUTTONS 10
uint8_t ButtonArray[NBUTTONS] = { 34, 35, 36, 37, 38,
                                  39, 40, 41, 42, 43 };

static ButtonState buttonState[NBUTTONS];

static const char *modeStr[] = {"OFF", "IMMEDIATE", "POSE", "PLAYBACK"};

int modeIsLocal = TRUE; // could control via serial, or via local button                                          
                                          

static int16_t maquetteToModel(uint16_t pos, int i, int j);
static int16_t clamp(int16_t value, int16_t max, int16_t min);
static void targetSculpture(int positionArray[NUM_TOWERS][NUM_JOINTS]);
static void broadcastModelPosition();
static void setModeFromSwitch();
static int poseDataValid();
static int slaveDataValid();
static void readAllButtons();
static void InitButtons();
static ButtonState *readButtonState(int button);
static void readSerial();
static void readSerial1();
static uint8_t accumulateCommandString(uint8_t c);
static void parseCommand();

static void CAN_init();
static void CAN_sendData(unsigned char *data, unsigned char len);


void setup(){
  Serial.begin(115200);
  Serial1.begin(115200);
  InitButtons();
  //CAN_init();
  Serial.println("Starting maquette...");
  //Serial1.println("Starting maquette...");
  //pinMode(35, INPUT_PULLUP);
}

int timerIdx = 0;
void loop() {
    delay(20); 

    //CAN_sendData("hi there", 8);

    timerIdx++;
    
    readAllButtons();

    readSerial();
    readSerial1();

    //Serial1.println("This is the 485 bus talking to you\r\n");
    
    if (timerIdx > 100) { // only run main control loop every 10th of a second
      timerIdx = 0; 

      for (int i=0; i<NUM_TOWERS; i++) {
          for (int j=0; j<NUM_JOINTS; j++) {
              uint16_t potValue  = analogRead(pinMapping[i][j]);
              modelPosition[i][j] = maquetteToModel(potValue, i, j);
             /* if (i==0 && j==1) { // debugging shit
                Serial.print("maquette read middle joint of first tower, raw: ");
                Serial.print(potValue);
                Serial.print(" , sculpture ");
                Serial.println(modelPosition[i][j]);
              } */
          }
      }
  
      // check value of buttons and switches
      if (modeIsLocal) {
        setModeFromSwitch();
      } 
 
  
      // What I do here depends on what mode I'm in.
      switch (mode) {
          case MODE_OFF:
              break;
          case MODE_FREE_PLAY:
              targetSculpture(modelPosition);
              break;
          case MODE_POSE:
              if (poseDataValid() ) {
                  targetSculpture(posePosition);
              }
              break;
          case MODE_SLAVE:
              if (slaveDataValid() ) {
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

static int poseDataValid() {
  if (buttonState[BTN_MODE_2].justReleased) {
    buttonState[BTN_MODE_2].justReleased = FALSE;

    if (debug) {
      char strBuf[256];
      sprintf(strBuf, "Setting Pose data!\n");
      Serial.print(strBuf);
    }

    
    // collect pose data
    for (int i=0; i<4; i++) {
      for (int j=0; j<3; j++) {
        posePosition[i][j] = modelPosition[i][j];
      }
    }
    poseValidBitmask = 0x0F;
  }
  return (poseValidBitmask == 0x0F);
}

static int slaveDataValid() {
  return slaveValidBitmask == 0x0F;
}


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



/* Translate between potentiometer reading (-16K : +16K) 
   to model position (-200 to +200). Note that not all
   values read by the potentiometer are valid - we clamp the 
   value between MAX_POT and MIN_POT */

static int16_t maquetteToModel(uint16_t potValue, int towerId, int jointId) {
// Potentiometer has a range of POT_RANGE (1K)
// Sculpture has a range of SCULPTURE_RANGE (8K)
// Assuming the zeros are lined up, that means that the translation is
   long potValueCentered = ((long)potValue) - jointCenter[towerId][jointId];
   int16_t val = (int16_t)((potValueCentered * SCULPTURE_RANGE)/POT_RANGE);
/*   if (towerId==0 && jointId == 1){
    Serial.print(potValueCentered);
    Serial.print(", ");
    Serial.println(val);
   }
*/
   return clamp(val, jointRange[towerId][jointId][0], jointRange[towerId][jointId][1]);
//  int16_t value = clamp(pos, MIN_POT, MAX_POT) - 512;
    
//    return  ((long)value * (long)(jointRange[i][j][1] - jointRange[i][j][0]))/(long)(MAX_POT - MIN_POT);
//    return  ((long)value * (long)(jointRange[i][j][1] - jointRange[i][j][0]))/(long)(MAX_POT - MIN_POT);
}


static int16_t clamp(int16_t value, int16_t min, int16_t max) {
  if (value > max) 
    return max;
  if (value < min)
    return min;
  return value;
    //return (value > max) ? max : (value < min ? min : value);
}

// Send target position to sculpture
static void targetSculpture(int positionArray[NUM_TOWERS][NUM_JOINTS]) {
    char modelString[256];
    char smallModelString[64];
    char *ptr = modelString;
    for (int i=0; i<NUM_TOWERS; i++){
        for (int j=0; j<NUM_JOINTS; j++) {
            sprintf(ptr, "<%d%dt%d>", i+1, j+1, positionArray[i][j]);
            ptr += strlen(ptr);
            if (i==0 && j==1) {
              sprintf(smallModelString, "<%d%dt%d>\r\n", i+1, j+1, positionArray[i][j]);
              Serial.print(smallModelString);
            }
        }
    }
    sprintf(ptr, "\r\n");
//    Serial1.print(modelString);
    Serial1.print(smallModelString);

 //   if (debug) {
 //     Serial.println(modelString);
 //   }
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
            sprintf(ptr, "%d,", modelPosition[i][j]);
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

uint8_t cmd_str[MAX_COMMAND_LENGTH];
uint8_t cmd_len = 0;

static void readSerial()
{
  //Serial.println("Sanity test...");
  while(Serial.available() > 0) {
    int haveCommand = FALSE;
    //uint8_t serialChar = Serial.read();
    haveCommand = accumulateCommandString(Serial.read());
    //Serial.write(serialChar);
    if (haveCommand) {
      Serial.println("Have command!!");
      parseCommand();
    }
    Serial.println("Read a char");
  }
}

static void readSerial1() {
/*  int nAvailable = Serial1.available();
  if (nAvailable > 0) {
    char strBuf[32];
    sprintf(strBuf, "Have %d bytes on Serial1\n\r", nAvailable);
    Serial.print(strBuf); 
  }*/
//  while (Serial1.available() > 0) {
//    Serial.print(Serial1.read()); // just echo for the moment
//  }
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



static void parseCommand() 
{
  //int16_t intData=0;    /* holds numerical value of parsed data */
  int8_t towerId=0;    /* tower we're working with, if there is one */
  int8_t jointId=0;    /* joint we're working with, if necessary */
  uint8_t charPos=1;    /* start with first char past the "<" */
  uint8_t c;            /* next char to parse */
  int val;
  uint8_t len = strlen(cmd_str);
  char *limitstr[3] = {NULL, NULL, NULL};
  uint16_t jtarget[3];  /* joint target data */
  uint16_t jcenter[3];  /* joint center data. Raw units from the potentiometer */
  int maxVal;
  int minVal;
  int centerVal;
  int idx = 0;
  int ntokens = 0;
  char strBuf[256];

  Serial.print("Received command\n");

  if (len < 3 ) {
    Serial.print("Command too short");
    return;
  }

  /* first character should be a command... */
  /* check that we're not going further than the command length!!!  XXX */
  c = cmd_str[charPos++];
  switch(c) {
    case 'L':  // set limits for a tower 
      /* next character should be a digit - tower 
         Note that this clause will catch EOL */

      c = cmd_str[charPos++];
      towerId = c -'0';
      if (towerId < 1 || towerId > 4) {
        Serial.println("cannot parse command string, unknown tower");
        return;
      } 
      towerId--; // 0 based indexing

      c = cmd_str[charPos++];
      jointId = c - '0';
      if (jointId < 1 || jointId > 3) {
        Serial.println("cannot parse command string, unknown joint");
        return;
      }
      jointId--; // 0 based indexing
      
      ntokens = sscanf ((char *)&cmd_str[charPos],"%d,%d,%d>",&minVal, &maxVal, &centerVal);

      if (ntokens < 3) {
        Serial.println("not enough values in command");
        return;
      }

      jointRange[towerId][jointId][0] = minVal;
      jointRange[towerId][jointId][1] = maxVal;
      jointRange[towerId][jointId][2] = centerVal;


      if (debug) {
        sprintf(strBuf, "Tower %d joint %d set to min: %d, max: %d, center: %d\n", towerId+1, jointId+1, minVal, maxVal, centerVal);
        Serial.print(strBuf);
      }
      
      break;
    case 'd':  // set debug level
      c = cmd_str[charPos++];
      debug = c - '0';
      if (debug) {
        sprintf(strBuf, "Setting debug level to %d\n", debug);
        Serial.print(strBuf);
      }
      break;
    case 'm':  // set mode
      c = cmd_str[charPos++];
      val = c - '0';
      Serial.print("Setting mode to ");
      Serial.println(val);
      modeIsLocal = FALSE;
      switch(val) {
        case 0:
            mode = MODE_OFF;
            break;
        case 1:
            mode = MODE_FREE_PLAY;
            break;
        case 2:
            mode = MODE_POSE;
            break;
        case 3:
            mode = MODE_SLAVE;
            break;
        default:
            modeIsLocal = TRUE;
            Serial.println("Restoring mode control to switch");
            break;
      }

      if (mode != MODE_SLAVE) {
        slaveValidBitmask = 0x00;
      }
      if (mode != MODE_POSE) {
        poseValidBitmask = 0x00;
      }

      if (!modeIsLocal) {
        if (debug ) {
          sprintf(strBuf, "Mode set to %s\n", modeStr[mode]);
          Serial.print(strBuf);
        }     
        sprintf(strBuf, "{'mode': %d}\n", mode);
        Serial.print(strBuf);
      }
      
      break;
    case 's': // Get status
        sprintf(strBuf, "{'mode': %d, [", mode);
        for (int i=0; i<4; i++) {
            char towerStr[128];
            sprintf(towerStr, "{'tower': %d, 'jointPos' : [%d, %d, %d ], 'jointCenter': [%d, %d, %d]},",
                                i, modelPosition[i][0], modelPosition[i][1], modelPosition[i][2],
                                jointCenter[i][0], jointCenter[i][1], jointCenter[i][2]);
            strcat(strBuf, towerStr);
        }
        strBuf[strlen(strBuf)-1] = '\0'; // remove trailing ','
        strcat(strBuf, "]}");
        Serial.println(strBuf);
        break;
    case 'P':  // set slave (pose) data
        ntokens = sscanf ((char *)&cmd_str[charPos],"t%d:%d,%d,%d", &towerId, &jtarget[0], &jtarget[1], &jtarget[2]); 
        if (ntokens != 4 || towerId > 4 || towerId < 1) {
          Serial.println("Invalid tower");
          break;
        }
        towerId--; // 0 based idx
        for (int i=0; i<3; i++) {
          slavePosition[towerId][i] = clamp(jtarget[i], jointRange[towerId][i][0], jointRange[towerId][i][1]);
          sprintf(strBuf, "Clamping %d between %d and %d, result %d\n", jtarget[i], jointRange[towerId][i][0], 
                                                                                    jointRange[towerId][i][1], 
                                                                                    slavePosition[towerId][i]);
          Serial.print(strBuf);
        } 
        slaveValidBitmask |= (1 << towerId);

        if (debug) {
           sprintf(strBuf, "Set pose data for tower %d to %d, %d, %d\n", towerId+1, slavePosition[towerId][0], 
                                                                                    slavePosition[towerId][1],
                                                                                    slavePosition[towerId][2]);
           Serial.print(strBuf);
           sprintf(strBuf, "Slave bitmask is %x\n", slaveValidBitmask);
           Serial.print(strBuf);
        }
        break;  
    case 'c': // Set center of joint.
        ntokens = sscanf((char *)&cmd_str[charPos], "%d:%d,%d,%d", &towerId, &jcenter[0], &jcenter[1], &jcenter[2]);  
        if (ntokens != 4 || towerId > 4 || towerId < 1) {
          Serial.println("Invalid tower");
          break;
        }

        if (jcenter[0] > POT_RANGE || jcenter[0] < 0) {
            Serial.print("Joint 1 center out of range\n");
            break;
        } else if (jcenter[1] > POT_RANGE || jcenter[1] < 0) {
            Serial.print("Joint 2 center out of range\n");
            break;
        } else if (jcenter[2] > POT_RANGE || jcenter[2] < 0) {
            Serial.print("Joint 3 center out of range\n");
            break;
        } 
        
       towerId--;
       for (int i=0; i<NUM_JOINTS; i++) {
          jointCenter[towerId][i] = jcenter[i];   
       }
       break;
       
    default:
      break;
  }
}


// CAN stuff here...
// Build an ID or PGN

 long unsigned int txID = 0x1881ABBA;// This format is typical of a 29 bit identifier.. the most significant digit is never greater than one.
unsigned char stmp[8] = {0x0E, 0x00, 0xFF, 0x22, 0xE9, 0xFA, 0xDD, 0x51};

//Construct a MCP_CAN Object and set Chip Select to 53.

MCP_CAN CAN(SPI_CS_PIN);                            


void CAN_init()
{
    //Serial.begin(115200);

    while (CAN_OK != CAN.begin(CAN_250KBPS))              // init can bus : baudrate = 250K
    {
        Serial.println("CAN BUS Module Failed to Initialized");
        Serial.println("Retrying....");
        delay(200); 
    }
    Serial.println("CAN BUS Shield init ok!");
}


void CAN_sendData(unsigned char *data, unsigned char len)
{   
    if (len > 8) {
      Serial.println("Bad CAN data len");
      return;
    }
    
    Serial.println("sending CAN data");

    // send the data:  id = 0x00, Extended Frame, data len = 8, stmp: data buf
    // Extended Frame = 1.
    
    unsigned char ret = CAN.sendMsgBuf(txID,1, len, data);
    if (ret == CAN_FAILTX) {
      Serial.println("Failed sending CAN data");
    }
    //delay(25);    // send data every 25mS
}


