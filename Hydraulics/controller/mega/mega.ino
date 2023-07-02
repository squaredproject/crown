#include <mcp_can_dfs.h>
//#include <mcp_can.h>
#include<avr/wdt.h>

#include <SPI.h>
#include <EEPROM.h>
#include "UART1.h"
#include "UART2.h"
#include "UARTbaudrates.h"

#include "CAN_message.h"
#include "mcp_can_wrapper.h"


// Basic Architecture:
// The Controller acts as the main point of contact to the Crown
// sculpture. There are multiple possible inputs - the maquette, the
// conductor (manipulating sine waves), or playback of recorded data -
// and the Controller is responsible for deciding which of these
// inputs to use, filtering the input for safety, and input data on
// to the sculpture.
//
// There are two separate buses that communicate with the sculpture. 
// Simple input commands go over a 485 bus. Status queries go over 
// a CAN bus. (The CAN bus was layered on top of an existing 485 bus.
// The 485 bus operates in a single sender, multiple receivers mode, which
// makes it inappropriate for request-response communication).
//
// Internally, the controller uses two separate SBCs: an Arduino Mega
// and a Raspberry Pi. The Arduino Mega handles the buses and the input
// selection. The Raspberry Pi provides a web interface for humans.

// CSW 4/27/23


// FIXME Issues when pulling apart maquette and controller:
// - Who controls what eeprom. Information on which tower or joint of the
// maquette is disabled is clearly a maquette function, but am I also using
// this for sculpture tower enable/disable? If not, how do I know which
// towers and joints are currently enabled and disabled? And shouldn't this
// information be stored in the controller?

// Note on the serial protocol:
// The serial protocol on both the RS485 line (to the driver at the sculpture arm)
// and on the serial line (to the higher level Raspberry Pi) takes the following 
// form:
// [m][tower_id][joint_id]<command>[args...]
// An 'm' at the beginning means that this command is explicitly for the maquette,
// and should not be sent to the sculpture driver.
// 'tower_id' is mandatory for driver commands (not maquette commands). Since each
// driver handles a single tower, the 'tower_id' is effectively the address on the
// 485 bus. 'joint_id' may or may not be optional, depending on the command.
// And 'command' is a single alpha character...
//
// If I were designing the system from scratch, this is not how I would specify the
// protocol. I'm basically layering the arduino and raspberry pi on top of an 
// existing system that just sent oscillating wave commands to the towers. I do not
// want to break that system because it operates as a backup control system while
// we get the kinks out of the new control system. 

#define NUM_TOWERS 4
#define NUM_JOINTS 3
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


typedef enum {
   MODE_OFF = 0,
   MODE_MAQUETTE = 1,
   MODE_CONDUCTOR = 2,
   MODE_MANUAL = 3,
} ControllerMode;
ControllerMode mode = MODE_OFF;
static const char *modeNames[] = {"OFF", "MAQUETTE", "CONDUCTOR", "MANUAL"};

// Block of calibration data
// Joint enabled is whether the joint is currently enabled.
// It is a uint8_t (bool), and there are 12 of them (4 towers, 3 joints per tower).
// Tower enabled is whether the tower is currently enabled. 
// It is a uint8_t (boo), and there are 4 of them (one for each tower).
// Maquette joint center is the center of the joint on the maquette, as read by the potentiometer.
// It is a uint16_t, and there are 12 of them (4 towers, 3 joints per tower).
// Maquette Tower Center Calibrated is whether all the joint centers on each
// tower are considered correctly calibrated.
// It is a uint8_t, and there are 4 of them.
#define EEPROM_CAL_DATE_OFFSET 0
#define EEPROM_JOINT_ENABLED_OFFSET (EEPROM_CAL_DATE_OFFSET + sizeof(uint32_t))
#define EEPROM_TOWER_ENABLED_OFFSET (EEPROM_JOINT_ENABLED_OFFSET + 12*sizeof(uint8_t))

// Enable/disable towers and joints
// Position commands will not be sent to any tower/joint that is disabled
int towerEnabled[NUM_TOWERS] = {TRUE, TRUE, TRUE, TRUE};
int jointEnabled[NUM_TOWERS][NUM_JOINTS] = {{TRUE, TRUE, TRUE},
                                          {TRUE, TRUE, TRUE},
                                          {TRUE, TRUE, TRUE},
                                          {TRUE, TRUE, TRUE}};


// Joint range is the valid offset *ON THE SCULPTURE*, relative to the joint center, for the motion. 
// This *should* be relatively stable - we shouldn't have to do the homing dance if we're careful.
// XXX - Note that this information, despite being needed to do the maquette to sculpture
// translation, is not actually set. (We can get it by asking the drivers. We just don't do
// that as a matter of course, and we do not save the values) FIXME
int16_t jointRange[NUM_TOWERS][NUM_JOINTS][2] = {{{-300, 200}, {-532, 328}, {-275, 276}},
                                                  {{-300, 520}, {-532, 288}, {-275, 296}},
                                                  {{-200, 200}, {-200, 200}, {-200, 200}},
                                                  {{-300, 520}, {-532, 288}, {-275, 296}}};




static int curJointTargets[NUM_TOWERS][NUM_JOINTS] = { {0,0,0},
                                             {0,0,0},
                                             {0,0,0},
                                             {0,0,0}};


static bool serialBroadcast = true;  // whether we broadcast the position data back to the rpi

// Switch Mode  API

// Pin definitions
#define SWITCH_CONDUCTOR_PIN 38
#define SWITCH_MAQUETTE_PIN 40
#define SWITCH_MANUAL_PIN 36
#define SWITCH_OFF_PIN      42   // XXX FIXME NO PHYSICAL ATTACHMENT!!

// Physical switch controlling mode
static void SwitchInit();
static int SwitchGetMode();
static void setModeFromSwitch();
static const char *modeStr[] = {"OFF", "IMMEDIATE", "POSE", "MANUAL"};


// Reading commands and parsing bus commands
#define MAX_COMMAND_LENGTH 32
typedef struct {
    uint8_t cmd_len;
    uint8_t cmd_str[MAX_COMMAND_LENGTH];
} CommandBuf;

CommandBuf serialCmd  = {.cmd_len=0};
CommandBuf rs485Cmd  = {.cmd_len=0};

static uint8_t accumulateCommandString(uint8_t c, CommandBuf *cbuf);
static void parseSerialCommand();
static void parseTowerCommand(char *buf, int len);
static void parseLocalCommand(char *buf, int len);

#define NETWORKFILE   &uart1file   // 485 bus output to the sculpture
static void Write485(char *buf);
// static void broadcastModelPosition();

static void readCAN();
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
static void HandleCANHomingResult(uint8_t towerId, uint8_t jointId, uint8_t *buf);

void log_info(char *str);
void log_error(char *str);
void log_debug(char *str);

#define DBG_LEVEL_DEBUG 3
#define DBG_LEVEL_INFO 2
#define DBG_LEVEL_ERROR 1
static const char *debugString[] = {"OFF", "ERROR", "INFO", "DEBUG"};

int debug = 1; // DBG_LEVEL_DEBUG;  // debug on/off

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

// There are two different configuration variables that we store in EEPROM. These are
// towerEnabled - whether the specified tower should be sent commands or not.
// jointEnabled - whether the specified joint should be sent commands or not.
// XXX - This was true for the maquette, where we want to be able to zero out
// misbehaving input data. Is it true for the tower, or does the responsibiity more
// properly reside in the driver at the tower?? FIXME

short EEPROMReadShort(int eepromOffset)
{
    byte first = EEPROM.read(eepromOffset);
    byte second = EEPROM.read(eepromOffset + 1);
    return ((short)second << 8) | first;
}

// XXX - I do not know if the tower itself is calibrated. This I can only know from the CANbus... 
static void debugCalibrationData(int tower)
{
    char buf[128];
    sprintf(buf, "Tower %d Status Data: \r\n ", tower);
    Serial.print(buf);
    sprintf(buf, "  Joint Enabled: %d, %d, %d\r\n", jointEnabled[tower][0], jointEnabled[tower][1], jointEnabled[tower][2]);
    Serial.print(buf);
    Serial.print(buf);
    sprintf(buf, "  Tower Enabled: %d\r\n", towerEnabled[tower]);
    Serial.print(buf); 
}

void printCalibrationData()
{
    // Print the first 256 hex characters of EEPROM
    char buf[128];
    int eeprom_offset = 0;
    for (int i=0; i<16; i++) {
        sprintf(buf, "0x%x 0x%x 0x%x 0x%x  0x%x 0x%x 0x%x 0x%x  0x%x 0x%x 0x%x 0x%x  0x%x 0x%x 0x0%x 0x%x", 
                      EEPROM.read(eeprom_offset),
                      EEPROM.read(eeprom_offset+1),
                      EEPROM.read(eeprom_offset+2),
                      EEPROM.read(eeprom_offset+3),
                      EEPROM.read(eeprom_offset+4),
                      EEPROM.read(eeprom_offset+5),
                      EEPROM.read(eeprom_offset+6),
                      EEPROM.read(eeprom_offset+7),
                      EEPROM.read(eeprom_offset+8),
                      EEPROM.read(eeprom_offset+9),
                      EEPROM.read(eeprom_offset+10),
                      EEPROM.read(eeprom_offset+11),
                      EEPROM.read(eeprom_offset+12),
                      EEPROM.read(eeprom_offset+13),
                      EEPROM.read(eeprom_offset+14),
                      EEPROM.read(eeprom_offset+15));
        Serial.print(buf);
        Serial.println("");
        eeprom_offset += 16;
    }
}

void readCalibrationData() 
{
    char buf[256];
    Serial.println("Reading calibration data from EEPROM");
    for (int i=0; i<NUM_TOWERS; i++) {
        for (int j=0; j<NUM_JOINTS; j++) {
            jointEnabled[i][j] = EEPROM.read(EEPROM_JOINT_ENABLED_OFFSET + i*sizeof(uint8_t)*NUM_JOINTS + j*sizeof(uint8_t));
        }
        towerEnabled[i] = EEPROM.read(EEPROM_TOWER_ENABLED_OFFSET + i*sizeof(uint8_t));
        debugCalibrationData(i);
    }
}

/* 
 * writeTowerCalibration
 * Save calibration information (4 fields) for the specified tower
*/
void writeTowerCalibrationData(int tower)
{
    for (int j=0; j<NUM_JOINTS; j++) {
        EEPROM.put(EEPROM_JOINT_ENABLED_OFFSET + tower*sizeof(uint8_t)*NUM_JOINTS + j*sizeof(uint8_t), jointEnabled[tower][j]);
    }
    Serial.print("Writing tower enabled ");
    Serial.print(towerEnabled[tower]);
    Serial.print(" to offset ");
    Serial.println(EEPROM_TOWER_ENABLED_OFFSET + tower*sizeof(uint8_t));
    EEPROM.put(EEPROM_TOWER_ENABLED_OFFSET + tower*sizeof(uint8_t), towerEnabled[tower]);
   
    Serial.println("Wrote Tower Calibration Data");
    debugCalibrationData(tower);
}

/*
 * writeCalibrationdata
 * Save calibration data (4 fields) for all four towers
 */
void writeCalibrationData()
{
    for (int i=0; i<NUM_TOWERS; i++) {
        writeTowerCalibrationData(i);
    }
}



// Main Mode Switch
// There is a physical mode switch on the front of the controller box. Hopefully
// it will not be ripped off when people physically throw the box in a trunk at
// the end of a run.
// The switch selects between three input modes - one in which the conductor 
// provides the input, one in which the maquette provides the input, and one
// in which the playback system is providing the input. Off mode (which needs to
// be implemented Real Soon Now) prevents standard inputs from being sent to
// the hydraulics, although the user can still calibrate the towers.


static void SwitchInit()
{
    Serial.println("Initializing all pins to INPUT_PULLUP");
    pinMode(SWITCH_CONDUCTOR_PIN, INPUT_PULLUP);
    pinMode(SWITCH_MAQUETTE_PIN, INPUT_PULLUP);
    pinMode(SWITCH_MANUAL_PIN, INPUT_PULLUP);
    pinMode(SWITCH_OFF_PIN,      INPUT_PULLUP);
}

static int SwitchGetMode()
{
    // Note that the pins are pulled high, so a low value is a connected switch 
    if (!digitalRead(SWITCH_OFF_PIN)) return MODE_OFF;
    if (!digitalRead(SWITCH_CONDUCTOR_PIN)) return MODE_CONDUCTOR;
    if (!digitalRead(SWITCH_MAQUETTE_PIN)) return MODE_MAQUETTE;
    if (!digitalRead(SWITCH_MANUAL_PIN)) return MODE_MANUAL;

    return MODE_OFF;
}

static void setModeFromSwitch()
{
    int newMode = SwitchGetMode();
    if (newMode != mode) {
        Serial.print("Switch value changed, ");
        Serial.print(" New value: ");
        Serial.println(modeNames[newMode]);
        mode = newMode;
        fprintf(NETWORKFILE, "Switch value changed to : %s\n", modeNames[newMode]);
        // XXX - invalidate/reset buffers/whatever all partial position information from playback/conductor/maquette.
    }
}

void setup(){
  Serial.begin(115200);
  UART1_Init(UART_115200);  // Sender - 485 to sculpture
  UART2_Init(UART_115200);  // Receiver - Conductor to controller
  wdt_disable();  // Disable watchdog
  delay(100);
  SwitchInit();
  CAN_init();  // XXX - CAN is not yet in the controller box
  Serial.println("Starting Controller...");
  fprintf(NETWORKFILE, "Starting Controller...\n");
  wdt_enable(WDTO_8S);

}



// Safety module - flag unsafe actions. Only one tower can be targeted for the red
// zone at a time.
// XXX - Ideally, this would depend on the current state, rather than the current
// target state. But in order to do that, I need to know the current state.
// XXX - get values for R1, R2, and R3; convert between canonical position and
// joint angle
// Getting theta from position information. Which I don't completely understand
static int targetRedZoneTower = -1;
#define ENCODER_RANGE 2048  // XXX - guessing, fixme
#define PI 3.14159265f
#define R1 72 
#define R2 60
#define R3 60
#define RED_ZONE_EDGE 100
// XXX - should do the same sine table trick that the conductor is doing.
//  FIXME
static float curJointAngles[NUM_TOWERS][NUM_JOINTS] = {{0.0, 0.0, 0.0},
                                              {0.0, 0.0, 0.0},
                                              {0.0, 0.0, 0.0}};
static bool positionInRedZone(int towerId, int jointId, int position)
{
  // pos1, 2, and 3 are in units of the rotary encoder, which for the moment I'm going to claim
  // is 4096 based
  float theta1 = curJointAngles[towerId][jointId];
  float theta2 = curJointAngles[towerId][jointId];
  float theta3 = curJointAngles[towerId][jointId];

  return R1*sin(theta1) + R2*sin(theta1+theta2) + R3*sin(theta1+theta2+theta3) > RED_ZONE_EDGE;
}

static float rawPositionToAngle(int raw) {
    return (raw * PI)/ENCODER_RANGE;
}

#define INVALID_TOWER (-1)
static int towerIdInRedZone = INVALID_TOWER;

// String is format: <TJtXXXXX>
// where T is the tower ID
// J is the joint ID
// t is the command to set a position
// XXXXX is a number
// Tried using sscanf but it's not very stable. DOing it by hand
// is more the arduino style

static bool positionIsSafe(const char *cmd_str) {

    bool positionOkay = true;

    return positionOkay;

    // WARNING: tower and joint on the wire are 1 to N, but we want zero
    // index in this spot. 
    int towerId = (int) cmd_str[1] - '1';
    int jointId = (int) cmd_str[2] - '1';
    int position = 0;
    int spos=4;
    bool negative = false;
    while ( spos < 32 ) {
        char c = cmd_str[spos++];
        if (c == '-') {
            negative = true;
            continue;
        }
        if ((c >= '0') && (c <= '9')) {
            position = (position * 10) + (c - '0');
            continue;
        }
        if (c == '>') {
            break;
        }
        if (c == 0) {
            return(false);
        }
    }
    if (spos == 32) {
        return(false);
    }
    if (negative) position = -1 * position;

    //Serial.println(" isSafe ");
    //Serial.println(towerId);
    //Serial.println(jointId);
    //Serial.println(position);
            
    if (towerId < 0 || towerId >= NUM_TOWERS) {
        return false;
    }
    if (jointId < 0 || jointId >= NUM_JOINTS) {
        return false;
    }

    if (positionInRedZone(towerId, jointId, position)) {
        if (towerIdInRedZone == INVALID_TOWER) {
            towerIdInRedZone = towerId;
            positionOkay = true;
        } else if (towerIdInRedZone != towerId) {
            positionOkay = false;
        }
    } else {
        if (towerId == towerIdInRedZone) {
            towerIdInRedZone = INVALID_TOWER;
            positionOkay = true;
        }
    }

    if (positionOkay) {
        curJointTargets[towerId][jointId] = position;
        curJointAngles[towerId][jointId] = rawPositionToAngle(position);
    }

    return positionOkay;
}

static void clearCommandString(CommandBuf *cmd)
{
    cmd->cmd_len = 0;
}

static void sendPosition(const char *cmd)
{
    if (serialBroadcast) {
        Serial.print(cmd);
    }
    Write485(cmd);
}

static void handleConductorInput() {
    // Parser for data on the 485 bus
    // The only commands we accept from this input source are
    // tower positions. If we're in CONDUCTOR mode (ie, the sculpture is being
    // controlled by the conductor), we check whether the target position
    // is safe. If it is, we both save the target position as part of
    // tracked sculpture state, and send the data along on the other 485
    // bus. If we're not in CONDUCTOR mode, we drop the data on the floor.
    if (UART2_data_in_ring_buf()) { // check for waiting UART data from SPU
      uint8_t cData;
      cData = UART2_ring_buf_byte(); // get next char from ring buffer...
      if (accumulateCommandString(cData, &rs485Cmd)) { // ... and add to command string
        // Serial.print("Have a command on the 485 port!:\r\n  ");
        // Serial.println((char *)&rs485Cmd.cmd_str[0]);
        if (mode != MODE_CONDUCTOR) {
            clearCommandString(&rs485Cmd);
        } else {
            // NB. cmd_len is 0 here because it's fully acumulated
            int le = strlen(rs485Cmd.cmd_str);
            if ((le >=6) && (le <=12) && (rs485Cmd.cmd_str[0] == '<') && (rs485Cmd.cmd_str[3] == 't')) {
                if (positionIsSafe(rs485Cmd.cmd_str)) {
                    sendPosition(rs485Cmd.cmd_str);
                }
                else {
                     Serial.println(" ignoreCmd: position not safe ");
                }
            } else {
                if (le < 6) {
                    Serial.print(" ignoreCmd: len short should be 6 or more ");
                    Serial.println(le);
                } else if (le > 12) {
                    Serial.print(" ignoreCmd: len long should be 12 or less ");
                    Serial.println(le);
                } else if (rs485Cmd.cmd_str[3] != 't') {
                    Serial.print(" ignoreCmd: char 3 not t ");
                    Serial.println(rs485Cmd.cmd_str[3]);
                } else {
                    Serial.println(" ignoreCmd: not sure why ");
                }
            }
         }
      }
   }
}


void loop() {
    
    unsigned long curTime = millis();

    handleSerialCommand();  // Includes position data from playback and maquette 
    handleConductorInput();
    readCAN();  // XXX - CAN module not physically hooked up yet
    setModeFromSwitch();
    wdt_reset();  // Pat watchdog
}


/*
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
*/


static void handleSerialCommand()
{
  //Serial.println("Sanity test...");
  while(Serial.available() > 0) {
    int haveCommand = FALSE;
    haveCommand = accumulateCommandString(Serial.read(), &serialCmd);
    if (haveCommand) {
      // log_debug("Have command");
      parseSerialCommand();
    }
  }
}


static uint8_t accumulateCommandString(uint8_t c, CommandBuf *cbuf)
{
  // if we were at full and get another character, it's time to 
  // if its the end of a string, might need the character and one more the null
  if (cbuf->cmd_len >= MAX_COMMAND_LENGTH - 2) {
    cbuf->cmd_len = 0;
  }

  /* catch beginning of this string */
  if (c == '<') { // this will catch re-starts and stalls as well as valid commands.
     cbuf->cmd_len = 1;
     cbuf->cmd_str[0] = '<';
     return 0;
  }
  
  if (cbuf->cmd_len != 0) {   // string in progress, accumulate next char
    cbuf->cmd_str[cbuf->cmd_len++] = c;
      
    if (c == '>') {
        //char buf[128];
        //memcpy(buf, cmd_str, cmd_len);
        //buf[cmd_len] = '\0';
        cbuf->cmd_str[cbuf->cmd_len] = '\0';
        cbuf->cmd_len = 0;
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
  uint8_t *cmd_str = serialCmd.cmd_str;
  int len = strlen(cmd_str);

  log_debug("Received command");

  if (len < 3 ) {
    log_error("Command too short");
    return;
  }

  /* first character after the '<' is either a command to a tower, or an 'm', signifying that this
     is a command to the mega board itself */
  c = cmd_str[1];  
  if (c == 'm') {
    parseLocalCommand(cmd_str+2, len-2); // skip '<m'
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


static void parseLocalCommand(char *buf, int len) {
    int maxVal;
    int minVal;
    int centerVal;
    char c;
    char *ptr = buf;
    char *bufEnd = buf + len;
    uint8_t val;
    char outBuf[512];
    uint8_t towerId;
    uint8_t jointId;
    uint16_t jtarget[3];  /* joint target data */
    uint16_t jcenter[3];  /* joint center data. Raw units from the potentiometer */
    int ntokens;
    
    if (bufEnd <= buf) return;
    
    c = *ptr++;
   
    switch (c) {
    case 'D':  // set debug level
      Serial.print("Setting debug level to ");
      Serial.println(*(ptr+1)); 
      ptr++; // NB - skipping ':', see note about parsing better globally.  XXX - test that this actually works from the website
      c = *ptr++;
      if (((c - '0') >= 0) && ((c - '0') <= 3)) {
        debug = c - '0';
        sprintf(outBuf, "Setting debug level to %s\n", debugString[debug]);
        Serial.println(outBuf);
      } else {
        sprintf(outBuf, "Unknown debug value %d\n", c - '0');
        Serial.println(outBuf);
      }
      Serial.println("<!mD+>");
      break; 
    case 's': // Get status
        Serial.println("Getting status");
        // Note that I'm being very careful here not to overrun outBuf, which can't be very big because we have a small stack.
        // sprintf(outBuf, "<!ms{\"mode\": \"%s\", \"towerState\": ", modeStr[mode]);
        Serial.print(outBuf);
        for (int i=0; i<NUM_TOWERS; i++) {
            sprintf(outBuf, " [\"towerEnabled\":%s, \"jointEnabled\" : [%s, %s, %s]],",
                                (towerEnabled[i] ? "true" : "false"),
                                (jointEnabled[i][0] ? "true" : "false"),
                                (jointEnabled[i][1] ? "true" : "false"),
                                (jointEnabled[i][2] ? "true" : "false"));
                                
            if (i==3)
                outBuf[strlen(outBuf)-1] = '\0'; // remove trailing ','
            Serial.print(outBuf);    
        }
        Serial.println("}>");
        break;
    case 'i':  // Get input 
        sprintf(outBuf, "<!mi{\"mode\": \"%s\"}>", modeNames[mode]);
        Serial.print(outBuf);
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
      towerEnabled[towerId-1] = val;
      writeCalibrationData();
      Serial.print("<!mE");
      Serial.print(towerId);
      Serial.println("+>"); // XXX fixme better return codes
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
      jointEnabled[towerId-1][jointId-1] = val;
      writeCalibrationData();
      Serial.print("<!me");
      Serial.print(towerId);
      Serial.println("+>"); // XXX fixme above
      break;
        
    default:
      Serial.print("Unknown command ");
      Serial.println(c);
      break; 
    }
    Serial.println("Finish parse maquette command");
} 

static void parseTowerCommand(char *buf, int len) {
    // For the most part, we don't parse tower commands. We just send them on
    // WARNING: towerId, jointId is 1 indexed here, 0 indexed elsewhere
    uint8_t towerId = buf[1] - '0';
    uint8_t jointId;
    char secondChar = buf[2];
    char command;
    int cmd_idx;
    int ret;
    char outBuf[64];
    
    if (secondChar <= '3' && secondChar >= '0') {
        // joint id. Next is command
        if (len < 4) {
            log_error("Tower command too short!");
            return;
        }
        jointId = secondChar-'0';
        cmd_idx = 3;
    } else {
        cmd_idx = 2;
    }

    command = buf[cmd_idx];
    
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
        // Set tower position. This may or may not pass, depending on mode
        if (mode == MODE_MAQUETTE || mode == MODE_MANUAL) {
            if (positionIsSafe(buf)) {
                sendPosition(buf);
            }
        }
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
    case 'z':  // XXX - this is 'e' at the driver level...
        // clear error condition
        Write485(buf);
        break;
    case 'E':
        // Software joint enable/disable
        Write485(buf);
        break;
    case 'F':
        // Set canonical joint position. Pass through
        Write485(buf);
        break;
    case 'f':
        // Set tower canonical position (3 joints). Pass through
        Write485(buf);
        break;
    default:
      Serial.println("Unknown command to maquette");
    }
}
        
static void Write485(char *buf)
{
    Serial.println(buf);
    fprintf(NETWORKFILE, "%s\n", buf);
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
          char buf[64];
          snprintf(buf, sizeof(buf), "Invalid CAN message, type %d\n", buf[CAN_CMD_BYTE]);
          log_error(buf);
        }
        return;
    }

}



#define MAX_SERIAL_BUF 256

// Translate CAN messages into a friendlier format, send them on... 

static void HandleCANPosition(uint8_t towerId, uint8_t *buf) 
{
    CAN_Position pos;
    
    CAN_BufferToPosition(buf, &pos);

    uint8_t serialBuf[MAX_SERIAL_BUF];
    snprintf(serialBuf, sizeof(serialBuf), "<!T%d[%d,%d,%d]>", towerId, pos.joint1Pos, pos.joint2Pos, pos.joint3Pos);
    Serial.println((char *)serialBuf);
}

static void HandleCANJointLimits(uint8_t towerId, uint8_t jointId, uint8_t *buf)
{
    CAN_Limits limits;
    
    CAN_BufferToJointLimits(buf, &limits);

    uint8_t serialBuf[MAX_SERIAL_BUF];
    snprintf(serialBuf,sizeof(serialBuf), "<!l%d%d{\"min\":%d,\"max\":%d, \"center\": %d}>", 
                         towerId, jointId, 
                         limits.minPos, limits.maxPos, limits.centerPos);
    Serial.println((char *)serialBuf);
}

static void HandleCANGeneralStatus(uint8_t towerId, uint8_t *buf) 
{
    Serial.println("CAN status message received");

    CAN_StatusStruct canStatus;

    CAN_BufferToStatus(buf, &canStatus);
   
    uint8_t serialBuf[MAX_SERIAL_BUF]; 
    snprintf(serialBuf,sizeof(serialBuf), "<!s%d{\"homed\":[%s,%s,%s], \"switches\": [%d, %d, %d], \"enabled\": [%s, %s, %s], \"running\": %s,\"error\": %d}>", 
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
    CAN_Valves valves;

    CAN_BufferToValves(buf, &valves);

    uint8_t serialBuf[MAX_SERIAL_BUF];
    snprintf(serialBuf, sizeof(serialBuf), "<!v%d[%d, %d, %d]>", towerId, 
                                             valves.driveLevel[0],
                                             valves.driveLevel[1],
                                             valves.driveLevel[2]);

    Serial.println((char *)serialBuf);
}

static void HandleCANJointStatus(uint8_t towerId, uint8_t jointId, uint8_t *buf)
{
    CAN_JointStatus joint; 

    CAN_BufferToJointStatus(buf, &joint);

    uint8_t serialBuf[MAX_SERIAL_BUF];
    snprintf(serialBuf, sizeof(serialBuf), "<!j%d%d{\"jointId\":%d, \"pos\":%d, \"target\":%d, \"valve\":%d, \"homed\":%s, \"enabled\":%s, \"switches\":0x%x}>",
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
    CAN_Homing homing; 

    CAN_BufferToHomingStatus(buf, &homing);

    uint8_t serialBuf[MAX_SERIAL_BUF];
    snprintf(serialBuf, sizeof(serialBuf), "<!H%d%d{\"sw\":%d, \"pos\":%d, \"mask\":%d, \"stalls\":%d}>", 
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
    CAN_Homing_Result homing; 

    CAN_BufferToHomingResult(buf, &homing);

    uint8_t serialBuf[MAX_SERIAL_BUF];
    snprintf(serialBuf, sizeof(serialBuf), "<!H%d%d{\"homed\":%s, \"detail\":\"%s\", \"target\":%d}>", 
                      towerId, jointId, 
                      TF_STRING(homing.success), 
                      homingReason(homing.detail),
                      homing.target);
    Serial.println((char *)serialBuf);             
}



static void HandleCANExtendedStatus(uint8_t towerId, uint8_t *buf)
{
    CAN_ExtendedStatus status;

    CAN_BufferToExtendedStatus(buf, &status);

    uint8_t serialBuf[MAX_SERIAL_BUF];
    snprintf(serialBuf, sizeof(serialBuf), "<!e%d{\"addr\":%d,\"debug\":%d, \"h1\":%d, \"h2\":%d, \"h3\":%d}>",
              towerId,
              status.debugLevel, 
              status.homeSpeed[0], status.homeSpeed[1], status.homeSpeed[2]);
    Serial.println((char *)serialBuf);       
}


static void HandleCANPIDValues(uint8_t towerId, uint8_t *buf) 
{
    CAN_PIDValues pids;

    CAN_BufferToPIDValues(buf, &pids);

    uint8_t serialBuf[MAX_SERIAL_BUF];
    snprintf(serialBuf,sizeof(serialBuf), "<!p%d[{\"p\":%d,\"i\":%d},{\"p\":%d,\"i\":%d}, {\"p\":%d,\"i\":%d}]>",
              towerId,
              pids.Kp[0], pids.Ki[0], 
              pids.Kp[1], pids.Ki[1], 
              pids.Kp[2], pids.Ki[2]);
              
    Serial.println((char *)serialBuf);       
}

static void HandleCANIntegrators(uint8_t towerId, uint8_t *buf)
{
    CAN_Integrators integrators;

    CAN_BufferToIntegrators(buf, &integrators);

    uint8_t serialBuf[MAX_SERIAL_BUF];
    snprintf(serialBuf,sizeof(serialBuf), "<!i%d[%d, %d, %d]>",
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



