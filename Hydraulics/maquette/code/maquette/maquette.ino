#include <mcp_can_dfs.h>
//#include <mcp_can.h>
#include<avr/wdt.h>

#include <SPI.h>
#include <EEPROM.h>
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

#define MAQUETTE_STAND_ALONE
//const int SPI_CS_PIN = 53;

typedef enum {
   MODE_OFF = 0,
   MODE_IMMEDIATE = 1,
   MODE_POSE = 2,
   MODE_PLAYBACK = 3,
} Mode;
Mode mode = MODE_OFF;

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
#define EEPROM_MAQUETTE_JOINT_CENTER_OFFSET (EEPROM_TOWER_ENABLED_OFFSET + 4*sizeof(uint8_t))
#define EEPROM_MAQUETTE_TOWER_CENTER_CALIBRATED_OFFSET (EEPROM_MAQUETTE_JOINT_CENTER_OFFSET + 12*sizeof(uint16_t))

// Enable/disable towers and joints
// Position commands will not be sent to any tower/joint that is disabled
int towerEnabled[NUM_TOWERS] = {TRUE, TRUE, TRUE, TRUE};
int jointEnabled[NUM_TOWERS][NUM_JOINTS] = {{TRUE, TRUE, TRUE},
                                          {TRUE, TRUE, TRUE},
                                          {FALSE, FALSE, FALSE},
                                          {TRUE, TRUE, TRUE}};

// Whether we've centered the towers. If we don't have center information, we don't dare to move them.
uint8_t maquetteTowerCenterCalibrated[NUM_TOWERS] = {FALSE, FALSE, FALSE, FALSE};


// Joint range is the valid offset *ON THE SCULPTURE*, relative to the joint center, for the motion. 
// This *should* be relatively stable - we shouldn't have to do the homing dance if we're careful.
// XXX - Note that this information, despite being needed to do the maquette to sculpture
// translation, is not actually set. (We can get it by asking the drivers. We just don't do
// that as a matter of course, and we do not save the values) FIXME
int16_t jointRange[NUM_TOWERS][NUM_JOINTS][2] = {{{-300, 200}, {-532, 328}, {-275, 276}},
                                                  {{-300, 520}, {-532, 288}, {-275, 296}},
                                                  {{-200, 200}, {-200, 200}, {-200, 200}},
                                                  {{-300, 520}, {-532, 288}, {-275, 296}}};


// Maquette joint center is the value read by the potentiometer when the joint is in the
// center position. It should be stable across reboots, and is saved and restored in EEPROM
uint16_t maquetteJointCenter[NUM_TOWERS][NUM_JOINTS] = {{POT_RANGE/2,POT_RANGE/2,POT_RANGE/2}, 
                                                        {POT_RANGE/2, POT_RANGE/2, POT_RANGE/2},
                                                        {POT_RANGE/2, POT_RANGE/2, POT_RANGE/2},
                                                        {POT_RANGE/2, POT_RANGE/2, POT_RANGE/2}};

// Maquette jointRange. Again, relative to the maquette center. I'm believing here I can just 
// set the range and then forget about it - this may or may not be correct. XXX - Check this.
int16_t maquetteJointRange[NUM_TOWERS][NUM_JOINTS][2] = {{{-100, 100}, {-60, 60}, {-50, 50}},
                                                      {{-100, 100}, {-60, 60}, {-50, 50}},
                                                      {{-100, 100}, {-60, 60}, {-50, 50}},
                                                      {{-100, 100}, {-60, 60}, {-50, 50}}};

// Canonical position is in the range [-1.0, 1.0], where -1.0 is all the way out, and 1.0 is all the way
// in. 0 is the center. This is calculated on the maquette.
// XXX - moving to fixed point. [-256, 255]
// float canonicalJointPosition[NUM_TOWERS][NUM_JOINTS] = {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}};
int canonicalJointPosition[NUM_TOWERS][NUM_JOINTS] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};

// Model position - joint position of sculpture, if it matched maquette
int modelPosition[NUM_TOWERS][NUM_JOINTS]       = {{0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}};
// Static model position - similiar to above, except de-noised a bit 
int staticModelPosition[NUM_TOWERS][NUM_JOINTS] = {{0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}};

int pinMapping[NUM_TOWERS][NUM_JOINTS] = { {A0, A1, A3},
                                          {A4, A5, A7},
                                          {A8, A9, A11},
                                          {A12, A13, A15} };


// In 'Slave' mode, we're reading poses from some external source (such as 
// a pose list on the rpi) and adjusting the towers to match the current
// pose.
int slavePosition[NUM_TOWERS][NUM_JOINTS] = { {0,0,0},
                                                {0,0,0},
                                                {0,0,0},
                                                {0,0,0} };

uint8_t slaveValidBitmask = 0x00; 


typedef struct _Button {
  uint8_t externalState;
  uint8_t internalState;
  uint8_t justPressed;
  uint8_t justReleased;
  int settleTime;
  int pin;
} Button;

#define POSE_BUTTON_LED 13
#define POSE_BUTTON     12
#define POSE_LENGTH_MILLIS 10000

static Button poseButton;

// External button API..
static void ButtonInit(int pin, Button &button);
static void buttonRead(Button &button);
static bool buttonJustPressed(Button &button, bool consume);
static bool buttonJustReleased(Button &button, bool consume);

// External poseMode API
static void poseModeRunStateMachine();


// XXX - we've got a level violation here. The input to the mode API is a number
// but the output is a string. This means that number to string translations
// take place at both this level and the level above, which is problematic. FIXME
static const char *modeStr[] = {"OFF", "IMMEDIATE", "POSE", "PLAYBACK"};


static int16_t maquetteToModel(uint16_t pos, int i, int j);
static int16_t clamp(int16_t value, int16_t max, int16_t min);
static void setSculptureTargetPosition(int positionArray[NUM_TOWERS][NUM_JOINTS]);
static void broadcastModelPosition();
static void setModeFromSwitch();
static int slaveDataValid();
static void readCAN();
static uint8_t accumulateCommandString(uint8_t c);
static void parseSerialCommand();
static uint8_t maquetteCalibrated();
static int16_t readJointPos(int towerIdx, int jointIdx);

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
static void HandleCANHomingResult(uint8_t towerId, uint8_t jointId, uint8_t *buf);

void log_info(char *str);
void log_error(char *str);
void log_debug(char *str);

#define DBG_LEVEL_DEBUG 3
#define DBG_LEVEL_INFO 2
#define DBG_LEVEL_ERROR 1
static const char *debugString[] = {"OFF", "ERROR", "INFO", "DEBUG"};

int debug = 0;  // debug on/off
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

// There are four different configuration variables that we store in EEPROM. These are
// towerEnabled - whether the specified tower should be sent commands or not.
// jointEnabled - whether the specified joint should be sent commands or not.
// maquetteJointCenter - the center of the joint, as read by the potentiometer
// maquetteTowerCenterCalibrated - whether all the joint centers are valid and the tower is considered calibrated

short EEPROMReadShort(int eepromOffset)
{
    byte first = EEPROM.read(eepromOffset);
    byte second = EEPROM.read(eepromOffset + 1);
    return ((short)second << 8) | first;
}

static void debugCalibrationData(int tower)
{
    char buf[256];
    sprintf(buf, "Tower %d Maquette Calibration Data: \r\n ", tower);
    Serial.print(buf);
    sprintf(buf, "  Joint Enabled: %d, %d, %d\r\n", jointEnabled[tower][0], jointEnabled[tower][1], jointEnabled[tower][2]);
    Serial.print(buf);
    sprintf(buf, "  Joint Center: %d, %d, %d\r\n", maquetteJointCenter[tower][0], maquetteJointCenter[tower][1], maquetteJointCenter[tower][2]);
    Serial.print(buf);
    sprintf(buf, "  Tower Enabled: %d,  Tower Center Calibrated  %d\r\n", towerEnabled[tower], maquetteTowerCenterCalibrated[tower]);
    Serial.print(buf); 
}

void printCalibrationData()
{
    // Print the first 256 hex characters of EEPROM
    char buf[256];
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
            maquetteJointCenter[i][j] = EEPROMReadShort(EEPROM_MAQUETTE_JOINT_CENTER_OFFSET + i*sizeof(uint16_t)*NUM_JOINTS + j*sizeof(uint16_t));
        }
        towerEnabled[i] = EEPROM.read(EEPROM_TOWER_ENABLED_OFFSET + i*sizeof(uint8_t));
        maquetteTowerCenterCalibrated[i] = EEPROM.read(EEPROM_MAQUETTE_TOWER_CENTER_CALIBRATED_OFFSET + i*sizeof(uint8_t));
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
        EEPROM.put(EEPROM_MAQUETTE_JOINT_CENTER_OFFSET + tower*sizeof(uint16_t)*NUM_JOINTS + j*sizeof(uint16_t), maquetteJointCenter[tower][j]);
    }
    Serial.print("Writing tower enabled ");
    Serial.print(towerEnabled[tower]);
    Serial.print(" to offset ");
    Serial.println(EEPROM_TOWER_ENABLED_OFFSET + tower*sizeof(uint8_t));
    EEPROM.put(EEPROM_TOWER_ENABLED_OFFSET + tower*sizeof(uint8_t), towerEnabled[tower]);
    EEPROM.put(EEPROM_MAQUETTE_TOWER_CENTER_CALIBRATED_OFFSET + tower*sizeof(uint8_t), maquetteTowerCenterCalibrated[tower]);
   
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

void invalidateCalibration(int tower)
{
    EEPROM.put(EEPROM_MAQUETTE_TOWER_CENTER_CALIBRATED_OFFSET + tower, FALSE);
}


// 485 bus here...
#define NETWORKFILE   &uart1file

void setup(){
  Serial.begin(115200);
  UART1_Init(UART_115200);  // Network Baud out to nodes
  wdt_disable();  // Disable watchdog
  delay(100);
  ButtonInit(POSE_BUTTON, poseButton);
  InitLEDs();
  CAN_init();
  Serial.println("Starting maquette...");
  fprintf(NETWORKFILE, "Starting maquette...");
  wdt_enable(WDTO_2S);
  readCalibrationData();
  //pinMode(35, INPUT_PULLUP);
   // XXX - FIXME - Need to read the EEPROM for maquette calibration information,
  // as well as whether we've turned on/off joints towers etc.
}

int timerIdx = 0;
int secIdx = 0;
unsigned long mainloopTimeout = 0;
unsigned long poseTimeout = 0;

void loop() {
    char outBuf[4096];
    unsigned long curTime = millis();


//    secIdx++;
    buttonRead(poseButton);  
    readSerialCommand();
    readCAN();
    wdt_reset();  // Tell watchdog not to bark
    
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
            /* if (i == 0 && timerIdx%50 == 0) {
                Serial.print("Tower pot values: ");
            }*/
            for (int j=0; j<NUM_JOINTS; j++) {
                // For each joint, read the value from the maquette, and check whether that value
                // has changed more than the accepted delta from the previous position
                int16_t potValue  = readJointPos(i, j);
                canonicalJointPosition[i][j] = maquetteRawToCanonical((int16_t)potValue, i, j);
                modelPosition[i][j] = maquetteToModel(potValue, i, j);
                /* if (i == 0 && timerIdx%50 == 0) {
                    Serial.print(potValue);
                    Serial.print(" ");
                    Serial.print(int(canonicalJointPosition[i][j]));
                    Serial.print(" ");
                }*/
/*                if (j == 1) {
                  char buf[512];
                  sprintf(buf, "pot val: %d, center: %d, model: %d\n", potValue, maquetteJointCenter[0][1], modelPosition[0][1]);
                  Serial.print(buf);
                }
*/
                if ( towerEnabled[i] && jointEnabled[i][j] && 
                   ((modelPosition[i][j] - 20 > staticModelPosition[i][j]) || 
                    (modelPosition[i][j] + 20 < staticModelPosition[i][j]))) {
                    //char buf[256];
                    //sprintf(buf, "Have movement on tower %d joint %d, old %d, new %d, potValue %d \n", i, j, staticModelPosition[i][j], modelPosition[i][j], potValue);
                    //Serial.print(buf);
                    staticModelPosition[i][j] = modelPosition[i][j];
                }
            }
/*
            if (i==0 && timerIdx%50==0) {
                Serial.println("");
            }
*/
        }
   
    
        // What I do here depends on what mode I'm in.
        switch (mode) {
            case MODE_OFF:
                break;
            case MODE_IMMEDIATE:
                if (!maquetteCalibrated()) {
                    break;
                }
                // sendToSculpture(modelPosition);
                setSculptureTargetPosition(canonicalJointPosition);  // XXX de-noise?
                break;
            case MODE_POSE:
                if (!maquetteCalibrated())
                    break;  // Safety valve - do not send data from maquette if maquette is not calibrated.
                poseModeRunStateMachine();
        
// FIXME  And if we decalibrate/calibrate anything we immediately leave pose mode.
                break;
            case MODE_PLAYBACK:
                if (slaveDataValid() && maquetteCalibrated()) {
                    setSculptureTargetPosition(slavePosition);
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

static uint8_t maquetteCalibrated(void)
{
  uint8_t centerVal = TRUE;
  for (int i=0; i<NUM_TOWERS; i++){
    centerVal = centerVal && maquetteTowerCenterCalibrated[i];
  }

  return centerVal;
}


static int slaveDataValid() {
  return slaveValidBitmask == 0x0F;
}

static int16_t simulatedJointPos = 100;
static int16_t readJointPos(int towerIdx, int jointIdx) {
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


static void ButtonInit(int pin, Button &button)
{
    button.pin = pin; 
    button.externalState = 0;
    button.internalState = 0;
    button.settleTime = 0;
    button.justPressed = FALSE;
    button.justReleased = FALSE;
    pinMode(pin, INPUT_PULLUP);
}


static void buttonRead(Button &button)
{
  // NB - most of this code handles debouncing. The
  // button must read the same value for a certain period of time
  // before its externally facing state changes.

  int curState = digitalRead(button.pin);
  curState = 1-curState; // invert meaning, since we use a pull up

  // if the state is not the currently reported state of the button,
  // change either the externally reported state, or update
  // our settling counter
  if (curState != button.externalState) {
    if (curState == button.internalState) {
      button.settleTime++;
    } else {
      button.settleTime = 0;
      button.internalState = curState;
    }
    
    if (button.settleTime > SETTLE_TIME) {
      button.externalState = curState;
      button.settleTime = 0;
      if (curState) {
        button.justPressed = TRUE;
        button.justReleased = FALSE;
      } else {
        button.justPressed = FALSE;
        button.justReleased = TRUE;
      }
      if (debug) {
        char strBuf[256];
        sprintf(strBuf, "%s button %d\n", button.justPressed ? "PRESSED" : "RELEASED", button.pin);
        Serial.print(strBuf);
      }
    }
  }
}

static bool buttonJustPressed(Button &button, bool consume)
{
    int ret = button.justPressed;
    if (consume) {
        button.justPressed = FALSE;
    }
    return ret;
}

static bool buttonJustReleased(Button &button, bool consume)
{
    int ret = button.justReleased;
    if (consume) {
        button.justReleased = FALSE;
    }
    return ret;
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

static int maquetteRawToCanonical(int16_t potValue, int towerId, int jointId) {
   int16_t center = maquetteJointCenter[towerId][jointId];
   int16_t min = maquetteJointRange[towerId][jointId][0] + center;
   int16_t max = maquetteJointRange[towerId][jointId][1] + center;
   // XXX - moving to fixed point, range [-256, 255]
   //float canonicalValue = potValue < center ? ((float)(potValue - center))/((float)(center - min)) : ((float)(potValue - center))/((float)(max - center));
   int canonicalValue = potValue < center ? ((center - potValue)<<8)/(min - center) : ((potValue - center)<<8)/(max - center);
   return clamp(canonicalValue, -256, 255);
}

static int16_t maquetteCanonicalToSculpture(int towerId, int jointId) {
   return (int16_t)(canonicalJointPosition[towerId][jointId]*SCULPTURE_RANGE/2) + SCULPTURE_RANGE/2; 
}
static int16_t maquetteToModel(uint16_t potValue, int towerId, int jointId) {
// Potentiometer has a range of POT_RANGE (1K)  XXX check this!!
// Sculpture has a range of SCULPTURE_RANGE (8K)

// Assuming the zeros are lined up, that means that the translatiion is
   long potValueCentered = ((long)potValue) - maquetteJointCenter[towerId][jointId];
   int16_t val = (int16_t)((potValueCentered * SCULPTURE_RANGE)/POT_RANGE);

   return clamp(val, jointRange[towerId][jointId][0], jointRange[towerId][jointId][1]);
}

static float clampf(float value, float min, float max) {
   if (value > max)
       return max;
   if (value < min)
       return min;
   return value;
}


static int16_t clamp(int16_t value, int16_t min, int16_t max) {
  if (value > max) 
    return max;
  if (value < min)
    return min;
  return value;
}

// Safety module - flag unsafe actions. Only one tower can be targeted for the red
// zone at a time.
// XXX - Ideally, this would depend on the current state, rather than the current
// target state. But in order to do that, I need to know the current state.
// XXX - get values for R1, R2, and R3; convert between canonical position and
// joint angle
static int targetRedZoneTower = -1;
static bool sculptureTargetPositionSafe(float pos1, float pos2, float pos3, int tower)
{
  bool isSafe = TRUE;
  /*
  float theta1 = pos1;  // modulo some math here...
  float theta2 = pos2;
  float theta3 = pos3;

  bool redZone = R1*sin(theta1) + R2*sin(theta1+theta2) + R3*sin(theta1+theta2+theta3) > RED_ZONE_EDGE;
  if (redZone && targetRedZoneTower != -1 && targetRedZoneTower != tower) {
    isSafe = FALSE;
  } else if (redZone && targetRedZoneTower == -1) {
    targetRedZoneTower = tower;
    Serial.print("Tower entering red zone: ");
    Serial.println(tower);
    isSafe = TRUE;
  } else if (!redZone && targetRedZoneTower == tower) {
    Serial.print("Tower leaving red zone: ");
    Serial.println(tower);
    targetRedZoneTower = -1;
    isSafe = TRUE;
  }
  */
  return isSafe;
}

// Send target position to sculpture
static void setSculptureTargetPosition(int positionArray[NUM_TOWERS][NUM_JOINTS]) {  // please tell me this isn't going to make a copy
    char modelString[256];
    char smallModelString[64];
    char *ptr = modelString;

    if (!maquetteCalibrated()) {
      log_error("Target sculpture called on uncalibrated sculpture. Center first!!");
      return;
    }

    for (int i=0; i<NUM_TOWERS; i++){
        if (!towerEnabled[i]) {
          Serial.print("Tower not enabled: ");
          Serial.println(i);
          continue;
        }
        if (!sculptureTargetPositionSafe(positionArray[i][0], positionArray[i][1], positionArray[i][2], i)) {
          Serial.print("Tower motion disallowed - safety: ");
          Serial.println(i);
          continue;
        }
        for (int j=0; j<NUM_JOINTS; j++) {
            if (!jointEnabled[i][j]) {
              Serial.print("Joint not enabled: ");
              Serial.print(i);
              Serial.println(j);
              continue;
            }
#ifdef USE_CANONICAL_VALUES
            // char floatStr[10]; // float converted to string
            // dtostrf(positionArray[i][j], 2, 3, floatStr);
            sprintf(ptr, "<%d%df%s>", i+1, j+1, positionArray[i][j]);
            ptr += strlen(ptr);
#else
            int position =  positionArray[i][j] * 2;  // convert from 255 range to 512 range. 512 range is safe for tower - 2K per quarter
#endif //USE_CANONICAL_VALUES

/*            if (i==0 && j==1) {
              sprintf(smallModelString, "<%d%dt%d>\r\n", i+1, j+1, positionArray[i][j]);
              Serial.print(smallModelString);
            }
*/
        }
    }
    sprintf(ptr, "\r\n");
#ifdef MAQUETTE_STAND_ALONE
    fprintf(NETWORKFILE, modelString);
#else
    Serial.print(modelString);
#endif // MAQUETTE_STANDALONE

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

/** POSE MODE FUNCTIONS ***/

// 'Pose' mode sets the towers to the current maquette state, and holds the pose for
// some number of seconds. It is triggered by a button press
static int posePosition[NUM_TOWERS][NUM_JOINTS] = { {0,0,0},
                                             {0,0,0},
                                             {0,0,0},
                                             {0,0,0}};
#define BLINK_TIME 400
#define POSE_STATE_WAITING 0
#define POSE_STATE_HOLD 1

uint8_t poseState = POSE_STATE_WAITING;
uint8_t poseDataValid = FALSE; 
static void poseLEDSet(int onOff);
static void poseLEDBlinkOnTimeout();
static bool poseGetPoseData();
static void poseModeInit();
static void poseModeSetWaiting();
static void poseModeSetHolding();

static void poseModeInit()
{
  poseState = POSE_STATE_WAITING;
  poseDataValid = FALSE;
  poseLEDSet(1);
}

static int poseLEDOnOff = 0;
static unsigned long poseBlinkTransitionTime = 0;
static void poseLEDSet(int onOff)
{
    digitalWrite(POSE_BUTTON_LED, onOff?HIGH:LOW);
    poseLEDOnOff = onOff;
}

static void poseLEDBlinkOnTimeout() 
{
   unsigned long curTime = millis();
   if (curTime > poseBlinkTransitionTime) {
      poseLEDOnOff = 1 - poseLEDOnOff;
      poseLEDSet(poseLEDOnOff);
      poseBlinkTransitionTime = curTime + BLINK_TIME;
   }
}


static void poseModeSetWaiting()
{
  Serial.println("Transition - to pose mode waiting");
  poseState = POSE_STATE_WAITING;
  poseLEDSet(1);
}


static void poseModeSetHolding()
{
  Serial.println("Transition - to pose mode holding");
  poseState = POSE_STATE_HOLD;
  poseLEDSet(0);
  poseTimeout = millis() + POSE_LENGTH_MILLIS;
}


static bool poseModeGetPoseData()
{
  bool haveNewPose = FALSE;
  if (poseState == POSE_STATE_WAITING) {
    // check for button press
    if (buttonJustPressed(poseButton, TRUE)) {
      // get pose data
      for (int i=0; i<NUM_TOWERS; i++) {
        for (int j=0; j<NUM_JOINTS; j++) {
          posePosition[i][j] = canonicalJointPosition[i][j];
        }
      }
      haveNewPose = TRUE;
    }
  } else if (poseState == POSE_STATE_HOLD) {
    // throw away button presses in hold state
    buttonJustPressed(poseButton, TRUE);
  }

  return haveNewPose;
}


static void poseModeRunStateMachine()
{
    // If we have valid pose data, pose the machine.
    // This is true no matter whether we're in waiting state or hold state -
    // even if we're for a new pose, we continue to send the previous pose
    // until told otherwise
    if (poseDataValid) {
        setSculptureTargetPosition(posePosition);
    }
    // Check the state transitions
    if (poseState == POSE_STATE_HOLD) {
        // Back to wait state if we've been holding too long
        if (millis() > poseTimeout) {
            poseModeSetWaiting();
        // Otherwise blink the button
        } else {
            poseLEDBlinkOnTimeout();    
        }
    } else if (poseState == POSE_STATE_WAITING) {
        if (poseModeGetPoseData()) { // Button pressed, new pose data
            poseModeSetHolding();
        }
    }
}


static int gLEDOn = 0;  // XXX testing LED blink...

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
    int error = FALSE; // XXX kill this
    
    if (bufEnd <= buf) return;
    
    c = *ptr++;
    log_debug("maquette command!!!");
    Serial.print("maquette command ");
    Serial.println(c);
   
    Serial.println("switching on char..."); 
    switch(c) {
    case 'b':  // toggle led on button... testing
      gLEDOn = 1 - gLEDOn;
      poseLEDSet(gLEDOn);
      Serial.println("<!mb+>");
      break;
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
      
      maquetteJointRange[towerId][jointId][0] = minVal;
      maquetteJointRange[towerId][jointId][1] = maxVal;

      if (debug) {
        sprintf(outBuf, "Maquette Tower %d joint %d set to min: %d, max: %d,\n", towerId+1, jointId+1, minVal, maxVal);
        log_debug(outBuf);
      }
      writeTowerCalibrationData(towerId);
      Serial.println("<!mL>");
      break; 
    case 'D':  // set debug level
      Serial.println("Setting debug level"); 
      ptr++; // XXX skipping ':', see note about parsing better globally
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
    case 'M':  // set mode
      Serial.println("Setting mode...");
      // skip to arguments. XXX I really out to parse them out up front.
      ptr++;  // skip the ':'
      c =*ptr++;
      val = c - '0';
      sprintf(outBuf, "Setting mode to %d", val);
      log_debug(outBuf);
      
      switch(val) {
        case 0:
            mode = MODE_OFF;
            poseLEDSet(0);
            break;
        case 1:
            if (!maquetteCalibrated()) {
                Serial.println("<!mM-Invalid State>");
                error = TRUE;
            } else {
                poseLEDSet(0);
                mode = MODE_IMMEDIATE;
            }
            break;
        case 2:
            if (!maquetteCalibrated()) {
                Serial.println("<!mM-Invalid State>");
                error = TRUE;
            } else {
                mode = MODE_POSE;
                poseModeInit();
            }
            break;
        case 3:
            if (!maquetteCalibrated()) {
                Serial.println("<!mM-Invalid State>");
                error = TRUE;
            } else {
                poseLEDSet(0);
                mode = MODE_PLAYBACK;
            }
            break;
        default:
            Serial.println("<!mM-Invalid Mode>");
            error = TRUE;
            break;
      }

      if (mode != MODE_PLAYBACK) {
        slaveValidBitmask = 0x00;
      }

      if (debug ) {
        if (!error) {
            sprintf(outBuf, "Mode set to %s\n", modeStr[mode]);
            log_debug(outBuf);
        }
      }     
      if (!error) { // XXX this is a terrible way to handle errors here, but I want to refactor most of this case shit.
        Serial.println("<!mM+>");
      }
      break; 
    case 's': // Get status
        Serial.println("Getting status");
        // Note that I'm being very careful here not to overrun outBuf, which can't be very big because we have a small stack.
        sprintf(outBuf, "<!ms{\"mode\": \"%s\", \"poseState\": %d, \"towerState\": [", modeStr[mode], poseState);
        Serial.print(outBuf);
        for (int i=0; i<NUM_TOWERS; i++) {
            // XXX NB - I'm splitting this printf up into several parts because the Arduino folks, in their infinite
            // wisdom, have decided that sprintf should not support float formatting. Serial.print(), however, does. Mfkers.
            sprintf(outBuf, "{\"tower\": %d, \"calibrated\": %s, \"jointPos\" : [", i, (maquetteTowerCenterCalibrated[i] ? "true" : "false"));
            Serial.print(outBuf);
            Serial.print(canonicalJointPosition[i][0]);
            Serial.print(", ");
            Serial.print(canonicalJointPosition[i][1]);
            Serial.print(", ");
            Serial.print(canonicalJointPosition[i][2]);
            sprintf(outBuf, "], \"jointCenter\": [%d, %d, %d], \"jointLimits\": [[%d,%d],[%d,%d],[%d,%d]], \"towerEnabled\":%s, \"jointEnabled\" : [%s, %s, %s]},",
                                maquetteJointCenter[i][0], maquetteJointCenter[i][1], maquetteJointCenter[i][2],
                                maquetteJointRange[i][0][0], maquetteJointRange[i][0][1], maquetteJointRange[i][1][0], maquetteJointRange[i][1][1], maquetteJointRange[i][2][0], maquetteJointRange[i][2][1],
                                (towerEnabled[i] ? "true" : "false"),
                                (jointEnabled[i][0] ? "true" : "false"),
                                (jointEnabled[i][1] ? "true" : "false"),
                                (jointEnabled[i][2] ? "true" : "false"));
                                
            if (i==3)
                outBuf[strlen(outBuf)-1] = '\0'; // remove trailing ','
            Serial.print(outBuf);    
        }
        Serial.println("]}>");
        break;
    case 'P':  // set slave (pose) data
        // XXX - this is much better done in canonical values rather than in raw values. This eliminates the
        // calibration issue.
        float target[3];
        // It does, however, cause problems because Arduino can't sscanf floats. 
        ntokens = sscanf (ptr,"t%hhu:%hu,%hu,%hu", 
                          &towerId, &target[0], &target[1], &target[2]); 
        if (ntokens != 4 || towerId > 4 || towerId < 1) {
          log_error("Invalid tower");
          break;
        }
        towerId--; // 0 based idx
        for (int i=0; i<3; i++) {
          slavePosition[towerId][i] = clamp(target[i], -1.0, 1.0);
          /* sprintf(outBuf, "Clamping %d between %d and %d, result %d\n", jtarget[i], maquetteJointRange[towerId][i][0], 
                                                                                    maquetteJointRange[towerId][i][1], 
                                                                                    slavePosition[towerId][i]);
          log_debug(outBuf);
          */
        } 
        slaveValidBitmask |= (1 << towerId);

        /*
        if (debug) {
           sprintf(outBuf, "Set pose data for tower %d to %d, %d, %d\n", towerId+1, slavePosition[towerId][0], 
                                                                                    slavePosition[towerId][1],
                                                                                    slavePosition[towerId][2]);
           log_debug(outBuf);
           sprintf(outBuf, "Slave bitmask is %x\n", slaveValidBitmask);
           log_debug(outBuf);
        }
        */
        Serial.println("<!mP+>");
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
          Serial.print("<!mC");
          Serial.print(towerId);
          Serial.println("-Invalid number of centers>");
          break; 
        }

        if (towerId > 4 || towerId < 1) {
          log_error("Invalid tower");
          Serial.println("<!mC-Invalid tower>");
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
              Serial.println("<!mC-Center 1 out of range>");
              break;
          } else if (jcenter[1] > POT_RANGE || jcenter[1] < 0) {
              log_error("Joint 2 center out of range\n");
              Serial.println("<!mC-Center 2 out of range>");
              break;
          } else if (jcenter[2] > POT_RANGE || jcenter[2] < 0) {
              log_error("Joint 3 center out of range\n");
              Serial.println("<!mC-Center 3 out of range>");
              break;
          } 
       } 
       
       for (int i=0; i<NUM_JOINTS; i++) {
          maquetteJointCenter[towerId][i] = jcenter[i];   
       }
       maquetteTowerCenterCalibrated[towerId] = TRUE;
       // Serial.println(maquetteToModel(jcenter[0], towerId, 0));

       writeTowerCalibrationData(towerId);
       Serial.print("<!mC");
       Serial.print(towerId+1);
       Serial.println("+>");  // XXX see comments below
       
       break;
    case 'c': // Get calibration status
      sprintf(outBuf, "<!mc[%d, %d, %d, %d]>", maquetteTowerCenterCalibrated[0], maquetteTowerCenterCalibrated[1], maquetteTowerCenterCalibrated[2], maquetteTowerCenterCalibrated[3]);
      Serial.println(outBuf);
      break;
    case 'X': // uncalibrate
      ntokens = sscanf(ptr, "%hhu", &towerId);
      if (ntokens == 1 && (towerId < 1 || towerId > 4)) {
        Serial.print("<!mX");
        Serial.print(towerId);
        Serial.println("-Bad Tower ID>");
        break;
      }
      if (ntokens == 1) {
        Serial.print("Setting tower center calibrated to FALSE for tower");
        Serial.println(towerId);
        maquetteTowerCenterCalibrated[towerId-1] = FALSE;
      } else {
        Serial.println("Setting center calibrated to false for all towers");
        for (int i=0; i<NUM_TOWERS; i++) { 
           maquetteTowerCenterCalibrated[i] = FALSE;
        }
      }
      // XXX - Notes for better things. Okay. So let's assume that I can write the protocol for the maquette. What do I want it to be?
      // Let's keep it of the form <mxxxx>
      // Then we can do <m<0-99><c>:<t>:<j>:<args>>  And here t and j and args can be blank, but we always have the ':' in them. Then the response is
      // <!m<0-99><c>:<success/failure>:response>>  And *that* is a lot more reasonable.
      // But first shall I figure out the storage problem? 
      writeCalibrationData();
      Serial.print("<!mX");
      Serial.print(towerId);
      Serial.println("+>");
      break;
    case 'q':
      // Nuke eeprom.
      for (int i=0; i<48; i++) {
        uint8_t foo = 0;
        EEPROM.put(i, foo);
      }
      Serial.println("<!mq+>");
      break;
    case 'Q':
      printCalibrationData();
      Serial.println("<!mQ+>");
      break;
    case 'w':
      ntokens = sscanf(ptr, "%hhu:%hhu", &towerId, &val);
      if (ntokens == 2) {
         unsigned int eeprom_offset = towerId;
         uint8_t byte_val = (uint8_t)val;
         EEPROM.put(eeprom_offset, byte_val);
         Serial.print("Attempting tp write ");
         Serial.print(byte_val);
         Serial.print(" at location ");
         Serial.println(eeprom_offset);
         printCalibrationData();
      } else {
        Serial.println("Wrong number of tokens");
      }
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
        
    case 'S': // simulation. Get the position that we would be sending the towers
       sprintf(outBuf, "<!mS[[%d,%d,%d], [%d,%d,%d],[%d,%d,%d],[%d,%d,%d]]>", 
            modelPosition[0,0], modelPosition[0,1], modelPosition[0,2],
            modelPosition[1,0], modelPosition[1,1], modelPosition[1,2],
            modelPosition[2,0], modelPosition[2,1], modelPosition[2,2],
            modelPosition[3,0], modelPosition[3,1], modelPosition[3,2]);
       Serial.println(outBuf);
       break;

    case 'A': // analog read of the joints. Sanity check
      sprintf(outBuf, "<!mA[[%d,%d,%d], [%d,%d,%d],[%d,%d,%d],[%d,%d,%d]]>",
        readJointPos(0,0), readJointPos(0,1), readJointPos(0,2),
        readJointPos(1,0), readJointPos(1,1), readJointPos(1,2),
        readJointPos(2,0), readJointPos(2,1), readJointPos(2,2),
        readJointPos(3,0), readJointPos(3,1), readJointPos(3,2));
        
      Serial.println(outBuf);
      break;
    
    case 'p': // Get just the joint position, canonical form
        // Using Serial.print because the Arduino folks have not implemented
        // sprintf for floats. Fuckers.
        Serial.print("<!mp[");
        for (int i=0; i<NUM_TOWERS; i++) {
            Serial.print("[");
            for (int j=0; j<NUM_JOINTS; j++) {
                Serial.print(canonicalJointPosition[i][j], 2);
                if (j < NUM_JOINTS-1) {
                    Serial.print(",");
                }
            }
            Serial.print("]");
            if (i< NUM_TOWERS-1) {
                Serial.print(",");
            }
        }
        Serial.print("]>");   
        break;
    default:
      Serial.println("Couldn not find case for char");
      break; 
    }
    Serial.println("Finish parse maquette command");
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



