#ifndef CAN_MESSAGE_H
#define CAN_MESSAGE_H
#include <stdint.h>
/* CAN Bus Spec for CROWN

- ids of the devices. (Doesn't much matter. Just make them different. Make
master have highest priority on the bus

master

home joint                  1   tj
stop running                2   t0
start running               3   t0
dump joint status           4   tj
set limits on joints        5   tj   m1  m1  M1  M1  c1  c1
set debug level             6   t0   level
set homing speed            7   tj   s
set joint position          8   tj   p1  p1
neuter                      9   tj (may be 0)
set valve drive limits (do I use these?) a
                            b   t0   m   M
set joint P                 c   tj   P1  P1
set joint I                 d   tj   I1  I1
get gen status              10  t
get position                11  t
get limits                  12  t

status running/error        3   t   1
status enabled              3   t   2
status joint limits         3   j   3
status P   values           3   0   4
status I   values           3   0   5
status position


drivers

position                    1   0   p1   p1  p2  p2  p3  p3
joint limits                2   j   m1   m2  M1  M2  c1  c2
general status              3   0   r|h  e   sw  je1 je2 je3  // bitfields r
<<0, h<<1 . h is three bits. sw is two bits per joint sw0 << 0, sw1 << 2, sw2 <<
4 valves                      4   0   v1   v1  v2  v2  v3  v3 joint status 5   j
p1   p1  v1  v1  e|h sw homing status               6   j   p1   p1  sw  tar
extended status             7   0   addr ver debug home speed
p values                    8   0   P1   P1  P2  P2  P3  P3
i values                    9   0   I1   I1  I2  I2  I3  I3
ack                         16  0   cmdId
*/

enum {
  /*    CAN_CMD_HOME_JOINT    = 0x01,
      CAN_CMD_START_RUNNING = 0x02,
      CAN_CMD_STOP_RUNNING  = 0x03,
      CAN_CMD_DUMP_JOINT    = 0x04,
      CAN_CMD_LIMIT_JOINT   = 0x05,
      CAN_CMD_DEBUG_LEVEL   = 0x06,
      CAN_CMD_HOME_SPEED    = 0x07,
      CAN_CMD_SET_JOINT_POS = 0x08,
      CAN_CMD_NEUTER_JOINT  = 0x09,
      CAN_CMD_SET_DRIVE_LIMITS = 0x0a,
      CAN_CMD_SET_JOINT_P   = 0x0b,
      CAN_CMD_SET_JOINT_I   = 0x0c,*/
  CAN_CMD_GET_GEN_STATUS = 0x11,
  CAN_CMD_GET_JOINT_STATUS = 0x12,
  CAN_CMD_GET_JOINT_LIMITS = 0x13,
  CAN_CMD_GET_POSITION = 0x14,
  CAN_CMD_GET_PID_VALUES = 0x15,
  CAN_CMD_GET_EXT_STATUS = 0x16,
  CAN_CMD_GET_VALVES = 0x17,
  CAN_CMD_GET_INTEGRATORS = 0x18,
};

#define CAN_CMD_BYTE 0
#define CAN_TOWER_JOINT_BYTE 1
#define CAN_TOWER_MASK 0x3F
#define CAN_JOINT_MASK 0x03
#define CAN_TOWER_SHIFT 2
#define CAN_JOINT_SHIFT 0
#define CAN_DATA 2

enum {
  CAN_MSG_GENERAL_STATUS = 0x01,
  CAN_MSG_JOINT_STATUS = 0x02,
  CAN_MSG_JOINT_LIMITS = 0x03,
  CAN_MSG_POSITION = 0x04,
  CAN_MSG_PID_VALUES = 0x05,
  CAN_MSG_EXTENDED_STATUS = 0x06,
  CAN_MSG_VALVES = 0x07,
  CAN_MSG_HOMING_STATUS = 0x08,
  CAN_MSG_HOMING_RESULT = 0x09,
  CAN_MSG_INTEGRATORS = 0x0a,
  CAN_MSG_ACK = 0x10
};

enum {
  HOMING_RESULT_OK = 0,
  HOMING_RESULT_STALL = 1,
  HOMING_RESULT_CENTER_NOT_FOUND = 2,
  HOMING_RESULT_ESTOP = 3,
  HOMING_RESULT_CLEAR = 4
};

#define MAQUETTE_TX_ID 0x11FFFFFF
#define TOWER_1_TX_ID 0x13FFFFFF
#define TOWER_2_TX_ID 0x14FFFFFF
#define TOWER_3_TX_ID 0x15FFFFFF
#define TOWER_4_TX_ID 0x16FFFFFF

typedef struct {
  uint8_t runState;
  uint8_t state;
  uint8_t homed[3];
  uint8_t sw[3];
  uint8_t jointEnable[3];
} CAN_StatusStruct;

#ifdef __cplusplus
extern "C" {
#endif
unsigned long CAN_getTowerTxId(uint8_t towerId);
void CAN_setTxId(unsigned long txId);
int CAN_RequestGeneralStatus(uint8_t towerId);
int CAN_RequestJointStatus(uint8_t towerId, uint8_t jointId);
int CAN_RequestJointLimits(uint8_t towerId, uint8_t jointId);
int CAN_RequestPosition(uint8_t towerId);
int CAN_RequestPIDValues(uint8_t towerId);
int CAN_RequestExtendedStatus(uint8_t towerId);
int CAN_RequestValves(uint8_t towerId);
int CAN_RequestIntegrators(uint8_t towerId);

int CAN_SendGeneralStatus(CAN_StatusStruct *status);
int CAN_SendJointStatus(uint8_t jointId, int16_t pos, int16_t target,
                        uint8_t valve, uint8_t sw, uint8_t homed,
                        uint8_t enabled);
int CAN_SendJointLimits(uint8_t jointId, int16_t minVal, int16_t maxVal,
                        int16_t centerVal);
int CAN_SendPosition(int16_t j1, int16_t j2, int16_t j3);
int CAN_SendPIDValues(int8_t p1, int8_t p2, int8_t p3, int8_t i1, int8_t i2,
                      int8_t i3);
int CAN_SendExtendedStatus(uint8_t addr, uint8_t debugLevel, uint8_t h1,
                           uint8_t h2, uint8_t h3);
int CAN_SendValves(uint8_t v1, uint8_t v2, uint8_t v3);
int CAN_SendIntegrators(uint16_t i1, uint16_t i2, uint16_t i3);

int CAN_SendHomingStatus(uint8_t jointId, int16_t pos, uint8_t sw,
                         uint8_t target, uint16_t numStalls);
int CAN_SendHomingResult(uint8_t jointId, uint8_t success, uint8_t detail,
                         uint8_t target);

typedef struct {
  uint8_t Kp[3];
  uint8_t Ki[3];
} CAN_PIDValues;

typedef struct {
  uint8_t addr;
  uint8_t debugLevel;
  uint8_t homeSpeed[3];
} CAN_ExtendedStatus;

typedef struct {
  uint16_t pos;
  uint8_t switches;
  uint8_t target;
  uint16_t stalls;
} CAN_Homing;

typedef struct {
  uint8_t success;
  uint8_t detail;
  uint8_t target; // final target - all the switches we were still looking for
                  // when we finished
} CAN_Homing_Result;

typedef struct {
  int16_t pos;
  int16_t target;
  uint8_t valve;
  uint8_t enabled;
  uint8_t homed;
  uint8_t switches;
} CAN_JointStatus;

typedef struct {
  uint8_t driveLevel[3];
} CAN_Valves;

typedef struct {
  int16_t minPos;
  int16_t maxPos;
  int16_t centerPos;
} CAN_Limits;

typedef struct {
  int16_t joint1Pos;
  int16_t joint2Pos;
  int16_t joint3Pos;
} CAN_Position;

typedef struct {
  uint16_t i1;
  uint16_t i2;
  uint16_t i3;
} CAN_Integrators;

void CAN_BufferToPosition(uint8_t *buf, CAN_Position *pos);
void CAN_BufferToJointLimits(uint8_t *buf, CAN_Limits *limits);
void CAN_BufferToStatus(uint8_t *buf, CAN_StatusStruct *canStatus);
void CAN_BufferToValves(uint8_t *buf, CAN_Valves *valves);
void CAN_BufferToJointStatus(uint8_t *buf, CAN_JointStatus *joint);
void CAN_BufferToPIDValues(uint8_t *buf, CAN_PIDValues *pids);
void CAN_BufferToExtendedStatus(uint8_t *buf, CAN_ExtendedStatus *status);
void CAN_BufferToHomingStatus(uint8_t *buf, CAN_Homing *homing);
void CAN_BufferToHomingResult(uint8_t *buf, CAN_Homing_Result *homing);
void CAN_BufferToIntegrators(uint8_t *buf, CAN_Integrators *integrators);

#ifdef __cplusplus
}
#endif
#endif // CAN_MESSAGE_H
