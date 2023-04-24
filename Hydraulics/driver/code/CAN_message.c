/* CAN Bus Spec

- ids of the devices. (Doesn't much matter. Just make them different. Make
master have highest priority on the bus

master

home joint                  1   tj
stop running                2   t0
start running               2   t1   0   0
dump joint status           3   tj
set limits on joints        4   tj   m1  m1  M1  M1  c1  c1
set debug level             5   t0   level
set homing speed            6   tj   s
set joint position          7   tj   p1  p1
neuter                      8   tj (may be 0)
set valve drive limits (do I use these?)
                            9   t0   m   M
set joint P                 a   tj   P1  P1
set joint I                 b   tj   I1  I1

status running/error        3   0   1
status enabled              3   0   2
status joint limits         3   j   3
PID   values         3   0   p1  p2  p3  i1  i2  i3

status position


drivers

position                    1   0   p1   p1  p2  p2  p3  p3
joint limits                2   j   m1   m2  M1  M2  c1  c2
general status              3   0   r|h  e   sw  je1 je2 je3  // bitfields r
<<0, h<<1 . h is three bits. sw is two bits per joint sw0 << 0, sw1 << 2, sw2 <<
4 valves                      4   0   v1   v1  v2  v2  v3  v3 joint status 5   j
p1   p1  v1  v1  e|h sw homing status               6   j   p1   p1  sw  tar
extended status             7   0   addr ver debug home speed
PID                         values         3   0   p1  p2  p3  i1  i2  i3
p values                    8   0   P1   P1  P2  P2  P3  P3
i values                    9   0   I1   I1  I2  I2  I3  I3
ack                         16  0   cmdId

enum {
    CAN_CMD_HOME_JOINT    0x01,
    CAN_CMD_START_RUNNING 0x02,
    CAN_CMD_STOP_RUNNING  0x03,
    CAN_CMD_DUMP_JOINT    0x04,
    CAN_CMD_LIMIT_JOINT   0x05,
    CAN_CMD_DEBUG_LEVEL   0x06,
    CAN_CMD_HOME_SPEED    0x07,
    CAN_CMD_SET_JOINT_POS 0x08,
    CAN_CMD_NEUTER_JOINT  0x09,
    CAN_CMD_SET_DRIVE_LIMITS 0x0a,
    CAN_CMD_SET_JOINT_P   0x0b,
    CAN_CMD_SET_JOINT_I   0x0c,
    CAN_CMD_GET_GEN_STATUS 0x10,
    CAN_CMD_GET_POSITION   0x11,
    CAN_CMD_GET_LIMITS     0x12,
    CAN_CMD_GET_VALVES     0x13,
    CAN_CMD_GET_JOINT_STATUS 0x014,
    CAN_CMD_GET_EXT_STATUS 0x15,
};

#define CAN_CMD_BYTE 0
#define CAN_TOWER_JOINT_BYTE 1
#define CAN_TOWER_MASK 0x3F
#define CAN_JOINT_MASK 0x03
#define CAN_TOWER_SHIFT 2
#define CAN_JOINT_SHIFT 0
#define CAN_DATA 2

enum {
    CAN_MSG_POSITION        0x01,
    CAN_MSG_JOINT_LIMITS    0x02,
    CAN_MSG_GENERAL_STATUS  0x03,
    CAN_MSG_VALVE_STATUS    0x04,
    CAN_MSG_JOINT_STATUS    0x05,
    CAN_MSG_HOMING_STATUS   0x06,
    CAN_MSG_EXTENDED_STATUS 0x07,
    CAN_MSG_ACK             0x10
};
*/
#include "CAN_message.h"
#include "mcp_can_wrapper.h"

static unsigned long g_txId = 0;

#define TX_ID_INVALID 0x1FFFFFFF

unsigned long CAN_TX_MAP[] = {TOWER_1_TX_ID, TOWER_2_TX_ID, TOWER_3_TX_ID,
                              TOWER_4_TX_ID};

unsigned long CAN_getTowerTxId(uint8_t towerId) {
  if (towerId < 1 || towerId > 4) {
    return TX_ID_INVALID;
  }

  return CAN_TX_MAP[towerId - 1];
}

void CAN_setTxId(unsigned long txId) { g_txId = txId; }

/*
int CAN_HomeJoint(uint8_t tower, uint8_t jointId) {
    uint8_t cmd[8];
    uint8_t cmdLen = 2;
    cmd[CAN_CMD_BYTE]         = CAN_CMD_HOME_JOINT;
    cmd[CAN_TOWER_JOINT_BYTE] = ((tower & CAN_TOWER_MASK) << CAN_TOWER_SHIFT) |
((jointId & CAN_JOINT_MASK) << CAN_JOINT_SHIFT);

    // push into CAN
    return (mcp_can_send(g_txId, 1, cmdLen, cmd));

}

int CAN_SetRunState(uint8_t tower, int bRunning) {
    uint8_t cmd[8];
    uint8_t cmdLen = 2;
    cmd[CAN_CMD_BYTE]         = bRunning ? CAN_CMD_START_RUNNING :
CAN_CMD_STOP_RUNNING; cmd[CAN_TOWER_JOINT_BYTE] = ((tower & CAN_TOWER_MASK) <<
CAN_TOWER_SHIFT);

    // push into CAN
    return (mcp_can_send(g_txId, 1, cmdLen, cmd));

}
*/

/*
int CAN_RequestJointStatus(uint8_t tower, uint8_t jointId) {
    uint8_t cmd[8];
    uint8_t cmdLen = 2;
    cmdCAN_CMD_BYTE]         = CMD_DUMP_JOINT;
    cmd[CAN_TOWER_JOINT_BYTE] = ((tower & CAN_TOWER_MASK) << CAN_TOWER_SHIFT) |
((jointId & CAN_JOINT_MASK) << CAN_JOINT_MASK);

    // push into CAN
    return (mcp_can_send(g_txId, 1, cmdLen, cmd));

}
// XXX - dump joint vs get joint status??
*/

/*
int CAN_SetJointLimits(uint8_t tower, uint8_t jointId, int16_t minVal, int16_t
maxVal, int16_t centerVal) { uint8_t cmd[8]; uint8_t cmdLen = 8;
    cmd[CAN_CMD_BYTE]         = CAN_CMD_LIMIT_JOINT;
    cmd[CAN_TOWER_JOINT_BYTE] = ((tower & CAN_TOWER_MASK) << CAN_TOWER_SHIFT) |
((jointId & CAN_JOINT_MASK) << CAN_JOINT_SHIFT); cmd[CAN_DATA+0]       =
(uint8_t)((minVal & 0xFF00) >> 8); cmd[CAN_DATA+1]       = (uint8_t)(minVal &
0x00FF); cmd[CAN_DATA+2]       = (uint8_t)((maxVal & 0xFF00) >> 8);
    cmd[CAN_DATA+3]       = (uint8_t)(maxVal & 0x00FF);
    cmd[CAN_DATA+4]       = (uint8_t)((centerVal & 0xFF00) >> 8);
    cmd[CAN_DATA+5]       = (uint8_t)(centerVal & 0x00FF);

    // push into CAN
    return (mcp_can_send(g_txId, 1, cmdLen, cmd));
}

int CAN_SetDebugLevel(uint8_t tower, uint8_t level) {
    uint8_t cmd[8];
    uint8_t cmdLen = 3;
    cmd[CAN_CMD_BYTE]         = CAN_CMD_DEBUG_LEVEL;
    cmd[CAN_TOWER_JOINT_BYTE] = ((tower & CAN_TOWER_MASK) << CAN_TOWER_SHIFT);
    cmd[CAN_DATA+0]       = level;
    return (mcp_can_send(g_txId, 1, cmdLen, cmd));
}

int CAN_SetHomeSpeed(uint8_t tower, uint8_t jointId, uint8_t speed) {
    uint8_t cmd[8];
    uint8_t cmdLen = 3;
    cmd[CAN_CMD_BYTE]         = CAN_CMD_HOME_SPEED;
    cmd[CAN_TOWER_JOINT_BYTE] = ((tower & CAN_TOWER_MASK) << CAN_TOWER_SHIFT) |
((jointId & CAN_JOINT_MASK) << CAN_JOINT_SHIFT); cmd[CAN_DATA+0]       = speed >
63 ? 63 : speed;

    // push into CAN
    return (mcp_can_send(g_txId, 1, cmdLen, cmd));
}

int CAN_Neuter(uint8_t tower, uint8_t joint){
    uint8_t cmd[8];
    uint8_t cmdLen = 2;
    cmd[CAN_CMD_BYTE]         = CAN_CMD_NEUTER_JOINT;
    cmd[CAN_TOWER_JOINT_BYTE] = ((tower & CAN_TOWER_MASK) << CAN_TOWER_SHIFT) |
((joint & CAN_JOINT_MASK) << CAN_JOINT_SHIFT);

    // push into CAN
    return (mcp_can_send(g_txId, 1, cmdLen, cmd));
}

int CAN_SetDriveLimits(uint8_t tower, uint8_t joint, int16_t minDrive, int16_t
maxDrive) { uint8_t cmd[8]; uint8_t cmdLen = 6; cmd[CAN_CMD_BYTE]         =
CAN_CMD_SET_DRIVE_LIMITS; cmd[CAN_TOWER_JOINT_BYTE] = ((tower & CAN_TOWER_MASK)
<< CAN_TOWER_SHIFT) | ((joint & CAN_JOINT_MASK) << CAN_JOINT_SHIFT);
    cmd[CAN_DATA+0]       = (uint8_t)((minDrive & 0xFF00) >> 8);
    cmd[CAN_DATA+1]       = (uint8_t)(minDrive & 0x00FF);
    cmd[CAN_DATA+2]       = (uint8_t)((maxDrive & 0xFF00) >> 8);
    cmd[CAN_DATA+3]       = (uint8_t)(maxDrive & 0x00FF);

    // push into CAN
    return (mcp_can_send(g_txId, 1, cmdLen, cmd));
}

int CAN_SetPValue(uint8_t tower, uint8_t joint, int16_t pVal) {
    uint8_t cmd[8];
    uint8_t cmdLen = 4;
    cmd[CAN_CMD_BYTE]         = CAN_CMD_SET_JOINT_P;
    cmd[CAN_TOWER_JOINT_BYTE] = ((tower & CAN_TOWER_MASK) << CAN_TOWER_SHIFT) |
((joint & CAN_JOINT_MASK) << CAN_JOINT_SHIFT); cmd[CAN_DATA+0]       =
(uint8_t)((pVal & 0xFF00) >> 8); cmd[CAN_DATA+1]       = (uint8_t)(pVal &
0x00FF);

    // push into CAN
    return (mcp_can_send(g_txId, 1, cmdLen, cmd));
}

int CAN_SetIValue(uint8_t tower, uint8_t joint, int16_t iVal) {
    uint8_t cmd[8];
    uint8_t cmdLen = 4;
    cmd[CAN_CMD_BYTE]         = CAN_CMD_SET_JOINT_I;
    cmd[CAN_TOWER_JOINT_BYTE] = ((tower & CAN_TOWER_MASK) << CAN_TOWER_SHIFT) |
((joint & CAN_JOINT_MASK) << CAN_JOINT_SHIFT); cmd[CAN_DATA+0]       =
(uint8_t)((iVal & 0xFF00) >> 8); cmd[CAN_DATA+1]       = (uint8_t)(iVal &
0x00FF);

    // push into CAN
    return (mcp_can_send(g_txId, 1, cmdLen, cmd));
}
*/

int CAN_RequestGeneralStatus(uint8_t tower) {
  uint8_t cmd[8];
  uint8_t cmdLen = 2;
  cmd[CAN_CMD_BYTE] = CAN_CMD_GET_GEN_STATUS;
  cmd[CAN_TOWER_JOINT_BYTE] = ((tower & CAN_TOWER_MASK) << CAN_TOWER_SHIFT);

  // push into CAN
  return (mcp_can_send(g_txId, 1, cmdLen, cmd));
}

int CAN_RequestPosition(uint8_t tower) {
  uint8_t cmd[8];
  uint8_t cmdLen = 2;
  cmd[CAN_CMD_BYTE] = CAN_CMD_GET_POSITION;
  cmd[CAN_TOWER_JOINT_BYTE] = ((tower & CAN_TOWER_MASK) << CAN_TOWER_SHIFT);

  // push into CAN
  return (mcp_can_send(g_txId, 1, cmdLen, cmd));
}

int CAN_RequestJointLimits(uint8_t tower, uint8_t joint) {
  uint8_t cmd[8];
  uint8_t cmdLen = 2;
  cmd[CAN_CMD_BYTE] = CAN_CMD_GET_JOINT_LIMITS;
  cmd[CAN_TOWER_JOINT_BYTE] = ((tower & CAN_TOWER_MASK) << CAN_TOWER_SHIFT) |
                              ((joint & CAN_JOINT_MASK) << CAN_JOINT_SHIFT);

  // push into CAN
  return (mcp_can_send(g_txId, 1, cmdLen, cmd));
}

int CAN_RequestValves(uint8_t tower) {
  uint8_t cmd[8];
  uint8_t cmdLen = 2;
  cmd[CAN_CMD_BYTE] = CAN_CMD_GET_VALVES;
  cmd[CAN_TOWER_JOINT_BYTE] = ((tower & CAN_TOWER_MASK) << CAN_TOWER_SHIFT);

  // push into CAN
  return (mcp_can_send(g_txId, 1, cmdLen, cmd));
}

int CAN_RequestExtendedStatus(uint8_t tower) {
  uint8_t cmd[8];
  uint8_t cmdLen = 2;
  cmd[CAN_CMD_BYTE] = CAN_CMD_GET_EXT_STATUS;
  cmd[CAN_TOWER_JOINT_BYTE] = ((tower & CAN_TOWER_MASK) << CAN_TOWER_SHIFT);

  // push into CAN
  return (mcp_can_send(g_txId, 1, cmdLen, cmd));
}

int CAN_RequestJointStatus(uint8_t tower, uint8_t joint) {
  uint8_t cmd[8];
  uint8_t cmdLen = 2;
  cmd[CAN_CMD_BYTE] = CAN_CMD_GET_JOINT_STATUS;
  cmd[CAN_TOWER_JOINT_BYTE] = ((tower & CAN_TOWER_MASK) << CAN_TOWER_SHIFT) |
                              ((joint & CAN_JOINT_MASK) << CAN_JOINT_SHIFT);

  // push into CAN
  return (mcp_can_send(g_txId, 1, cmdLen, cmd));
}

int CAN_RequestPIDValues(uint8_t tower) {
  uint8_t cmd[8];
  uint8_t cmdLen = 2;
  cmd[CAN_CMD_BYTE] = CAN_CMD_GET_PID_VALUES;
  cmd[CAN_TOWER_JOINT_BYTE] = ((tower & CAN_TOWER_MASK) << CAN_TOWER_SHIFT);

  // push into CAN
  return (mcp_can_send(g_txId, 1, cmdLen, cmd));
}

int CAN_RequestIntegrators(uint8_t tower) {
  uint8_t cmd[8];
  uint8_t cmdLen = 2;
  cmd[CAN_CMD_BYTE] = CAN_CMD_GET_INTEGRATORS;
  cmd[CAN_TOWER_JOINT_BYTE] = ((tower & CAN_TOWER_MASK) << CAN_TOWER_SHIFT);

  // push into CAN
  return (mcp_can_send(g_txId, 1, cmdLen, cmd));
}

int CAN_SendPosition(int16_t j1, int16_t j2, int16_t j3) {
  uint8_t cmd[8];
  uint8_t cmdLen = 8;
  cmd[CAN_CMD_BYTE] = CAN_MSG_POSITION;
  cmd[CAN_TOWER_JOINT_BYTE] = 0;
  cmd[CAN_DATA + 0] = (uint8_t)((j1 & 0xFF00) >> 8);
  cmd[CAN_DATA + 1] = (uint8_t)(j1 & 0x00FF);
  cmd[CAN_DATA + 2] = (uint8_t)((j2 & 0xFF00) >> 8);
  cmd[CAN_DATA + 3] = (uint8_t)(j2 & 0x00FF);
  cmd[CAN_DATA + 4] = (uint8_t)((j3 & 0xFF00) >> 8);
  cmd[CAN_DATA + 5] = (uint8_t)(j3 & 0x00FF);

  // push into CAN
  return (mcp_can_send(g_txId, 1, cmdLen, cmd));
}

int CAN_SendJointLimits(uint8_t jointId, int16_t minVal, int16_t maxVal,
                        int16_t centerVal) {
  uint8_t cmd[8];
  uint8_t cmdLen = 8;
  cmd[CAN_CMD_BYTE] = CAN_MSG_JOINT_LIMITS;
  cmd[CAN_TOWER_JOINT_BYTE] = ((jointId & CAN_JOINT_MASK) << CAN_JOINT_SHIFT);
  cmd[CAN_DATA + 0] = (uint8_t)((minVal & 0xFF00) >> 8);
  cmd[CAN_DATA + 1] = (uint8_t)(minVal & 0x00FF);
  cmd[CAN_DATA + 2] = (uint8_t)((maxVal & 0xFF00) >> 8);
  cmd[CAN_DATA + 3] = (uint8_t)(maxVal & 0x00FF);
  cmd[CAN_DATA + 4] = (uint8_t)((centerVal & 0xFF00) >> 8);
  cmd[CAN_DATA + 5] = (uint8_t)(centerVal & 0x00FF);

  // push into CAN
  return (mcp_can_send(g_txId, 1, cmdLen, cmd));
}

int CAN_SendGeneralStatus(CAN_StatusStruct *status) {
  uint8_t cmd[8];
  uint8_t cmdLen = 7;
  cmd[CAN_CMD_BYTE] = CAN_MSG_GENERAL_STATUS;
  cmd[CAN_TOWER_JOINT_BYTE] = 0;
  cmd[CAN_DATA + 0] = (status->runState << 3) | (status->homed[0] << 2) |
                      (status->homed[1] << 1) | (status->homed[2] << 0);
  cmd[CAN_DATA + 1] = (status->state);
  cmd[CAN_DATA + 2] = ((status->sw[0] & 0x07) << 4) | (status->sw[1] & 0x07);
  cmd[CAN_DATA + 3] = (status->sw[2] & 0x07);
  cmd[CAN_DATA + 4] = (status->jointEnable[0] << 2) |
                      (status->jointEnable[1] << 1) |
                      (status->jointEnable[2] << 0);

  // push into CAN
  return (mcp_can_send(g_txId, 1, cmdLen, cmd));
}

int CAN_SendValves(uint8_t v1, uint8_t v2, uint8_t v3) {
  uint8_t cmd[8];
  uint8_t cmdLen = 5;
  cmd[CAN_CMD_BYTE] = CAN_MSG_VALVES;
  cmd[CAN_TOWER_JOINT_BYTE] = 0;
  cmd[CAN_DATA + 0] = v1;
  cmd[CAN_DATA + 1] = v2;
  cmd[CAN_DATA + 2] = v3;

  // push into CAN
  return (mcp_can_send(g_txId, 1, cmdLen, cmd));
}

typedef struct __attribute__((__packed__)) {
  uint8_t v1;
  uint8_t v2;
  uint8_t v3;
} CAN_Data_Valves;

int CAN_SendJointStatus(uint8_t jointId, int16_t pos, int16_t target,
                        uint8_t valve, uint8_t sw, uint8_t homed,
                        uint8_t hw_enabled, uint8_t sw_enabled) {
  uint8_t cmd[8];
  uint8_t cmdLen = 8;
  cmd[CAN_CMD_BYTE] = CAN_MSG_JOINT_STATUS;
  cmd[CAN_TOWER_JOINT_BYTE] = ((jointId & CAN_JOINT_MASK) << CAN_JOINT_SHIFT);
  cmd[CAN_DATA + 0] = (uint8_t)((pos & 0xFF00) >> 8);
  cmd[CAN_DATA + 1] = (uint8_t)(pos & 0x00FF);
  cmd[CAN_DATA + 2] = (uint8_t)((target & 0xFF00) >> 8);
  cmd[CAN_DATA + 3] = (uint8_t)(target & 0x00FF);
  cmd[CAN_DATA + 4] = valve;
  cmd[CAN_DATA + 5] = ((sw_enabled & 0x01) << 5 | (hw_enabled & 0x01) << 4) |
                      ((homed & 0x01) << 3) | (sw & 0x7);

  // push into CAN
  return (mcp_can_send(g_txId, 1, cmdLen, cmd));
}

typedef struct __attribute__((__packed__)) {

} CAN_header;

typedef struct __attribute__((__packed__)) {
  int16_t pos;
  uint8_t sw;
  uint8_t target;
  uint16_t stalls;
} CAN_Data_HomeStatus;

int CAN_SendHomingStatus(uint8_t jointId, int16_t pos, uint8_t sw,
                         uint8_t target, uint16_t numStalls) {
  uint8_t cmd[8];
  uint8_t cmdLen = 8;
  cmd[CAN_CMD_BYTE] = CAN_MSG_HOMING_STATUS;
  cmd[CAN_TOWER_JOINT_BYTE] = ((jointId & CAN_JOINT_MASK) << CAN_JOINT_SHIFT);
  cmd[CAN_DATA + 0] =
      (uint8_t)((pos & 0xFF00) >> 8); // ?? network byte order? htons
  cmd[CAN_DATA + 1] = (uint8_t)(pos & 0x00FF);
  cmd[CAN_DATA + 2] = sw;
  cmd[CAN_DATA + 3] = target;
  cmd[CAN_DATA + 4] = (uint8_t)((numStalls & 0xFF00) >> 8);
  cmd[CAN_DATA + 5] = (uint8_t)(numStalls & 0x00FF);

  // push into CAN
  return (mcp_can_send(g_txId, 1, cmdLen, cmd));
}

int CAN_SendHomingResult(uint8_t jointId, uint8_t success, uint8_t detail,
                         uint8_t target) {
  uint8_t cmd[8];
  uint8_t cmdLen = 5;
  cmd[CAN_CMD_BYTE] = CAN_MSG_HOMING_RESULT;
  cmd[CAN_TOWER_JOINT_BYTE] = ((jointId & CAN_JOINT_MASK) << CAN_JOINT_SHIFT);
  cmd[CAN_DATA + 0] = success;
  cmd[CAN_DATA + 1] = detail;
  cmd[CAN_DATA + 2] = target;

  // push into CAN
  return (mcp_can_send(g_txId, 1, cmdLen, cmd));
}

typedef struct __attribute__((__packed__)) {
  uint8_t addr;
  uint8_t debugLevel;
  uint8_t home_speed;
} CAN_Data_ExtendedStatus;

int CAN_SendExtendedStatus(uint8_t addr, uint8_t debugLevel, uint8_t h1,
                           uint8_t h2, uint8_t h3) {
  uint8_t cmd[8];
  uint8_t cmdLen = 7;
  cmd[CAN_CMD_BYTE] = CAN_MSG_EXTENDED_STATUS;
  cmd[CAN_TOWER_JOINT_BYTE] = 0;
  cmd[CAN_DATA + 0] = addr;
  cmd[CAN_DATA + 1] = debugLevel;
  cmd[CAN_DATA + 2] = h1;
  cmd[CAN_DATA + 3] = h2;
  cmd[CAN_DATA + 4] = h3;

  // push into CAN
  return (mcp_can_send(g_txId, 1, cmdLen, cmd));
}

int CAN_SendPIDValues(int8_t p1, int8_t p2, int8_t p3, int8_t i1, int8_t i2,
                      int8_t i3) {
  uint8_t cmd[8];
  uint8_t cmdLen = 8;
  cmd[CAN_CMD_BYTE] = CAN_MSG_PID_VALUES;
  cmd[CAN_TOWER_JOINT_BYTE] = 0;
  cmd[CAN_DATA + 0] = (uint8_t)p1;
  cmd[CAN_DATA + 1] = (uint8_t)p2;
  cmd[CAN_DATA + 2] = (uint8_t)p3;
  cmd[CAN_DATA + 3] = (uint8_t)i1;
  cmd[CAN_DATA + 4] = (uint8_t)i2;
  cmd[CAN_DATA + 5] = (uint8_t)i3;

  // push into CAN
  return (mcp_can_send(g_txId, 1, cmdLen, cmd));
}

int CAN_SendIntegrators(uint16_t i1, uint16_t i2, uint16_t i3) {
  uint8_t cmd[8];
  uint8_t cmdLen = 8;
  cmd[CAN_CMD_BYTE] = CAN_MSG_INTEGRATORS;
  cmd[CAN_TOWER_JOINT_BYTE] = 0;
  cmd[CAN_DATA + 0] = (uint8_t)((i1 & 0xFF00) >> 8);
  cmd[CAN_DATA + 1] = (uint8_t)(i1 & 0x00FF);
  cmd[CAN_DATA + 2] = (uint8_t)((i2 & 0xFF00) >> 8);
  cmd[CAN_DATA + 3] = (uint8_t)(i2 & 0x00FF);
  cmd[CAN_DATA + 4] = (uint8_t)((i3 & 0xFF00) >> 8);
  cmd[CAN_DATA + 5] = (uint8_t)(i3 & 0x00FF);

  // push into CAN
  return (mcp_can_send(g_txId, 1, cmdLen, cmd));
}

int CAN_SendAck(uint8_t cmd_to_ack) {
  uint8_t cmd[8];
  uint8_t cmdLen = 3;
  cmd[CAN_CMD_BYTE] = CAN_MSG_ACK;
  cmd[CAN_TOWER_JOINT_BYTE] = 0;
  cmd[CAN_DATA + 0] = cmd_to_ack;

  // push into CAN
  return (mcp_can_send(g_txId, 1, cmdLen, cmd));
}

void CAN_BufferToPosition(uint8_t *buf, CAN_Position *pos) {
  pos->joint1Pos = buf[CAN_DATA + 0] << 8 | buf[CAN_DATA + 1];
  pos->joint2Pos = buf[CAN_DATA + 2] << 8 | buf[CAN_DATA + 3];
  pos->joint3Pos = buf[CAN_DATA + 4] << 8 | buf[CAN_DATA + 5];
}

void CAN_BufferToJointLimits(uint8_t *buf, CAN_Limits *limits) {
  limits->minPos = buf[CAN_DATA + 0] << 8 | buf[CAN_DATA + 1];
  limits->maxPos = buf[CAN_DATA + 2] << 8 | buf[CAN_DATA + 3];
  limits->centerPos = buf[CAN_DATA + 4] << 8 | buf[CAN_DATA + 5];
}

void CAN_BufferToIntegrators(uint8_t *buf, CAN_Integrators *integrators) {
  integrators->i1 = buf[CAN_DATA + 0] << 8 | buf[CAN_DATA + 1];
  integrators->i2 = buf[CAN_DATA + 2] << 8 | buf[CAN_DATA + 3];
  integrators->i3 = buf[CAN_DATA + 4] << 8 | buf[CAN_DATA + 5];
}

void CAN_BufferToStatus(uint8_t *buf, CAN_StatusStruct *canStatus) {
  canStatus->runState = (buf[CAN_DATA] & 0x08) >> 3;
  canStatus->homed[0] = (buf[CAN_DATA] & 0x04) >> 2;
  canStatus->homed[1] = (buf[CAN_DATA] & 0x02) >> 1;
  canStatus->homed[2] = (buf[CAN_DATA] & 0x01) >> 0;
  canStatus->state = buf[CAN_DATA + 1];
  canStatus->sw[0] = (buf[CAN_DATA + 2] & 0xF0) >> 4;
  canStatus->sw[1] = (buf[CAN_DATA + 2] & 0x0F);
  canStatus->sw[2] = (buf[CAN_DATA + 3]);
  canStatus->jointEnable[0] = (buf[CAN_DATA + 4] & 0x04) >> 2;
  canStatus->jointEnable[1] = (buf[CAN_DATA + 4] & 0x02) >> 1;
  canStatus->jointEnable[2] = (buf[CAN_DATA + 4] & 0x01);
  canStatus->jointSWEnable[0] = (buf[CAN_DATA + 4] & 0x08) >> 3;
  canStatus->jointSWEnable[1] = (buf[CAN_DATA + 4] & 0x10) >> 4;
  canStatus->jointSWEnable[2] = (buf[CAN_DATA + 4] & 0x20) >> 5;
}

void CAN_BufferToValves(uint8_t *buf, CAN_Valves *valves) {
  valves->driveLevel[0] = buf[CAN_DATA + 0];
  valves->driveLevel[1] = buf[CAN_DATA + 1];
  valves->driveLevel[2] = buf[CAN_DATA + 2];
}

void CAN_BufferToJointStatus(uint8_t *buf, CAN_JointStatus *joint) {
  joint->pos = buf[CAN_DATA + 0] << 8 | buf[CAN_DATA + 1];
  joint->target = buf[CAN_DATA + 2] << 8 | buf[CAN_DATA + 3];
  joint->valve = buf[CAN_DATA + 4];
  joint->sw_enabled = ((buf[CAN_DATA + 5] >> 5) & 0x01);
  joint->enabled = ((buf[CAN_DATA + 5] >> 4) & 0x01);
  joint->homed = ((buf[CAN_DATA + 5] >> 3) & 0x01);
  joint->switches = buf[CAN_DATA + 5] & 0x07;
}

void CAN_BufferToPIDValues(uint8_t *buf, CAN_PIDValues *pids) {
  pids->Kp[0] = buf[CAN_DATA + 0];
  pids->Kp[1] = buf[CAN_DATA + 1];
  pids->Kp[2] = buf[CAN_DATA + 2];

  pids->Ki[0] = buf[CAN_DATA + 3];
  pids->Ki[1] = buf[CAN_DATA + 4];
  pids->Ki[2] = buf[CAN_DATA + 5];
}

void CAN_BufferToExtendedStatus(uint8_t *buf, CAN_ExtendedStatus *status) {
  status->addr = buf[CAN_DATA + 0];
  status->debugLevel = buf[CAN_DATA + 1];
  status->homeSpeed[0] = buf[CAN_DATA + 2];
  status->homeSpeed[1] = buf[CAN_DATA + 3];
  status->homeSpeed[2] = buf[CAN_DATA + 4];
}

void CAN_BufferToHomingStatus(uint8_t *buf, CAN_Homing *homing) {
  homing->pos = buf[CAN_DATA + 0] << 8 | buf[CAN_DATA + 1];
  homing->switches = buf[CAN_DATA + 2];
  homing->target = buf[CAN_DATA + 3];
  homing->stalls = buf[CAN_DATA + 4] << 8 | buf[CAN_DATA + 5];
}

void CAN_BufferToHomingResult(uint8_t *buf, CAN_Homing_Result *homing) {
  homing->success = buf[CAN_DATA + 0];
  homing->detail = buf[CAN_DATA + 1];
  homing->target = buf[CAN_DATA + 2];
}

/*
/Users/carolyn/Library/Arduino15/packages/arduino/tools/avrdude/6.3.0-arduino6/bin/avrdude
-C/Users/carolyn/Library/Arduino15/packages/arduino/tools/avrdude/6.3.0-arduino6/etc/avrdude.conf
-v -patmega2560 -cwiring -P/dev/cu.usbmodem145141 -b115200 -D
-Uflash:w:/var/folders/27/_ndl9qsj43l74vmkjbl8_ynr0000gn/T/arduino_build_228116/maquette.ino.hex:i
REST API;

// Limits
GET /crown/limits      // get current limits for all towers  serial request:
<T1L>, <T2L>, <T3L> <T4L> serial response {limits : {towerid: right: left:
center}} PUT /crown/limits      // set limits for towers GET /crown/savedLimits
// get saved limits PUT /crown/savedLimits // save new set of limits

#######

GET /crown/sculpture/(parameters, error state, homing state?) <T1S> <T2S> <T3S>
<T4S> GET /crown/sculpture/towers/[id] (returns position, limits, debug level,
pid, error, run stat] PUT /crown/sculpture/towers/[id]/[limits|debug|error|pid]
<T[id]S> POST /crown/sculpture/towers/[id]/joint/[id]/home # XXX status??? PUT
/crown/sculpture/towers/[id]/joint/[id]/position

GET /crown/parameters/[pid|limits]

// send homing state on the bus... which joint we're homing, switches, targets,
and values

/crown/record
/crown/playlists
/crown/playback

/crown/maquette

#######

// register listeners on the serial port
// each listener gets a chance to parse; say if it's theirs
// clean up listeners

GET /crown/savedParameters/limits/pid

POST /crown/sculpture/home?tower=
GET  /crown/sculpture/homestatus/<id>

// PID parameters
GET /crown/pid
PUT /crown/pid
GET /crown/savedPid
PUT /crown/savedPid

// Positions
GET /crown/positions
PUT /crown/positions




// Homing
POST /crown/home/<tower>/joint
GET  /crown/home/<tower> (get homing status for tower)

// Errors
GET /crown/errors

// Clear errors

// Record sequence
POST /crown/record/<tower>/start
POST /crown/record/<tower>/stop
GET  /crown/record/state
PUT  /crown/record/<recordingid> // change name
DELETE /crown/record/<recordingid>

// Playlist
POST /crown/playlists
GET  /crown/playlists
GET  /crown/playlist/<playlistId>
PUT  /crown/playlist/<playlistId>

// Playback
POST /crown/playback/<playlistId>
GET  /crown/playback
PUT  /crown/playback (pause, stop)

// Debug level. Homing speed. Dead band value. Tower Status

// Maquette
GET /crown/maquette
PUT /crown/maquette/limits
PUT /crown/maquette/mode


// Save pose data?



// ??? homing speed?
// change mode on brain thingy? (on/off) Make sure we go back to neutral on off!



// starting python here


# XXX what is the point of this again?
*/
