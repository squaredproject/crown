// jointPID.h
#ifndef JOINTPID_H
#define JOINTPID_H

#define FORWARD 1
#define REVERSE -1


// ----------------------------------------------------------------------
// Parameters ofr joint control PID
typedef struct {
  int8_t	id;	            /* joint number (for debugging, etc) */
  int8_t	Kp;	            /* proportional PID term */
  int8_t	Ki;			    /* integral PID term */
  int8_t	Kd;			    /* derivative PID term */
  int16_t targetPos;		/* where we want to go */
  int16_t currentPos;		/* where we are now */
  uint8_t drive;	        /* the pwer we are giving it now */
  uint8_t homespeed;	    /* the homing speed */
  int16_t dead_band;	    /* close enough to desired speed if within */
  int16_t lastError;	    /* the error we had last iteration */
  uint8_t lastVal;	        /* the power we gave it last iteration */
  int16_t integrator;  		/* integrator value */
  int16_t intLimit;  		/* integrator limit */
  int16_t center;           /* center value: subtract this from raw pos */
  int8_t  dmin;	            /* limit drive to this min value */
  int8_t  dmax;	            /* limit drive to this max value */
  int16_t minpos;			/* value at left limit sw */
  int16_t maxpos;			/* value at right limit sw */
  uint8_t trace;			/* trace value for online tuning */
  uint8_t direction;		/* non-zero to INVERT sense of PID */
  uint8_t switches;		    /* last measured value of limit switches */
  uint8_t homed;		    /* set to 1 if we are homed (center = OK) */
  uint8_t sw_enabled;          /* software enable/disable. There is also a physical switch that we also check */  
} joint_control_block;


void PID_Init(void);
void Joint_Init_JCB( joint_control_block *j,uint8_t id);
void Print_Joint_Status(joint_control_block *j);
void Print_all_status(joint_control_block *j);
void Home_Joint(joint_control_block *j);
uint8_t Joint_Servo_Task(joint_control_block *j);
void Poll_Limit_Switches(void);
void Dump_JCB(joint_control_block *j);
void Neuter_Joint(joint_control_block *j);
//int16_t limit(int16_t *v, int16_t minVal, int16_t maxVal);

#define LEFT_SW (0x01) 			/* limit switch masks */
#define HOME_SW (0x02)
#define RIGHT_SW (0x04)

#endif // JOINTPID_H
