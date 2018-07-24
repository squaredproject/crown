// timer.h

void Timer0_Init(void);
void Timer0_reset(void);
void PWM_Init(void);
void ICP3_Init(void);
void ICP4_Init(void);
void ICP5_Init(void);

/*
#define CHA_PWM(x) OCR3AH=0; OCR3AL=(uint8_t)(x)
#define CHB_PWM(x) OCR3BH=0; OCR3BL=(uint8_t)(x)
#define CHC_PWM(x) OCR3CH=0; OCR3CL=(uint8_t)(x)
*/

#define TIMER2_PWM(x)  OCR2B=(uint8_t)(x)

