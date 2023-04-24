// a2d.h
#ifndef A2D_H
#define A2D_H

#include <stdint.h>

#define CURRENT_SENSE_CHANNEL 0
#define STEERING_FEEDBACK_POT 1

#define IMU_Gyro_X_CHANNEL 2
#define IMU_Gyro_Y_CHANNEL 3
#define IMU_VREF_CHANNEL 4
#define IMU_Accel_Z_CHANNEL 5
#define IMU_Accel_Y_CHANNEL 6
#define IMU_Accel_X_CHANNEL 7

#define IMU_FIRST_CHANNEL 2
#define IMU_LAST_CHANNEL 7

void A2D_Init(void);
void A2D_poll_adc(void);
uint16_t A2D_read_channel(uint8_t chanNum);

#endif // A2D_H
