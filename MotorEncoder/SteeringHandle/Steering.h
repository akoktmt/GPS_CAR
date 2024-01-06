/*
 * Steering.h
 *
 *  Created on: Dec 27, 2023
 *      Author: win 10
 */

#ifndef STEERING_H_
#define STEERING_H_

#define Tult 0.5
#define KP (0.6*4.501)
#define KI (2*KP/Tult)
#define KD (KP * Tult/8)

#define LEFTLIMIT 86
#define RIGHTLIMIT 61
#define STRAIGHTANGLE 73

#include "tim.h"
#include "main.h"
void Steering_Control(uint8_t Pwm);
double convertAngle(double oldAngle);
void Steering_PIDControl(volatile uint8_t rcdata[],double *Steering , double *SteeringControl , double *SPWM , double *SteeOut);
#endif /* STEERING_H_ */
