/*
 * Steering.c
 *
 *  Created on: Dec 27, 2023
 *      Author: win 10
 */
#include "Steering.h"
#include "PID.h"
void Steering_PIDControl(volatile uint8_t rcdata[],double *Steering , double *SteeringControl , double *SPWM , double *SteeOut){
	 	 	 double SteeringAngle;
	 	 	 *Steering=(double)rcdata[0]/255;
			  SteeringAngle=*Steering*360;
			 * SteeringControl= convertAngle(SteeringAngle);
		  	 * SPWM-=*SteeOut;
		  	  if(*SPWM>LEFTLIMIT){
		  		*SPWM=LEFTLIMIT;
		  	  }
		  	  if(*SPWM<RIGHTLIMIT)
		  	  {
		  		*SPWM=RIGHTLIMIT;
		  	  }
}

void Steering_Control(uint8_t Pwm){
	if(Pwm<RIGHTLIMIT){
		Pwm=RIGHTLIMIT;
	}
	if(Pwm>LEFTLIMIT){
		Pwm=LEFTLIMIT;
	}
	  __HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_1,Pwm);
}

double convertAngle(double oldAngle) {
  float newAngle;

  if (oldAngle >= 0 && oldAngle <= 180) {
    newAngle = oldAngle;
  }
  else {
    newAngle = oldAngle - 360;
  }

  return newAngle;
}


