/*  

Library:     pwm01.h (version 01)
Date:        2/11/2013
Written By:  randomvibe  

Purpose:     
   Setup one or two unique PWM frequenices directly in sketch program,
   set PWM duty cycle, and stop PWM function.

User Functions:     
   pwm_set_resolution(int res) ~  setup PWM resolution; up to 16 bit
   pwm_setup( uint32_t pwm_pin, uint32_t pwm_freq, int iclock) ~ Setup PWM on clock-A (iclock=1) or clock-B (iclock-2)
   pwm_write_duty( uint32_t pwm_pin, uint32_t pwm_duty) ~ Write PWM duty cycle
   pwm_stop( uint32_t pwm_pin) ~ Force PWM stop

Notes:
   - Applies to Arduino-Due board, PWM pins 6, 7, 8 & 9.
   - Libary Does not operate on the TIO pins.
   - Unique frequencies set via PWM Clock-A ("CLKA") and Clock-B ("CLKB")
     Therefore, up to two unique frequencies allowed.
   - Set max duty cycle counts (pwm_max_duty_Ncount) equal to 255
     per Arduino approach.  This value is best SUITED for low frequency
     applications (2hz to 40,000hz) such as PWM motor drivers, 
     38khz infrared transmitters, etc.
   - Future library versions will address high frequency applications.
   - Arduino's "wiring_analog.c" function was very helpful in this effort.
      
*/

#ifdef __arm__

#ifndef PWM01_H
#define PWM01_H

#include "Arduino.h"

extern int       pwm_resolution_nbit;
extern uint32_t  pwm_clockA_freq;
extern uint32_t  pwm_clockB_freq;
extern uint32_t  pwm_max_duty_Ncount;

void pwm_set_resolution(int res);

void pwm_set_clockA_freq(uint32_t  val);

void pwm_set_clockB_freq(uint32_t  val);

static inline uint32_t mapResolution(uint32_t value, uint32_t from, uint32_t to);

// MAIN PWM INITIALIZATION
//--------------------------------
void  pwm_setup( uint32_t  pwm_pin,  uint32_t  pwm_freq,  int iclock  );

// WRITE DUTY CYCLE
//--------------------------------
void  pwm_write_duty( uint32_t  pwm_pin,  uint32_t  pwm_duty );

// FORCE PWM STOP
//--------------------------------
void  pwm_stop( uint32_t  pwm_pin ) ;

#endif

#endif
