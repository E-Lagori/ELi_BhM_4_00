
/*****************************************
 * Library   : BhM_ESP32D_4_00.cpp - Library for BLDC motor driver for E-LAGORi.
 * Programmer: Anish Bekal
 * Comments  : This library is to use with BLDC motor driver with hall effect sensor inputs from E-Lagori
 * Versions  :
 * ------ 	---------- 		-------------------------
 * 0.1.0  	2023-01-11		First beta
 *****************************************/

/*
 * Source for BhM_ESP32D_4_00
 *
 * Copyright (C) 2023  Anish Bekal https://www.e-lagori.com/product/bldc-driver-with-hall-effect-sensor-input/
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, version 3.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * This file contains the code for ESP32MotorControl library.
 *
 */

/*
 * TODO list:
 * - Stepper motor drive using MCPWM 
 */

/*
 * TODO known issues:
 */
#include <ELi_BhM_4_00.h>
#include <ELi_McM_4_00.h>

#include "driver/mcpwm.h"
#include "soc/mcpwm_reg.h"
#include "soc/mcpwm_struct.h"
#include <Arduino.h>

BLDC_4_00_SD::BLDC_4_00_SD(MD_4_00_Pinconfig p, float maxspeed, float maxrate, mcpwm_unit_t unt, mcpwm_io_signals_t io, uint32_t f){
  mcpwm_config_t pwm_conf;
  this->pin = p;
  this->freq = f;
  this->wait = 1/float(this->freq) *1000.0; //in ms
  this->pwm_num = unt;
  this->maxspeed = maxspeed;
  this->p_speed = 0;
  this->maxrate = maxrate;
  this->io = io;
  
  switch(this->io){
    case MCPWM0A: this->tim = MCPWM_TIMER_0;this->opr = MCPWM_OPR_A;pwm_conf.cmpr_a = 0;break;
    case MCPWM0B: this->tim = MCPWM_TIMER_0;this->opr = MCPWM_OPR_B;pwm_conf.cmpr_b = 0;break;
    case MCPWM1A: this->tim = MCPWM_TIMER_1;this->opr = MCPWM_OPR_A;pwm_conf.cmpr_a = 0;break;
    case MCPWM1B: this->tim = MCPWM_TIMER_1;this->opr = MCPWM_OPR_B;pwm_conf.cmpr_b = 0;break;
    case MCPWM2A: this->tim = MCPWM_TIMER_2;this->opr = MCPWM_OPR_A;pwm_conf.cmpr_a = 0;break;
    case MCPWM2B: this->tim = MCPWM_TIMER_2;this->opr = MCPWM_OPR_B;pwm_conf.cmpr_b = 0;break;
  };
  pwm_conf.frequency = this->freq;
  pwm_conf.counter_mode = MCPWM_UP_COUNTER;
  pwm_conf.duty_mode = MCPWM_DUTY_MODE_0;
  pinMode(this->pin.INH,OUTPUT);
  mcpwm_gpio_init(this->pwm_num, this->io, this->pin.IN1);
  mcpwm_init(this->pwm_num, this->tim, &pwm_conf);
}

void BLDC_4_00_SD::startmotor(){
  digitalWrite(this->pin.INH,true);
  mcpwm_start(this->pwm_num,this->tim);
  this->setspeed(this->p_speed);
}

void BLDC_4_00_SD::setspeed(float speed){
  float temp = ((abs(speed))/this->maxspeed);
  mcpwm_set_duty(this->pwm_num, this->tim, this->opr, ((temp < 0)?0:temp) * 100);
}

void BLDC_4_00_SD::stopmotor(){
	digitalWrite(this->pin.INH,false);
  this->setspeed_lin(0);
   mcpwm_set_signal_low(this->pwm_num, this->tim, this->opr);
   mcpwm_stop(this->pwm_num,this->tim);
}

void BLDC_4_00_SD::setspeed_lin(float sp){
  float temp_speed = sp - this->p_speed;
  float iter = (abs(temp_speed)/this->maxspeed)*(this->maxrate/this->wait) * 100;
  int int_iter = iter;
  float del_pwm = temp_speed/int_iter;
  float extr_wait = fmod(iter, 1)*this->wait;
  float pwm = this->p_speed;
  for(int i=0; i<int_iter; i++){
      pwm += del_pwm;
      this->setspeed(pwm);
      delay(this->wait);
  }
  if (extr_wait > 1)
  	delay(extr_wait);
  this->setspeed(sp);
  this->p_speed=sp;
}
void BLDC_4_00_SD::inh(){
  digitalWrite(this->pin.INH,LOW);
}

void BLDC_4_00_SD::active(){
  digitalWrite(this->pin.INH,HIGH);
}

/*---------------------------------------------------*/
void ARDUINO_ISR_ATTR hallisr(void *arg) {
  static unsigned long prevtime = 0;
  BLDC_4_00 *b = static_cast<BLDC_4_00*>(arg);
  if (!b->inttrig){
    b->inttrig = 1; 
    b->delta = micros() - prevtime;
    prevtime = micros();
  }    
}

BLDC_4_00::BLDC_4_00(BLDC_motconfig *m, uint8_t npoles = 8){
  this->m = m;
  if (this->m->u) pinMode(this->m->u,INPUT);
  if (this->m->v) pinMode(this->m->v,INPUT);
  if (this->m->w) pinMode(this->m->w,INPUT);
  this->n_sections = npoles >> 1;
//  pinMode(this->m->A.inh_,OUTPUT);
//  pinMode(this->m->B.inh_,OUTPUT);
//  pinMode(this->m->C.inh_,OUTPUT);
  this->m->A->inh();
  this->m->B->inh();
  this->m->C->inh();
}

void BLDC_4_00::startmotor(char intpin = 'A')
{
  this->inttrig = 0;
  this->m->A->startmotor();
  this->m->B->startmotor();
  this->m->C->startmotor();
  this->m->A->setspeed_lin(0);
  this->m->B->setspeed_lin(0);
  this->m->C->setspeed_lin(0);
  switch (intpin){
    case 'u': 
              attachInterruptArg(this->m->u,hallisr,this,CHANGE);
              this->intfactor = 2;
              break;
    case 'v': attachInterruptArg(this->m->v,hallisr,this,CHANGE);
              this->intfactor = 2;
              break;
    case 'w': attachInterruptArg(this->m->w,hallisr,this,CHANGE);
              this->intfactor = 2;
              break;
    case 'A': attachInterruptArg(this->m->u,hallisr,this,CHANGE);
              attachInterruptArg(this->m->v,hallisr,this,CHANGE);
              attachInterruptArg(this->m->w,hallisr,this,CHANGE);
              this->intfactor = 6;
              break;
    default:  this->intfactor = 6;
              break;
  } 
}
void BLDC_4_00::brake(){
  this->m->A->inh();
  this->m->B->inh();
  this->m->C->inh();
}
float BLDC_4_00::commute_trapz(uint8_t state, float dc){
    bool l = 0;
    if (dc > this->maxDC)
      dc = maxDC;
    if (dc > 100)
      dc = 100;
    if (dc < 0)
      dc = 0;
    this->m->A->inh();
    this->m->B->inh();
    this->m->C->inh();
    //1,2,3,4,5,6
    if (state == this->comtable[2]){ //3 
      this->m->C->setspeed(0);
      this->m->B->setspeed(dc);
      this->m->A->inh();
      this->m->C->active();
      this->m->B->active();
      return(dc);
    }
    if (state == this->comtable[5]) { //6
      this->m->B->setspeed(0);
      this->m->A->setspeed(dc);
      this->m->C->inh();
      this->m->B->active();
      this->m->A->active();
      return(dc);
    }
    if (state == this->comtable[1]){ //2
      this->m->C->setspeed(0);
      this->m->A->setspeed(dc);
      this->m->B->inh();
      this->m->C->active();
      this->m->A->active();
      return(dc);
    }
    if (state == this->comtable[4]) { //5
      this->m->A->setspeed(0);
      this->m->C->setspeed(dc);
      this->m->B->inh();
      this->m->A->active();
      this->m->C->active();
      return(dc);
    }
    if (state == this->comtable[0]){ //1
      this->m->A->setspeed(0);
      this->m->B->setspeed(dc);
      this->m->C->inh();
      this->m->A->active();
      this->m->B->active();
      return(dc);
    }
    if (state == this->comtable[3]) { //4
      this->m->B->setspeed(0);
      this->m->C->setspeed(dc);
      this->m->A->inh();
      this->m->B->active();
      this->m->C->active();
      return(dc);
    }
}

bool BLDC_4_00::isinttrig(){
  return (this->inttrig);
}
void BLDC_4_00::clrint(){
  this->inttrig = 0;
}
uint8_t BLDC_4_00::getstate(){
  uint8_t s = 0;
  s = (s << 1) | digitalRead(this->m->u);
  s = (s << 1) | digitalRead(this->m->v);
  s = (s << 1) | digitalRead(this->m->w);
  return s;
}
uint8_t BLDC_4_00::nextstate(uint8_t s, bool dir){
  uint8_t *seq = dir?seqCW:seqCCW;
  uint8_t i;
  for (i = 0; i<6; i++)
    if (seq[i] == s)
      break;
  return seq[(i+1)%6];
}
void BLDC_4_00::setcomtable(uint8_t *com){
  for (int i = 0; i<6; i++)
    this->comtable[i] = com[i];
}
void BLDC_4_00::setseqCW(uint8_t *sCW){
  for (int i = 0; i<6; i++)
    this->seqCW[i] = sCW[i];
}
void BLDC_4_00::setseqCCW(uint8_t *sCCW){
  for (int i = 0; i<6; i++)
    this->seqCCW[i] = sCCW[i];
}
    
float BLDC_4_00::rpm_btn_int(){
  return (60/(this->n_sections*this->intfactor*this->delta*1e-6));
}

float BLDC_4_00::del_btn_int(){
  return (this->delta);
}
