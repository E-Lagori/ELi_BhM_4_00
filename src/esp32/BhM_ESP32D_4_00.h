/* Header for BhM_ESP32D_4_00.h
 *
 * Copyright (C) 2018  E-LAGORi https://github.com/E-Lagori/ELi_BhM_4_00/tree/main/src/esp32
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
 * This header file describes the public API for SerialDebug.
 *
*/



#ifndef BhM_ESP32D_4_00_H
#define BhM_ESP32D_4_00_H

#include <ELi_McM_4_00.h>
#include "driver/mcpwm.h"
#include "soc/mcpwm_reg.h"
#include "soc/mcpwm_struct.h"

struct MD_4_00_Pinconfig{
  uint8_t IN1, INH;
};

class BLDC_4_00_SD{
  private:
    MD_4_00_Pinconfig pin;
    mcpwm_unit_t pwm_num;
    mcpwm_io_signals_t io;
    mcpwm_timer_t tim;
    mcpwm_operator_t opr;
    uint32_t freq;
    float maxspeed;
    float maxrate, p_speed = 0, wait;
  public:
    BLDC_4_00_SD(MD_4_00_Pinconfig p, float maxspeed = 100, float maxrate = 1, mcpwm_unit_t unt = MCPWM_UNIT_0, mcpwm_io_signals_t io = MCPWM0A, uint32_t f = 300);
    void startmotor();
    void setspeed(float);
    void stopmotor();
	void setspeed(float,bool);
    void setspeed_lin(float);
	void inh();
	void active();
};

struct BLDC_motconfig{
  uint8_t u,v,w;
  BLDC_4_00_SD *A,*B,*C;
};

class BLDC_4_00{
  private:
    BLDC_motconfig *m;
    bool inttrig;
    uint8_t refstate,comtable[6] = {1,2,3,4,5,6}, seqCW[6] = {1,3,2,6,4,5}, seqCCW[6] = {1,5,4,6,2,3};
    uint8_t n_sections, intfactor;
    unsigned long delta;
  public:
    uint8_t maxDC = 20;
         BLDC_4_00(BLDC_motconfig*, uint8_t);
    void startmotor(char);
    void brake();
    float commute_trapz(uint8_t state, float dc);
    bool isinttrig();
    void clrint();
    uint8_t getstate();
    void setcomtable(uint8_t *);
    void setseqCW(uint8_t *);
    void setseqCCW(uint8_t *);
    uint8_t nextstate(uint8_t s, bool); 
    float del_btn_int();
    float rpm_btn_int();
    friend void ARDUINO_ISR_ATTR hallisr(void *); 
};
#endif
