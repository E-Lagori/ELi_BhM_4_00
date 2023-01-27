#include <ELi_McM_4_00.h>
#include <ELi_BhM_4_00.h>
#include <Arduino.h>
#include <PID.h>

#define PWMFREQ 1000
#define FLEN 3


struct MD_4_00_Pinconfig p1 = {.IN1 = CtrlBUS_AF,.INH = CtrlBUS_AC};
struct MD_4_00_Pinconfig p2 = {.IN1 = CtrlBUS_AE,.INH = CtrlBUS_AB};
struct MD_4_00_Pinconfig p3 = {.IN1 = CtrlBUS_AD,.INH = CtrlBUS_AA};

BLDC_4_00_SD mot1(p1, 100, 0, MCPWM_UNIT_0,MCPWM0A,PWMFREQ);
BLDC_4_00_SD mot2(p2, 100, 0, MCPWM_UNIT_0,MCPWM0B,PWMFREQ);
BLDC_4_00_SD mot3(p3, 100, 0, MCPWM_UNIT_0,MCPWM1A,PWMFREQ);

struct BLDC_motconfig m = {.u = CtrlBUS_Q, .v = CtrlBUS_R, .w = CtrlBUS_T,
                           .A =  &mot1,
                           .B =  &mot2, 
                           .C =  &mot3
                          };

BLDC_4_00 b(&m,8);
PIDcntrl p;
float maxDC = 1, spset = 120, stDCmax = 10;
bool startpulse, rot, stamotor = 0, dir = 1;
unsigned long prevmaintim;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);//(2000000);
  // PID Initial parameters
  p.K = 0.01; p.Ti = 1e3; p.Td = 0; 
  p.Tset = spset;
  p.vmax = 15;
  p.vmin = -15; 
  
  maxDC = 0;
  startpulse = 1;
  rot = 0;
  b.maxDC = 90;
  b.startmotor('A');
  stamotor = 0;
  Serial.println("---------------------------------------");
  Serial.print("Direction: "); Serial.println((dir == 0)? "CCW":"CW");
  Serial.print("PID K: "); Serial.println(p.K);
  Serial.print("PID Ki: "); Serial.println(p.Ti);
  Serial.print("PID Vmax: "); Serial.println(p.vmax);
  Serial.print("PID Vmin: "); Serial.println(p.vmin);
  Serial.print("Speed: "); Serial.println(p.Tset);
  Serial.print("Max PWM: "); Serial.println(b.maxDC);  
  Serial.println("Press A1 to start the motor");
  prevmaintim = millis();
}

void loop() {
  // put your main code here, to run repeatedly:
  static uint8_t s,i, refstate = 001,sp_index = 0;
  static bool ctrl = 1, dirtemp = 1;
  static float T = 2, sp, sparray[FLEN], spfil;
  static unsigned long period, sptime,spdelta;
  static unsigned long secount, NRcount, NRcntupd = 0;
//  Serial.println(b.isinttrig());

  if(Serial.available())
  {
    char inByte = Serial.read();
    if (inByte != '\n'){
      float infloat = Serial.parseFloat();
      switch(inByte){
        case 'M': maxDC = infloat; break; // PWM value
        case 'D': dir = (infloat<0)?0:1;dirtemp = dir;break;
        case 'P': p.K = infloat;break;
        case 'I': p.Ti = infloat;break;
        case 'V': p.vmax = infloat;p.vmin = -infloat;break;
        case 'S': spset = infloat;p.Tset += (infloat - p.Tset)/T;break;
        case 'U': b.maxDC = infloat;break;
        case 'L': stDCmax = infloat;break;
        case 'T': T = abs(infloat);break;
        case 'C': ctrl = bool(infloat);break;
        case 'r': p.reset();break;
        case 'R': b.brake();break;
        case 'A': stamotor = (infloat<0.5)?0:1; break;
        case 'O': Serial.println("---------------------------------------");
                  Serial.print("Direction: "); Serial.println((dir == 0)? "CCW":"CW");
                  Serial.print("PID K: "); Serial.println(p.K);
                  Serial.print("PID Ki: "); Serial.println(p.Ti);
                  Serial.print("PID Vmax: "); Serial.println(p.vmax);
                  Serial.print("PID Vmin: "); Serial.println(p.vmin);
                  Serial.print("Speed: "); Serial.println(p.Tset);
                  Serial.print("Max PWM: "); Serial.println(b.maxDC);
                  break;
      }
    }
  }
  if (stamotor){
      if (b.isinttrig()||startpulse){
        if (b.isinttrig()){
          prevmaintim = millis();
          startpulse = 0;
          period+=b.del_btn_int();
          if(refstate == b.getstate())
            secount++;
          NRcount = 0;
        }
        s = b.getstate();
        maxDC = b.commute_trapz(b.nextstate(s,dir),maxDC);//,maxDC);//
        b.clrint();
      }
      if (secount == 4){
        secount = 0;
        rot = 1;NRcount = 0;
        dir = dirtemp;
        sp = 1.0/period*1e6*60;
        sparray[sp_index] = sp;
        sp_index = (sp_index+1)%FLEN;
        spfil = 0;
        for (int c = 0; c<FLEN; c++)
          spfil += sparray[c];
        spfil = spfil/FLEN;
        if (ctrl) 
          maxDC += p.evalCntrl(sp,float(period)*1e-6);
        else
          p.reset();
        if ((abs(p.Tset-spfil)/p.Tset*100) < 1){
          p.Tset += (spset - p.Tset)/T; 
        }
        Serial.print(dir); Serial.print(' ');
        Serial.print(maxDC,3); Serial.print(' ');
//        Serial.print((abs(p.Tset-sp)/p.Tset*100),5); Serial.print(' ');
        Serial.print(p.Tset,3); Serial.print(' ');
//        Serial.print(spset); Serial.print(' ');
//        Serial.print(dirtemp); Serial.print(' ');
//        Serial.print(sp); Serial.print(' ');
        Serial.println(spfil,3);
        period = 0;
      }
    
      if ((millis() - prevmaintim) > 100){
        if (rot)
          dirtemp = dir;
        rot = 0;
        prevmaintim = millis();
        NRcount++;
        startpulse = 1;
      }
    
      if (((NRcount%3) == 1) && (NRcntupd != NRcount)){
        NRcntupd = NRcount;
//        Serial.print(NRcount); Serial.print(' ');
        Serial.print(dir); Serial.print(' ');
        Serial.print(maxDC); Serial.print(' ');
        Serial.println("NR");
        maxDC+=1;
        if (maxDC > stDCmax){
          maxDC = stDCmax;
          dir = 1;
        }
      }
  }    
  else {
    b.brake();
    maxDC = 0;
  }

}
