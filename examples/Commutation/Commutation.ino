#include <ELi_McM_4_00.h>
#include <ELi_BhM_4_00.h>
#include <Arduino.h>
#include <PID.h>

#define PWMFREQ 1000

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

float maxDC = 1;
void setup() {
  // put your setup code here, to run once:
  // put your setup code here, to run once:
  Serial.begin(2000000);
  maxDC = 10;
  Serial.println("End of setup");
}

void loop() {
  // put your main code here, to run repeatedly:
  uint8_t s = b.getstate();
  Serial.print(s,BIN); Serial.print(' ');
  Serial.println(s);
  if(Serial.available())
  {
    char inByte = Serial.read();
    if (inByte != '\n'){
      float infloat = Serial.parseFloat();
      switch(inByte){
        case 'M': maxDC = infloat; break;
        default : b.commute_trapz(inByte-'0',maxDC);
      }
    }
  } 
}
