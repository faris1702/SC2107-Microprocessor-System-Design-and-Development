// RSLK Self Test via UART

/* This example accompanies the books
   "Embedded Systems: Introduction to the MSP432 Microcontroller",
       ISBN: 978-1512185676, Jonathan Valvano, copyright (c) 2017
   "Embedded Systems: Real-Time Interfacing to the MSP432 Microcontroller",
       ISBN: 978-1514676585, Jonathan Valvano, copyright (c) 2017
   "Embedded Systems: Real-Time Operating Systems for ARM Cortex-M Microcontrollers",
       ISBN: 978-1466468863, , Jonathan Valvano, copyright (c) 2017
 For more information about my classes, my research, and my books, see
 http://users.ece.utexas.edu/~valvano/

Simplified BSD License (FreeBSD License)
Copyright (c) 2017, Jonathan Valvano, All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice,
   this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are
those of the authors and should not be interpreted as representing official
policies, either expressed or implied, of the FreeBSD Project.
*/
//imports
#include "msp.h"
#include <stdint.h>
#include <string.h>
#include "..\inc\UART0.h"
#include "..\inc\EUSCIA0.h"
#include "..\inc\FIFO0.h"
#include "..\inc\Clock.h"
//#include "..\inc\SysTick.h"
#include "..\inc\SysTickInts.h"
#include "..\inc\CortexM.h"
#include "..\inc\TimerA1.h"
//#include "..\inc\Bump.h"
#include "..\inc\BumpInt.h"
#include "..\inc\LaunchPad.h"
#include "..\inc\Motor.h"
#include "../inc/IRDistance.h"
#include "../inc/ADC14.h"
#include "../inc/LPF.h"
#include "..\inc\Reflectance.h"
#include "../inc/TA3InputCapture.h"
#include "../inc/Tachometer.h"
#include "..\inc\TExaS.h"

#define P2_4 (*((volatile uint8_t *)(0x42098070)))
#define P2_3 (*((volatile uint8_t *)(0x4209806C)))
#define P2_2 (*((volatile uint8_t *)(0x42098068)))
#define P2_1 (*((volatile uint8_t *)(0x42098064)))
#define P2_0 (*((volatile uint8_t *)(0x42098060)))





//IR SENSOR--------------------------------------
volatile uint32_t ADCvalue;
volatile uint32_t ADCflag;
volatile uint32_t nr, nc, nl;

void SensorRead_ISR(void)
{  // runs at 2000 Hz
    uint32_t raw17, raw12, raw16;
    P1OUT ^= 0x01;         // profile
    P1OUT ^= 0x01;         // profile
    ADC_In17_12_16(&raw17, &raw12, &raw16);  // sample
    nr = LPF_Calc(raw17);  // right is channel 17 P9.0
    nc = LPF_Calc2(raw12);  // center is channel 12, P4.1
    nl = LPF_Calc3(raw16);  // left is channel 16, P9.1
    ADCflag = 1;           // semaphore
    P1OUT ^= 0x01;         // profile
}
void IRSensor_Init(void)
{
    uint32_t raw17, raw12, raw16;
    uint32_t s;
    ADCflag = 0;
    s = 256; // replace with your choice
    ADC0_InitSWTriggerCh17_12_16();   // initialize channels 17,12,16
    ADC_In17_12_16(&raw17, &raw12, &raw16);  // sample
    LPF_Init(raw17, s);     // P9.0/channel 17
    LPF_Init2(raw12, s);     // P4.1/channel 12
    LPF_Init3(raw16, s);     // P9.1/channel 16
    UART0_Init();          // initialize UART0 115,200 baud rate
    LaunchPad_Init();
    TimerA1_Init(&SensorRead_ISR, 250);    // 2000 Hz sampling
}

// Reset function
void RSLK_Reset(void){
    DisableInterrupts();
    Clock_Init48MHz();

    LaunchPad_Init();
    //Initialise modules used e.g. Reflectance Sensor, Bump Switch, Motor, Tachometer etc
    Motor_Init();
    IRSensor_Init();


    EnableInterrupts();
}

//Bump Switch-------------------------------------

volatile uint8_t CollisionData, CollisionFlag;
int count, flag;
void HandleCollision(uint8_t bumpSensor){
//    Motor_Stop();
    CollisionData = bumpSensor;
    CollisionFlag = 1;

    //password checker
//    if (count == 0){
//        if ((CollisionData & 0x3f) == 0b00011111){  //6
//            count++;
//            UART0_OutString("   Correct\r\n");
//        }
//        else{
//            count = 0;
//            UART0_OutString("   Wrong\r\n");
//        }
//    }
//    else if (count == 2){
//            if ((CollisionData & 0x3f) == 0b00111101){ //2
//                count++;
//                UART0_OutString("   Correct\r\n");
//            }
//            else{
//                count = 0;
//                UART0_OutString("   Wrong\r\n");
//            }
//        }
//    else if (count == 4){
//            if ((CollisionData & 0x3f) == 0b00111011){ //3
//                count++;
//                UART0_OutString("   Correct\r\n");
//            }
//            else{
//                count = 0;
//                UART0_OutString("   Wrong\r\n");
//            }
//        }
//    else if (count == 6){
//            if ((CollisionData & 0x3f) == 0b00111110){//1
//                flag = 1;
//                UART0_OutString("   Correct\r\n");
//            }
//            else{
//                count = 0;
//                UART0_OutString("   Wrong\r\n");
//            }
//        }
//    else{
//        count++;
//    }

    P4->IFG &= ~0xED;
}

void MotorMovt(void){
    static uint32_t count=0;
    static uint8_t motor_state=0;

    switch (motor_state){
       case 0:
           Motor_Forward(3000,3000);
           if (CollisionFlag == 1){
               Motor_Stop();
               motor_state = 1;
               count++;
           }
           if (count == 15)
               Motor_Stop();
           break;
       case 1:
           Motor_Backward(3000,3000);
           Clock_Delay1ms(500);
           Motor_Right(3000,3000);
           Clock_Delay1ms(1400);
           CollisionFlag = 0;
           motor_state = 0;
           break;
       }
}

uint8_t ConvertCollisionData(uint8_t data){
    return data&0x3f;
}
//-------------------------------------
//Tacho
uint16_t Period0;              // (1/SMCLK) units = 83.3 ns units
uint16_t First0=0;             // Timer A3 first edge, P10.4
uint32_t Done0=0;              // set each rising

uint16_t Period2;              // (1/SMCLK) units = 83.3 ns units
uint16_t First2=0;             // Timer A3 first edge, P8.2
uint32_t Done2=0;              // set each rising
void PeriodMeasure0(uint16_t time){
  Period0 = (time - First0)&0xFFFF; // 16 bits, 83.3 ns resolution
  First0 = time;                    // setup for next
  Done0++;
}

// max period is (2^16-1)*83.3 ns = 5.4612 ms
// min period determined by time to run ISR, which is about 1 us
void PeriodMeasure2(uint16_t time){
  Period2 = (time - First2)&0xFFFF; // 16 bits, 83.3 ns resolution
  First2 = time;                    // setup for next
  Done2++;
}

void TimedPause(uint32_t time){
  Clock_Delay1ms(time);         // run for a while and stop
  Motor_Stop();
  while(LaunchPad_Input()==0);  // wait for touch
  while(LaunchPad_Input());     // wait for release
}

//Initialise GPIO Port1 registers
void Port1_Init(void){
  P1->SEL0 = 0x00;
  P1->SEL1 = 0x00;                        // configure P1.4 and P1.1 as GPIO
  P1->DIR = 0x01;                         // make P1.4 and P1.1 in, P1.0 output
  P1->REN = 0x12;                         // enable pull resistors on P1.4 and P1.1
  P1->OUT = 0x12;                         // P1.4 and P1.1 are pull-up
}

//Read Port1 input data register
uint8_t Port1_Input(void){
  return (P1->IN&0x12);                   // read P1.4,P1.1 inputs
}

//Initialise GPIO Port2 registers
void Port2_Init(void){
  P2->SEL0 = 0x00;
  P2->SEL1 = 0x00;                        // configure P2.2-P2.0 as GPIO
  P2->DS = 0x07;                          // make P2.2-P2.0 high drive strength
  P2->DIR = 0x07;                         // make P2.2-P2.0 out
  P2->OUT = 0x00;                         // all LEDs off
}

//Output data to Port1 GPIO pins by writing to Port1 output data register
void Port1_Output(uint8_t data){        // write all of P1.0 outputs
  P1->OUT = (P1->OUT&0xFE)|data;
}

//Output data to Port2 GPIO pins by writing to Port2 output data register
void Port2_Output(uint8_t data){        // write all of P2 outputs
  P2->OUT = data;
}
#define RED       0x01
#define GREEN     0x02
#define BLUE      0x04
//--------------------------------

// RSLK Self-Test
// Sample program of how the text based menu can be designed.
// Only one entry (RSLK_Reset) is coded in the switch case. Fill up with other menu entries required for Lab5 assessment.
// Init function to various peripherals are commented off.  For reference only. Not the complete list.

int main(void) {
  uint32_t cmd=0xDEAD, menu=0;

  DisableInterrupts();
  Clock_Init48MHz();  // makes SMCLK=12 MHz
  //SysTick_Init(48000,2);  // set up SysTick for 1000 Hz interrupts
//   Motor_Forward(2000,2000);
//   Clock_Delay1ms(1000);
  //Motor_Stop();
   LaunchPad_Init();
  //Bump_Init_Edge();
  //Bumper_Init();
  BumpInt_Init(&HandleCollision);
//   Bump_Init();
  IRSensor_Init();
  //Tachometer_Init();
  Port1_Init();
    Port2_Init();
  EUSCIA0_Init();     // initialize UART
  Motor_Init();
  EnableInterrupts();

  while(1){                     // Loop forever
      // write this as part of Lab 5
      EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);
      EUSCIA0_OutString("RSLK Testing"); EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);
      EUSCIA0_OutString("[0] RSLK Reset"); EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);
      EUSCIA0_OutString("[1] Motor Test"); EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);
      EUSCIA0_OutString("[2] IR Sensor Test"); EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);
      EUSCIA0_OutString("[3] Bumper Test"); EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);
      EUSCIA0_OutString("[4] Reflectance Sensor Test"); EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);
      EUSCIA0_OutString("[5] Tachometer Test"); EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);
      EUSCIA0_OutString("[6] IR Dead Recognition Test"); EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);
      EUSCIA0_OutString("[7] Bumper Dead Recognition Test"); EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);
      EUSCIA0_OutString("[8] Blinking when black surface"); EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);
      EUSCIA0_OutString("[9] LED based on distance"); EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);
      EUSCIA0_OutString("[10] Press switch, motor progressively faster"); EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);
      EUSCIA0_OutString("[11] Hit front 2 bumper, stop reverse"); EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);
      EUSCIA0_OutString("[12] Parking at black line"); EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);
      EUSCIA0_OutString("[13] Follow black line"); EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);
      EUSCIA0_OutString("[14] Password + move around object"); EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);

      EUSCIA0_OutString("CMD: ");
      cmd=EUSCIA0_InUDec();
      EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);

      switch(cmd){
//[0] RSLK Reset---------------------------------------------------------------
          case 0:
              EUSCIA0_OutChar(CR);EUSCIA0_OutChar(LF);
              EUSCIA0_OutString("Resetting ......");
              EUSCIA0_OutChar(CR);EUSCIA0_OutChar(LF);
              RSLK_Reset(); //call reset function

              menu =1;
              cmd=0xDEAD;
              break;

              // ....
//[1] Motor Test-----------------------------------------------------------------------------
          case 1:
              EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);
              EUSCIA0_OutString("Motor Testing running: "); EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);
              EUSCIA0_OutString("[0] Go Forward"); EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);
              EUSCIA0_OutString("[1] Go Backward"); EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);
              EUSCIA0_OutString("[2] Turn Left"); EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);
              EUSCIA0_OutString("[3] Turn Right"); EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);


              EUSCIA0_OutString("choice: ");
              uint32_t choice =EUSCIA0_InUDec();
              EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);
              switch(choice){
                  case 0:
                      Motor_Forward(3000,3000);   //both wheels forward
                      Clock_Delay1ms(1000);
                      Motor_Stop();
                      break;
                  case 1:
                      Motor_Backward(3000,3000);   //both wheels backwards
                      Clock_Delay1ms(1000);
                      Motor_Stop();
                      break;

                  case 2:
                      Motor_Left(1500,1500);    //only right wheel forward
                      Clock_Delay1ms(1300);
                      Motor_Stop();
                      break;

                  case 3:
                      Motor_Right(1500,1500);    //only left wheel forward
                      Clock_Delay1ms(1300);
                      Motor_Stop();
                      break;
                  default:
                      break;
              }
              menu = 1;
              cmd=0xDEAD;
              break;
//[2] IR Sensor test-----------------------------------------------------------------------------
      case 2:
          EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);
          EUSCIA0_OutString("IR Sensor testing ......"); EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);

          int32_t n;
          Clock_Init48MHz();  //SMCLK=12Mhz

          EnableInterrupts();
          for(int i = 0; i<20; i++)
          {
          for(n=0; n<2000; n++){
                while(ADCflag == 0){};
                ADCflag = 0; // show every 2000th point
              }
          UART0_OutUDec5(LeftConvert(nl));UART0_OutString(" mm,");
          UART0_OutUDec5(CenterConvert(nc));UART0_OutString(" mm,");
          UART0_OutUDec5(RightConvert(nr));UART0_OutString(" mm\r\n");
          }
          menu = 1;
          cmd=0xDEAD;
          break;
//[3] Bumper Test-------------------------------------------------------------------------------------------------------------------------------
      case 3:
          EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);
          EUSCIA0_OutString("Bump Switches Testing ......"); EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);
          EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);

//          while (1){
//              WaitForInterrupt();
//              UART0_OutUDec5(password);
//          }

          CollisionData = 0x3F;  //0011 1111 left to right bumper
          CollisionFlag = 0;

           while(1){
              CollisionFlag = 0;
              WaitForInterrupt();
              if(CollisionFlag==1)
              {
                  uint8_t tmp = CollisionData & 0x3f;
                  for(int i = 0; i<6; i++){
                      int digit = tmp & 1;
                      tmp=tmp>>1;
                      if(digit == 1){
                          UART0_OutString("1");
                      }else{
                          UART0_OutString("0");
                      }
                  }
                  UART0_OutString("\r\n");
                  UART0_OutUDec5(ConvertCollisionData(CollisionData));
                  break;
              }

          }


            menu = 1;
            cmd = 0xDEAD;
            break;

//[4]Reflectance
      case 4:
          EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);
          EUSCIA0_OutString("Reflectance testing......"); EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);
          EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);
          uint8_t refData = 0;
          int32_t position;
          while(LaunchPad_Input()==0){
              refData = Reflectance_Read(1000);
              position = Reflectance_Position(refData);
              for(int i = 0; i<8;i++){
                  EUSCIA0_OutUDec(refData%2);
                  refData=refData/2;    //right to left reflectance
              }
              EUSCIA0_OutString("  ");
              EUSCIA0_OutUDec(position);
              EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);
              Clock_Delay1ms(100);
          }

          menu = 1;
                      cmd = 0xDEAD;
                      break;

//[5]Tachometer----------------------------------------------------------------------------------------------
//1 ROTATION OF WHEEL HAS 1440 ticks (EDGES) FOR 2 Channels A and B
//HERE WE USE CHANNEL A ONLY, WITH RISING EDGES => NEED 1440/360 PULSES/TICKS FOR 1 FULL ROTATION OF MOTOR
//PERIOD BETWEEN 2 TICKS CALCULATED BY TACHO METER (in 83.3ns unit as clock of microprocessor like that)
// RPM = 1 minute (in microsecs) / Period between ticks (in microsecs)
//(60 * 1000 * 1000 )/(360*83.3*Period0/1000))
      case 5:
                EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);
                EUSCIA0_OutString("Tachometer testing......"); EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);
                EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);
                uint32_t main_count=0;
                TimerA3Capture_Init(&PeriodMeasure0,&PeriodMeasure2);

                    Motor_Forward(3000,3000);
                while(LaunchPad_Input()==0){
                      WaitForInterrupt();
                      main_count++;
                      if(main_count%1000){
                          UART0_OutString("Period0 = ");UART0_OutUDec5(Period0);UART0_OutString(" Period2 = ");UART0_OutUDec5(Period2);UART0_OutString(" \r\n");
                      }
                    }
                    Motor_Stop();
                                    menu = 1;
                                    cmd = 0xDEAD;
                                    break;


//[6]
//EXTRA IMPLEMENTATION: ANY OBSTACLE DETECT LESS THAN 10cm AWAY FROM THE IR SENSORS => EVASIVE ACTION
//EXIT BY BUMPER CONTACT
      case 6:
              CollisionFlag = 0;
            EUSCIA0_OutString("IR Sensor Dead Recognition"); EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);
            TimerA1_Init(&SensorRead_ISR,250);
            EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);
            while (1){
                Motor_Forward(3000,3000);
                if(CenterConvert(nc)<=10 && LeftConvert(nl)<=10){
                                    Motor_Stop();
                                    Motor_Backward(3000,3000);
                                    Clock_Delay1ms(500);
                                    Motor_Right(3000,3000);
                                    Clock_Delay1ms(1400);
                                }
                                else if(CenterConvert(nc)<=10 && RightConvert(nl)<=10){
                                   Motor_Stop();
                                   Motor_Backward(3000,3000);
                                   Clock_Delay1ms(500);
                                   Motor_Left(3000,3000);
                                   Clock_Delay1ms(1400);
                               }

                                else if(LeftConvert(nl)<=10){
                                    Motor_Stop();
                                    Motor_Right(3000,3000);
                                    Clock_Delay1ms(740);
                                }

                                else if(RightConvert(nr)<=10){
                                    Motor_Stop();
                                    Motor_Left(3000,3000);
                                    Clock_Delay1ms(740);
                                }
                                //Motor_Stop(); //Stop for more accurate readings of IR
                                UART0_OutUDec5(LeftConvert(nl));UART0_OutString(" cm,");
                                UART0_OutUDec5(CenterConvert(nc));UART0_OutString(" cm,");
                                UART0_OutUDec5(RightConvert(nr));UART0_OutString(" cm\r\n");
                                if (CollisionFlag)
                                    break;
                            }
                            Motor_Stop();
                            menu=1;
                            cmd=0xDEAD;
                            break;

                //[7]
                //EXTRA IMPLEMENTATION: WHEN BUMPER CONTACTED, MOVE BACKWARD AND ROTATE, THEN REPEAT ACTION (FORWARD) -> EVASIVE
                case 7:
                           EUSCIA0_OutString("Bumper Dead Recognition"); EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);
                              TimerA1_Init(&MotorMovt,50000);
                              //EXIT BY ANY BUTTONS ON LAUNCH PAD
                              while(LaunchPad_Input()==0);  // wait for touch
                              while(LaunchPad_Input());     // wait for release
                              Motor_Stop();
                              TimerA1_Stop(); //Halt
                              menu=1;
                              cmd=0xDEAD;
                              break;



                    menu = 1;
                  cmd = 0xDEAD;
                break;

                //-----------------------------------------------------------------------------------------------
                case 8:
                    uint8_t sensor_choice = 0x18; //middle
                    uint8_t Data;
                    while (1){
                    Data = Reflectance_Read(1000);
                    if ((Data & sensor_choice) == sensor_choice){
                            P2->OUT = 0x04;
                            Clock_Delay1ms(250);
                            P2->OUT = 0x00;
                            Clock_Delay1ms(250);
                        }
                        else{
                            P2->OUT = 0x00;}
                      }


                                    menu = 1;
                                  cmd = 0xDEAD;
                                break;

                case 9:
                    Clock_Init48MHz();  //SMCLK=12Mhz
                    uint32_t dist;

                  while (1){
                      UART0_OutUDec5(LeftConvert(nl));UART0_OutString(" mm,");
                                UART0_OutUDec5(CenterConvert(nc));UART0_OutString(" mm,");
                                UART0_OutUDec5(RightConvert(nr));UART0_OutString(" mm\r\n");
                  dist = CenterConvert(nc);
                      if (dist < 700){
                          P2->OUT = 0x01; //red
                      }
                      else if (dist < 900){
                          P2->OUT = 0x04; //green
                      }
                      else{
                          P2->OUT = 0x02; //blue
                      }
                  }


                  menu = 1;
                cmd = 0xDEAD;
                break;

                case 10:
                    uint8_t status;
                    while(1){
                        status = Port1_Input();
                        switch(status){
                        case 0x10:
                            Motor_Forward(1000,1000);
                            Clock_Delay1ms(1000);
                            Motor_Forward(2000,2000);
                            Clock_Delay1ms(1000);
                            Motor_Forward(3000,3000);
                            Clock_Delay1ms(1000);
                            Motor_Stop();
                            break;
                        case 0x02:
                            Motor_Forward(1000,1000);
                            Clock_Delay1ms(500);
                            Motor_Forward(2000,2000);
                            Clock_Delay1ms(500);
                            Motor_Forward(3000,3000);
                            Clock_Delay1ms(1000);
                            Motor_Stop();
                            break;
                        case 0x00:
                            Motor_Forward(1000,1000);
                            Clock_Delay1ms(500);
                            Motor_Forward(2000,2000);
                            Clock_Delay1ms(500);
                            Motor_Forward(3000,3000);
                            Clock_Delay1ms(1000);
                            Motor_Stop();
                            break;
                        default:
                            break;
                        }

                    }
                    menu = 1;
                   cmd = 0xDEAD;
                   break;

                case 11:  //hit front 2 bumpers, stop reverse
                    CollisionData = 0x3F;  //0011 1111 left to right bumper
                      CollisionFlag = 0;
                      uint8_t tmp;
                       while(1){
                          Motor_Forward(1000,1000);
                          CollisionFlag = 0;
                          WaitForInterrupt();
                          if(CollisionFlag==1)
                          {
                              tmp = CollisionData & 0x3f;
                              if (tmp == 0b00110011 || tmp == 0b00111011 || tmp == 0b00110111){ //00110011
                                  Port2_Output(RED);
                                  Motor_Stop();
                                  Clock_Delay1ms(750);
                                  Motor_Backward(1500,1500);
                                  Clock_Delay1ms(1000);
                                  Motor_Stop();
                                  Clock_Delay1ms(750);
                                  Motor_Right(1500,1500);
                                  Clock_Delay1ms(1300);
                                  Motor_Stop();
                                  Clock_Delay1ms(750);
                                  Motor_Forward(1000,1000);
                                  Clock_Delay1ms(3000);
                                  Motor_Stop();
                                  break;
                              }
                          }
                       }
                              menu = 1;
                           cmd = 0xDEAD;
                           break;

                case 12:   //parking at black line
                    uint8_t reflectance_data;
                    Port2_Output(0x00);
//                    Clock_Delay1ms(2000);
                    while (1){
                        Motor_Forward(1000,1000);
                        reflectance_data = Reflectance_Read(1000);
                        if (reflectance_data == 0xFF){
                            Port2_Output(BLUE);
                            Clock_Delay1ms(1300);
                            Motor_Stop();
                            break;
                        }
                    }
                    menu = 1;
                  cmd = 0xDEAD;
                  break;

                case 13:
                    int32_t Position;
                    uint8_t is_negative = 0;
                    uint8_t black_line_count = 0;
                    while (1){
                        if (Port1_Input() == 0x10 || Port1_Input() == 0x02){
                            while (1){
        //                        Motor_Forward(1000,1000);
                                reflectance_data = Reflectance_Read(1000);
                                 Position = Reflectance_Position(reflectance_data);

                                 //check if negative
                                 if (Position > 0xF0000000){
                                     is_negative = 1;
                                     Position = ~(Position) + 1;
                                     UART0_OutString('-');
                                 }
                                 else{
                                     is_negative = 0;
                                 }
                                 EUSCIA0_OutUDec(Position);
                                 UART0_OutString("\r\n");

                                 //check on black line
                                   if (reflectance_data == 0xFF){ // detect black line
                                       Port2_Output(0x05);
                                       if (black_line_count == 1){
                                           Motor_Stop();
                                           break;
                                       }
                                       else{
                                           black_line_count = 1;
                                           Motor_Forward(1000,1000);
                                           Clock_Delay1ms(500);
                                       }
                                   }
                                 //check on course

                                   //check straight
                                   else if (Position == 0){
                                       Port2_Output(GREEN);
                                       Motor_Forward(1000,1000);
                                       Clock_Delay1ms(100);
                                   }

                                 //   right turn
                                   else if (Position > 0 && !(is_negative)){
                                     Port2_Output(BLUE);
                                     Motor_Right(1000,1500);
                                     Clock_Delay1ms(100);

                                 }

                                 //   left turn
                                 else if (Position > 0 && is_negative){
                                     Port2_Output(RED);
                                     Motor_Left(1500,1000);
                                     Clock_Delay1ms(100);

                                 }


                             }
                        }
                    }

                case 14:
                    count = 0;
                    flag = 0;
                    int unlocked = 0;
                    while (unlocked == 0){
                        WaitForInterrupt();
                        if (flag == 0){
                            Port2_Output(RED);
                            Clock_Delay1ms(250);
                            Port2_Output(0x00);
                            Clock_Delay1ms(250);
                        }
                        else{
                            unlocked = 1;
                            Port2_Output(GREEN);
                        }
                    }

                    while (1){
                        if (Port1_Input() == 0x10 || Port1_Input() == 0x02){
                            while (1){
                                Port2_Output(BLUE);
//                                break;
                                Motor_Forward(1000,1000);
                                if (CenterConvert(nc) < 1500){
                                    Motor_Stop();
                                    Clock_Delay1ms(500);
                                    //turn left
                                    Motor_Left(1500,1500);
                                    Clock_Delay1ms(650);

                                    //go straight
                                    Motor_Forward(1000,1000);
                                    Clock_Delay1ms(1000);

                                    //turn right
                                    Motor_Right(1500,1500);
                                    Clock_Delay1ms(650);

                                    //go straight
                                    Motor_Forward(1000,1000);
                                    Clock_Delay1ms(2000);

                                    //turn right
                                    Motor_Right(1500,1500);
                                    Clock_Delay1ms(650);

                                    //go straight
                                    Motor_Forward(1000,1000);
                                    Clock_Delay1ms(1000);

                                    //turn left
                                    Motor_Left(1500,1500);
                                    Clock_Delay1ms(650);

                                    //go straight
                                    Motor_Forward(1000,1000);
                                    Clock_Delay1ms(1000);
                                    Motor_Stop();
                                    break;



                                }
                            }
                        }
                    }

                default:
                              menu=1;
                              break;
                      }

                      if(!menu)Clock_Delay1ms(3000);
                      else{
                          menu=0;
                      }

                      // ....
                      // ....
                  }
                }

                #if 0
                //Sample program for using the UART related functions.
                int Program5_4(void){
                //int main(void){
                    // demonstrates features of the EUSCIA0 driver
                  char ch;
                  char string[20];
                  uint32_t n;
                  DisableInterrupts();
                  Clock_Init48MHz();  // makes SMCLK=12 MHz
                  EUSCIA0_Init();     // initialize UART
                  EnableInterrupts();
                  EUSCIA0_OutString("\nLab 5 Test program for EUSCIA0 driver\n\rEUSCIA0_OutChar examples\n");
                  for(ch='A'; ch<='Z'; ch=ch+1){// print the uppercase alphabet
                     EUSCIA0_OutChar(ch);
                  }
                  EUSCIA0_OutChar(LF);
                  for(ch='a'; ch<='z'; ch=ch+1){// print the lowercase alphabet
                    EUSCIA0_OutChar(ch);
                  }
                  while(1){
                    EUSCIA0_OutString("\n\rInString: ");
                    EUSCIA0_InString(string,19); // user enters a string
                    EUSCIA0_OutString(" OutString="); EUSCIA0_OutString(string); EUSCIA0_OutChar(LF);

                    EUSCIA0_OutString("InUDec: ");   n=EUSCIA0_InUDec();
                    EUSCIA0_OutString(" OutUDec=");  EUSCIA0_OutUDec(n); EUSCIA0_OutChar(LF);
                    EUSCIA0_OutString(" OutUFix1="); EUSCIA0_OutUFix1(n); EUSCIA0_OutChar(LF);
                    EUSCIA0_OutString(" OutUFix2="); EUSCIA0_OutUFix2(n); EUSCIA0_OutChar(LF);

                    EUSCIA0_OutString("InUHex: ");   n=EUSCIA0_InUHex();
                    EUSCIA0_OutString(" OutUHex=");  EUSCIA0_OutUHex(n); EUSCIA0_OutChar(LF);
                  }
                }
                #endif
