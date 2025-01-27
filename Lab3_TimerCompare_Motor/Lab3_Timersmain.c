// Lab3_Timersmain.c

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
// Negative logic bump sensors
// P8.7 Bump5
// P8.6 Bump4
// P8.5 Bump3
// P8.4 Bump2
// P8.3 Bump1
// P8.0 Bump0

// Sever VCCMD=VREG jumper on Motor Driver and Power Distribution Board and connect VCCMD to 3.3V.
//   This makes P3.7 and P3.6 low power disables for motor drivers.  0 to sleep/stop.
// Sever nSLPL=nSLPR jumper.
//   This separates P3.7 and P3.6 allowing for independent control
// Left motor direction connected to P1.7 (J2.14)
// Left motor PWM connected to P2.7/TA0CCP4 (J4.40)
// Left motor enable connected to P3.7 (J4.31)
// Right motor direction connected to P1.6 (J2.15)
// Right motor PWM connected to P2.6/TA0CCP3 (J4.39)
// Right motor enable connected to P3.6 (J2.11)

#include "msp.h"
#include "..\inc\bump.h"
#include "..\inc\Clock.h"
#include "..\inc\SysTick.h"
#include "..\inc\CortexM.h"
#include "..\inc\LaunchPad.h"
#include "..\inc\Motor.h"
#include "..\inc\TimerA1.h"
#include "..\inc\TExaS.h"
#include "..\inc\Reflectance.h"


volatile uint8_t reflectance_data, bump_data;

// Driver test
void TimedPause(uint32_t time){
  Clock_Delay1ms(time);          // run for a while and stop
  Motor_Stop();
  while(LaunchPad_Input()==0);  // wait for touch
  while(LaunchPad_Input());     // wait for release
}

// Test of Periodic interrupt
#define REDLED (*((volatile uint8_t *)(0x42098060)))
#define BLUELED (*((volatile uint8_t *)(0x42098068)))
uint32_t Time;
void Task(void){
#if 0
  REDLED ^= 0x01;       // toggle P2.0
  REDLED ^= 0x01;       // toggle P2.0
  Time = Time + 1;
  REDLED ^= 0x01;       // toggle P2.0
#endif

#if 1
  volatile static uint8_t count=0;

  count++;
  if(count==10)count=0;

  if(Bump_Read()!=0x3F){
      P3->OUT &= ~0xC0;   // low current sleep mode
  }
  else{
      P3->OUT |= 0xC0;   // Wake up motor
  }

#endif
}

int main(void){
    // Uses Timer generated PWM to move the robot
    // Uses TimerA1 to periodically
    // check the bump switches, stopping the robot on a collision
    volatile uint8_t x = 1000;
    uint8_t reflectance_data;

    Clock_Init48MHz();
    LaunchPad_Init(); // built-in switches and LEDs
    //test code
//    UART0_Init();
    //test code
//    Bump_Init();      // bump switches
    P2->SEL0 = 0x00;
        P2->SEL1 = 0x00;                        // configure P2.2-P2.0 as GPIO
        P2->DS = 0x07;                          // make P2.2-P2.0 high drive strength
        P2->DIR = 0x07;                         // make P2.2-P2.0 out
        P2->OUT = 0x00;

    TExaS_Init(LOGICANALYZER_P2);
    TimerA1_Init(&Task,50000);  // 10 Hz
    //test code
    P1->SEL0 &= ~0x12;
    P1->SEL1 &= ~0x12;
    P1->DIR &= ~0x12;
//    Reflectance_Init();
    //end test code
//    EnableInterrupts();
    Motor_Init();     // your function
    Motor_Forward(2000,2000);
    Clock_Delay1ms(500);
    Motor_Stop();
    while (1){
        //blink led
        P2->OUT = 0x01;
        Clock_Delay1ms(250);
        P2->OUT = 0x00;
        Clock_Delay1ms(250);

        //press button to move
       if (((P1->IN & 0x12) == 0x10) || ((P1->IN & 0x12) == 0x02)){ //if button pressed
            Motor_Forward(2000,2000);
            Clock_Delay1ms(3000);
            Motor_Forward(3000,3000);
            Clock_Delay1ms(3000);
            Motor_Stop();
        }
    }

//    TimedPause(1000);
//    while(1){
//      Motor_Forward(1000,1000);  // your function
//      Clock_Delay1us(50);
//      WaitForInterrupt();
//      Clock_Delay1ms(300);
//      break;
//      Motor_Backward(3000,3000); // your function
//      TimedPause(1000);
//      Motor_Left(3000,3000);     // your function
//      TimedPause(1000);
//      Motor_Right(3000,3000);    // your function
//      TimedPause(1000);

      //test code (press switch to move)
//      Move_Forward(x, x);
//      Clock_Delay1ms(100);
//      x += 500;
      //end test code

      //test code 2 (parking)
//      reflectance_data = Reflectance_Read(1000);
//      if (reflectance_data == 0xFF){
//          Motor_Stop();
//          break;
//      }
      //end test code 2
//    }
}

