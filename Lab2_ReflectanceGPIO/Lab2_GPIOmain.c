// Lab06_GPIOmain.c
// Runs on MSP432
// Solution to GPIO lab
// Daniel and Jonathan Valvano
// May 21, 2017
// Provide test main program for QTR-8RC reflectance sensor array
// Pololu part number 961.

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

// reflectance LED illuminate connected to P5.3
// reflectance sensor 1 connected to P7.0 (robot's right, robot off road to left)
// reflectance sensor 2 connected to P7.1
// reflectance sensor 3 connected to P7.2
// reflectance sensor 4 connected to P7.3 center
// reflectance sensor 5 connected to P7.4 center
// reflectance sensor 6 connected to P7.5
// reflectance sensor 7 connected to P7.6
// reflectance sensor 8 connected to P7.7 (robot's left, robot off road to right)

#include <stdint.h>
#include "msp.h"
#include "..\inc\Reflectance.h"
#include "..\inc\Clock.h"
#include "..\inc\TExaS.h"
#include "..\inc\Motor.h"

//void LED_Init(void);
//void ON_LED(void);
//void ON_LED_Color(uint8_t color);
//void Blink_LED(void);

// built-in LED1 connected to P1.0
// negative logic built-in Button 1 connected to P1.1
// negative logic built-in Button 2 connected to P1.4
// built-in red LED connected to P2.0
// built-in green LED connected to P2.1
// built-in blue LED connected to P2.2
// Color    LED(s) Port2
// dark     ---    0
// red      R--    0x01
// blue     --B    0x04
// green    -G-    0x02
// yellow   RG-    0x03
// sky blue -GB    0x06
// white    RGB    0x07
// pink     R-B    0x05


//Initialise GPIO Port2 registers
//void LED_Init(void){
//  P2->SEL0 = 0x00;
//  P2->SEL1 = 0x00;                        // configure P2.2-P2.0 as GPIO
//  P2->DS = 0x07;                          // make P2.2-P2.0 high drive strength
//  P2->DIR = 0x07;                         // make P2.2-P2.0 out
//  P2->OUT = 0x00;                         // all LEDs off
//}
//
//void ON_LED(void){
//    uint8_t color = 0x01;  //follow from list above
//    P2->OUT = color;
//}
//
//void ON_LED_Color(uint8_t color){
//    P2->OUT = color;
//}
//
//void OFF_LED(void){
//    P2->OUT = 0x00;
//}
//
//void Blink_LED(void){
//    ON_LED();
//    Clock_Delay1ms(10);
//    OFF_LED();
//}

uint8_t Data; // QTR-8RC
int32_t Position; // 332 is right, and -332 is left of center
uint8_t sensor_choice, result, refData;
uint8_t numOfInterrupt;
int main(void){
    uint8_t on_black_line = 0;
    uint8_t black_line = 0;
  Clock_Init48MHz();
  Reflectance_Init();
  TExaS_Init(LOGICANALYZER_P7);
    P2->SEL0 = 0x00;
    P2->SEL1 = 0x00;                        // configure P2.2-P2.0 as GPIO
    P2->DS = 0x07;                          // make P2.2-P2.0 high drive strength
    P2->DIR = 0x07;                         // make P2.2-P2.0 out
    P2->OUT = 0x00;
    P1->SEL0 = 0x00;
    P1->SEL1 = 0x00;                        // configure P2.2-P2.0 as GPIO
    P1->DS = 0x01;                          // make P2.2-P2.0 high drive strength
    P1->DIR = 0x01;                         // make P2.2-P2.0 out
    P1->OUT = 0x00;
    sensor_choice = 0x18;

    //test code
      Motor_Init();
      //test code

//  while(1){
////     P1->OUT = 0x01;
////     Clock_Delay1ms(250);
////     P1->OUT = 0x00;
////     Clock_Delay1ms(250);
//     Data = Reflectance_Read(1000);
//     refData = Data;
////    Data = Reflectance_Center(1000);
////    Position = Reflectance_Position(Data);
////     for(int i = 0; i<8;i++){
////         UART0_OutUDec5(refData%2);
//////        UART0_OutString("-");
////         refData = refData/2;    //right to left reflectance
////     }
////    UART0_OutString("\r\n");
//    if ((Data & sensor_choice) == sensor_choice){
//        P2->OUT = 0x04;
//        Clock_Delay1ms(250);
//        P2->OUT = 0x00;
//        Clock_Delay1ms(250);
//    }
//    else{
//        P2->OUT = 0x00;}
//  }



      //----line following, stop on 2nd black line--------
      while (1){
          Data = Reflectance_Read(1000);
          Position = Reflectance_Position(Data);
          Motor_Forward(1000,1000);
          Clock_Delay1ms(500);

          //check on black line
            if (Data <= 0x3F){ //6 sensor detect black
                if (black_line == 1){ //now at 2nd black line
                    Motor_Stop();
                    break;
                }
                else{  //now at first line
                    on_black_line = 1;
                }
            }

            else{
                if (on_black_line == 1){
                    black_line = 1;
                    on_black_line = 0;
                }
            }

          //check on course
          //332, 237, 142, 47, -47, -142, -237, -332
          //   right turn
          if (Position > 0){
              if (Position < 150){  //not so off
                  Motor_Right(1000,1000);
                  Clock_Delay(100);
              }
              else{ //quite off
                  Motor_Right(1500,1500);
                  Clock_Delay(100);
              }
          }

          //   left turn
          else if (Position < 0){
              if (Position > -150){ //not so off
                  Motor_Left(1000,1000);
                  Clock_Delay1ms(100);
              }
              else{  //quite off
                  Motor_Left(1500,1500);
                  Clock_Delay1ms(100);
              }
          }

      }
}


