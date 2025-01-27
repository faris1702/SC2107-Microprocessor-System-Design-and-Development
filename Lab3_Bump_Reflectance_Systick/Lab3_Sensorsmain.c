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

// reflectance LED illuminate connected to P5.3
// reflectance sensor 1 connected to P7.0 (robot's right, robot off road to left)
// reflectance sensor 2 connected to P7.1
// reflectance sensor 3 connected to P7.2
// reflectance sensor 4 connected to P7.3 center
// reflectance sensor 5 connected to P7.4 center
// reflectance sensor 6 connected to P7.5
// reflectance sensor 7 connected to P7.6
// reflectance sensor 8 connected to P7.7 (robot's left, robot off road to right)

#include "msp.h"
#include "..\inc\bump.h"
#include "..\inc\Reflectance.h"
#include "..\inc\Clock.h"
#include "..\inc\SysTickInts.h"
#include "..\inc\CortexM.h"
#include "..\inc\LaunchPad.h"
#include "..\inc\TExaS.h"
#include "..\inc\Motor.h"

volatile uint8_t reflectance_data, bump_data;
volatile uint32_t MainCount=0;

void SysTick_Handler(void){ // every 1ms
    volatile static uint8_t count=0;
    if(count==0){
        Reflectance_Start();
    }
    else if (count==1) {
        reflectance_data =  Reflectance_End();
        bump_data = Bump_Read();
    }
    count++;
    if(count==10)count=0;
}

//
//uint8_t numOfInterrupt = 0;
//uint8_t read_data;
//uint32_t password = 0;
//uint32_t correct_password = 6142;
//
//void PORT4_IRQHandler(void){
//    read_data = Bump_Read();
//    read_data = ~read_data;
//    numOfInterrupt++;
////    b = b^0x111111;
//    if ((read_data & 0x01) == 0x01){ //red
//        P2->OUT = 0x01;
//        password += 1;
//        password *= 10;
//    }
//    else if ((read_data & 0x02) == 0x02){ //green
//            P2->OUT = 0x02;
//            password += 2;
//            password *= 10;
//    }
//    else if ((read_data & 0x04) == 0x04){ //yellow
//            P2->OUT = 0x03;
//            password += 3;
//            password *= 10;
//    }
//    else if ((read_data & 0x08) == 0x08){ //blue
//            P2->OUT = 0x04;
//            password += 4;
//            password *= 10;
//    }
//    else if ((read_data & 0x10) == 0x10){ //pink
//            P2->OUT = 0x05;
//            password += 5;
//            password *= 10;
//    }
//    else if ((read_data & 0x20) == 0x20){ //sky blue
//        password += 6;
//        password *= 10;
//            P2->OUT = 0x06;
//    }
//
//    Clock_Delay1ms(250);  //for blinking LED
////    Motor_Stop();
////    Clock_Delay1ms(1000);
////    Motor_Backward(1000,1000);
////    Clock_Delay1ms(500);
////    Motor_Right(1800,1800);
////    Clock_Delay1ms(750);
//
////    b = 0;
//    P2->OUT = 0x00;
//    P4->IFG &= ~0xED;    //clear flag
//}
//


int main(void){

volatile uint8_t data;
//volatile uint8_t x = 1000;
    uint8_t run = 0;
    Clock_Init48MHz();
    LaunchPad_Init();
    Bump_Init();
    Reflectance_Init();
    TExaS_Init(LOGICANALYZER_P7);
//    Motor_Init();
//    SysTick_Init(48000,1);  // set up SysTick for 1000 Hz interrupts
    //test code
      P2->SEL0 &= ~0x07;
      P2->SEL1 &= ~0x07;                        // configure P2.2-P2.0 as GPIO
      P2->DS &= ~0x07;                          // make P2.2-P2.0 high drive strength
      P2->DIR |= 0x07;                         // make P2.2-P2.0 out
      P2->OUT &= ~0x07;
      P1->SEL0 &= ~0x01;
      P1->SEL1 &= ~0x01;                        // configure P2.2-P2.0 as GPIO
      P1->DS |= 0x01;                          // make P2.2-P2.0 high drive strength
      P1->DIR |= 0x01;                         // make P2.2-P2.0 out
      P1->OUT &= ~0x01;
      //init button P1.1, 1.4
      P1->SEL0 &= ~0x12;
      P1->SEL1 &= ~0x12;
      P1->DIR &= ~0x12;
      Motor_Init();
//      UART0_Init();
      //end test code
    EnableInterrupts();


    while(1){
        if (numOfInterrupt == 4){
            if (password/10 == correct_password){
                run = 1;
            }
        }

      if (run == 1){
          P1->OUT = 0x00;
//          Motor_Forward(1000,1000);
          Clock_Delay1ms(200);
//          Motor_Stop();
          break;
      }

      WaitForInterrupt();
//      if(MainCount%1000 == 0)P2->OUT ^= 0x01; // foreground thread
//      MainCount++;

    }
}



