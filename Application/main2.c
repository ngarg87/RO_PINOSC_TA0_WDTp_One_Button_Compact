/* --COPYRIGHT--,BSD
 * Copyright (c) 2012, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * --/COPYRIGHT--*/
//******************************************************************************
// RO_PINOSC_TA0_One_Button_Compact example
// Touch the middle element to turn on/off the center button LED
// Uses the low-level API function calls for reduced memory footprint.
// Note: Baseline tracking is implemented in the Application Layer
// RO method capactiance measurement using PinOsc IO, TimerA0, and WDT+
//
//          Schematic Description:
//
//                         MSP430G2452
//                      +---------------+
//                      |
//             C--------|P2.4
//           C----------|P2.1
//               C------|P2.3
//             C--------|P2.2
//                      |
//           C----------|P2.5
//                      |
//           C----------|P2.0
//
//        The WDT+ interval represents the measurement window.  The number of
//        counts within the TA0R that have accumulated during the measurement
//        window represents the capacitance of the element. This is lowest
//        power option with either LPM3 (ACLK WDTp source) or LPM0 (SMCLK WDTp
//        source).
//
//******************************************************************************
#include  "CTS_Layer.h"

typedef enum
 {
  INIT_PRESS,
//  TYPE_PRESS,
  CHECK_PRESS,
  ACTION_PRESS
 } Press_state;

// Define User Configuration values //
//----------------------------------//
#define DELAY 5000 		// Timer delay timeout count - 5000*0.1msec = 500 msec

// Sensor settings
#define KEY_middle     0x1B0              // Defines the min count for a "key press"
                                     // Set to ~ half the max delta expected
#define KEY_up			0x180
#define KEY_down1		0x180
#define KEY_down2		0x230
#define KEY_left		0x590
#define KEY_right		0x590
#define KEY_diag_left	0x300
#define KEY_diag_right  0xA0

#define num_keys		7
#define delta 			50



// Global variables for sensing
unsigned int base_cnt[7];
unsigned int meas_cnt[7];
unsigned int delta_cnt[7];
unsigned int touch_lvl[7] = {KEY_middle, KEY_down1, KEY_up, KEY_left, KEY_right, KEY_diag_left, KEY_diag_right};
unsigned int touch[7];
unsigned int touch_occurred;
unsigned int delta_array[50];
unsigned int delta_array2[50];
unsigned int j=0;
unsigned int home_press=0;
unsigned int home_press_valid=0;
unsigned int home_press_invalid=0;
unsigned int other_press=0;
unsigned int press=0;
unsigned int press2 = 0;
unsigned int counter=0;
Press_state	Press_machine=INIT_PRESS;

char key_pressed, key_loc;
int cycles;

// Sleep Function
// Configures Timer A to run off ACLK, count in UP mode, places the CPU in LPM3
// and enables the interrupt vector to jump to ISR upon timeout
void sleep(unsigned int time)
{
    TA0CCR0 = time;
    TA0CTL = TASSEL_1+MC_1+TACLR;
    TA0CCTL0 &= ~CCIFG;
    TA0CCTL0 |= CCIE;
    __bis_SR_register(LPM3_bits+GIE);
    __no_operation();
}

// Main Function
void main(void)
{

  unsigned int test_down=0;
  unsigned int enter = 0;

  WDTCTL = WDTPW + WDTHOLD;             // Stop watchdog timer
  BCSCTL1 = CALBC1_1MHZ;                // Set DCO to 1, 8, 12 or 16MHz
  DCOCTL = CALDCO_1MHZ;
  BCSCTL1 |= DIVA_0;                    // ACLK/(0:1,1:2,2:4,3:8)
  BCSCTL3 |= LFXT1S_2;                  // LFXT1 = VLO

  P1OUT = 0x00;							// Clear Port 1 bits
  P1DIR |= BIT0;						// Set P1.0 as output pin

  P2SEL &= ~(BIT6 + BIT7);	// Configure XIN (P2.6) and XOUT (P2.7) to GPIO
  P2OUT = 0x00;				// Drive all Port 2 pins low
  P2DIR = 0xFF;				// Configure all Port 2 pins outputs

  // Initialize Baseline measurement
  TI_CAPT_Raw(&middle_button,&base_cnt[0]);
  TI_CAPT_Raw(&volume_down1,&base_cnt[1]);
  TI_CAPT_Raw(&volume_up1,&base_cnt[2]);
  TI_CAPT_Raw(&left1,&base_cnt[3]);
  TI_CAPT_Raw(&right1,&base_cnt[4]);
  TI_CAPT_Raw(&diag_left1,&base_cnt[5]);
  TI_CAPT_Raw(&diag_right1,&base_cnt[6]);

  // Main loop starts here
  while (1)
  {
	  int i;

  	  // Take raw delta measurement
      TI_CAPT_Raw(&middle_button,&meas_cnt[0]);
      TI_CAPT_Raw(&volume_down1,&meas_cnt[1]);
      TI_CAPT_Raw(&volume_up1,&meas_cnt[2]);
      TI_CAPT_Raw(&left1,&meas_cnt[3]);
      TI_CAPT_Raw(&right1,&meas_cnt[4]);
      TI_CAPT_Raw(&diag_left1,&meas_cnt[5]);
      TI_CAPT_Raw(&diag_right1,&meas_cnt[6]);

      touch_occurred = 0;

      for (i =0; i<num_keys; i++) {
    	  if(base_cnt[i] < meas_cnt[i])
    		  // Handle baseline measurment for a base C decrease
    	  {                                 // beyond baseline, i.e. cap decreased
    		  base_cnt[i] = (base_cnt[i]+meas_cnt[i]) >> 1; // Re-average baseline up quickly
    		  delta_cnt[i] = 0;             // Zero out delta for position determination
    		  touch[i]=0;
    	  }
    	  else
    	  {
    		  delta_cnt[i] = base_cnt[i] - meas_cnt[i];  // Calculate delta: c_change
    		  if (delta_cnt[i] > touch_lvl[i]) {
    			  touch[i] = 1;
    			  touch_occurred = 1;
    		  }
    	  }
      }

      delta_array[j] = delta_cnt[0];					//test logging code
      delta_array2[j++] = delta_cnt[1];					//test logging code
      if (j==50) j=0;

//      if ((test_down == 0) && (delta_cnt[1] > KEY_down1)) {
//    	  sleep(3000);
//    	  test_down = 1;
//      }
//
//      else if ((test_down == 1) && (delta_cnt[1] > KEY_down2)) {
//    	  test_down = 0;
//    	  P1OUT &= ~BIT0;
//    	  sleep(3000);
//      }

      enter = 1;
      counter++;

      switch(Press_machine)
         {
      	  case INIT_PRESS:
      		  if (touch_occurred) {
//          		  Press_machine=TYPE_PRESS;
          		  counter = 0;
//        	  }
//      		  break;

//      	  case TYPE_PRESS:
				  if (touch[1] || touch[2] || touch[3] || touch[4] || touch[5] || touch[6]) {
					  other_press = 1;
					  home_press_valid = 0;
				  }
				  else other_press = 0;

				  if (touch[0] && !other_press) {
					  home_press = 1;
	//            	  home_press_valid = 1;
				  }
				  else home_press = 0;

				  Press_machine = CHECK_PRESS;
      		  }

              break;

      	  case CHECK_PRESS:
      		  if (other_press) {			//other_press flag has been set so it's all invalid until release
      			  if ((delta_cnt[0] > 100) || (delta_cnt[1] > 100) || (delta_cnt[2] > 100) || (delta_cnt[3] > 100) || (delta_cnt[4] > 100) || (delta_cnt[5] > 100) || (delta_cnt[6] > 100)) {
      			  __no_operation();		//something still being touched, do nothing else
      			  }

      			  else Press_machine = ACTION_PRESS;
      		  }

      		  else if (home_press) {
      			  if (((delta_cnt[1] > KEY_down2) || touch[2] || touch[3] || touch[4] || touch[5] || touch[6]) || (delta_cnt[1] > delta_cnt[0])) {
      				  home_press_invalid = 1;				//finger touched rejection area of sensor
      			  }

      			  else if (delta_cnt[0]>100) {
      				  home_press_valid = 1;
      			  }

      			  else Press_machine = ACTION_PRESS;
      		  }
      		  break;

      	  case ACTION_PRESS:
      	      if (home_press_valid && !home_press_invalid && (counter < 20))
      	      {
      				// turn on LED
      				P1OUT |= BIT0;
      				delta_array[j-1]=696;
      				sleep(3000);
      	      }

      	      home_press_valid = 0;
      	      other_press=0;
			  home_press=0;
			  home_press_invalid = 0;

      	      Press_machine=INIT_PRESS;
      		  break;

         }

    	  //enter = 0;
//    	  if ((delta_cnt[2] > KEY_up) || (delta_cnt[3] > KEY_left) || (delta_cnt[4] > KEY_right))
//    	  {
//    		  sleep(3000);
//    		  P1OUT &= ~BIT0;
//    	  }
//    	  else if ((delta_cnt[0] > (delta_cnt[1]-delta)) && (delta_cnt[0] > KEY_middle))
//    	  {
//    		  enter = 1;
//    	  }
//    	  else
//    	  {
//    		  sleep(3000);
//    		  P1OUT &= ~BIT0;
//    	  }



        // turn off LED
        base_cnt[0] = base_cnt[0] - 1;  // Adjust baseline down, should be slow to
        base_cnt[1] = base_cnt[1] - 1;  // Adjust baseline down, should be slow to
        base_cnt[2] = base_cnt[2] - 1;  // Adjust baseline down, should be slow to
        base_cnt[3] = base_cnt[3] - 1;  // Adjust baseline down, should be slow to
        base_cnt[4] = base_cnt[4] - 1;  // Adjust baseline down, should be slow to
        base_cnt[5] = base_cnt[5] - 1;  // Adjust baseline down, should be slow to
        base_cnt[6] = base_cnt[6] - 1;  // Adjust baseline down, should be slow to
        P1OUT &= ~BIT0;

  }
} // End Main

/******************************************************************************/
// Timer0_A0 Interrupt Service Routine: Disables the timer and exists LPM3
/******************************************************************************/
#pragma vector=TIMER0_A0_VECTOR
__interrupt void ISR_Timer0_A0(void)
{
  TA0CTL &= ~(MC_1);
  TA0CCTL0 &= ~(CCIE);
  __bic_SR_register_on_exit(LPM3_bits+GIE);
}


#pragma vector=PORT2_VECTOR,             \
  PORT1_VECTOR,                          \
  TIMER0_A1_VECTOR,                      \
  NMI_VECTOR,COMPARATORA_VECTOR,         \
  ADC10_VECTOR
__interrupt void ISR_trap(void)
{
  // the following will cause an access violation which results in a PUC reset
  WDTCTL = 0;
}
