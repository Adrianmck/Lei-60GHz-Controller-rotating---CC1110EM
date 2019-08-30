/***********************************************************************************
  Filename:     main.c

  Description:  

  Comments:     

  Note:

***********************************************************************************/

/***********************************************************************************
* INCLUDES
*/

#include "Radio.h"
#include "hal_main.h"
#include "Power_Mode.h"
#include "wdt.h"
#include "rnd.h"
#include "clock.h"
#include "ioCC1110.h"
#include "UART.h"
#include "hal_bui_SmartRF04EB.h"


/*******************************/
// comment here if bulk building
/*********************************/

//// RFID Tag Number - 6 IEEE (?) Byte Address
#define MAX_NODES 2
#define TWO_ANTENNA 2
#define CTRL_PKT 0x01              // control packet is 1
#define CTRL_LENGTH 6              // control packet is 1
#define INSTALL_ID 0x01            // maximum of 4 bites
#define TIMER_MULTIPLE  1       
//#define TIMER_DIVIDE  1      //
#define CLOCK_FREQUENCY 26
//#define PACKET_LENGTH 10
//#define INSTALL_ID 0x02                     // Install ID bits 2-8 (2 to 256) or 1 to 128 << 1
//#define SLEEP_EVENT0 0x1600                  // Sleep timer Event0 high and low bits WOREVT0 WOREVT1
//#define SLEEP_EXP 0x01                      // Sleep timer resolution WOR_RES
//#define PA0 0xC2                         // out put powerr settings 0x50 0dBm, 0x84 5dBm, 0xC2 10dBm
#define UART_NUMBER 0
#define CRYSTALSPEED 26
/********************************************/
#define SIZE_OF_TX_BUFFER 30

/***********************************************************************************
* CONSTANTS
*/


/***********************************************************************************
* LOCAL VARIABLES
*/
uint8 j;                        // Loop variable
uint16 sequenceNumber = 0x01;   // Sequence number
uint8 txBuffer[SIZE_OF_TX_BUFFER]; // Local transmit
uint8 txNode = 0;
uint8 ctrlPkt = CTRL_PKT << 4;    //The packet type is the first 4 bits
uint16 outerLoop = 0;
uint16 innerLoop = 0;
static BOOL SendFlag = FALSE;            // Flag set whenever a packet is sent 
uint8 T2Sqn = 0;
BOOL CalFlag = FALSE;
uint8 T2count = 0;
uint8 steps = 0;                        // current postion of motor
uint8 step_inc = 0;                          // size of motor step
uint8 step_freq = 0;                    // now often to move motor.
uint8 step_limit = 0;                   // how far we can move the motor
uint8 step_reverse = 0;                 // wether to stop and reverese the motor direction
uint8 TX_ON;                            // transmit or edit menu
uint8 Menu_vars[4];                     //[ steps_inc, step_freq,step_limit,step_reverse]
uint8 menu_idx;


static DMA_DESC dmaConfig[2];                  // Struct for the DMA configuration
uint8 UartTxBuffer[SIZE_OF_TX_BUFFER]; // Local transmit
uint8 UART_DMA_clear = TRUE;
UART_PROT_CONFIG uartProtConfig;   // UART settings
/***********************************************************************************
* LOCAL FUNCTIONS
*/
uint8 tempResetWDT(void);
void clockSetMainSrc(uint8);
void SendPacket();
void SendSQN2Uart();
/***********************************************************************************
* @fn          main
*
* @brief       
*
* @param       void
*
* @return      0
*/
void main (void)
{
  
  // Initialise the allocated UART buffers:
  for (j = 0; j < SIZE_OF_TX_BUFFER; j++)
  {
    txBuffer[j] = 0xCC; //0;
  }
 
   if (CLOCK_FREQUENCY == 26) {
/* Sync word qualifier mode = 30/32 sync word bits detected */
/* Channel spacing = 199.951172 */
/* Data rate = 499.878 */
/* RX filter BW = 812.500000 */
/* PA ramping = false */
/* Preamble count = 8 */
/* Whitening = false */
/* Address config = No address check */
/* Carrier frequency = 868.899719 */
/* Device address = 0 */
/* TX power = 10 */
/* Manchester enable = false */
/* CRC enable = true */
/* Phase transition time = 0 */
/* Packet length mode = Variable packet length mode. Packet length configured by the first byte after sync word */
/* Packet length = 255 */
/* Modulation format = MSK */
/* Base frequency = 868.299866 */
/* Modulated = true */
/* Channel number = 3 */
/* RF settings SoC: CC1110 */
PKTCTRL0  = 0x05; // packet automation control 
CHANNR    = 0x03; // channel number 
FSCTRL1   = 0x0E; // frequency synthesizer control 
FREQ2     = 0x21; // frequency control word, high byte 
FREQ1     = 0x65; // frequency control word, middle byte 
FREQ0     = 0x6A; // frequency control word, low byte 
MDMCFG4   = 0x0E; // modem configuration 
MDMCFG3   = 0x3B; // modem configuration 
MDMCFG2   = 0x73; // modem configuration 
MDMCFG1   = 0x42; // modem configuration 
DEVIATN   = 0x00; // modem deviation setting 
    MCSM1     = 0x31; // main radio control state machine configuration 
MCSM0     = 0x08; // main radio control state machine configuration 
FOCCFG    = 0x1D; // frequency offset compensation configuration 
BSCFG     = 0x1C; // bit synchronization configuration 
AGCCTRL2  = 0xC7; // agc control 
AGCCTRL1  = 0x00; // agc control 
AGCCTRL0  = 0xB0; // agc control 
FREND1    = 0xB6; // front end rx configuration 
FSCAL3    = 0xEA; // frequency synthesizer calibration 
FSCAL2    = 0x2A; // frequency synthesizer calibration 
FSCAL1    = 0x00; // frequency synthesizer calibration 
FSCAL0    = 0x1F; // frequency synthesizer calibration 
TEST1     = 0x31; // various test settings 
TEST0     = 0x09; // various test settings 
PA_TABLE0 = 0xC2; // pa power setting 0 



    
  }
  else if (CLOCK_FREQUENCY == 24) {

////
///   
    
  }   


 // set up UART
    
  // Initialise the allocated UART buffers:
  for (j = 0; j < SIZE_OF_TX_BUFFER; j++)
  {
    UartTxBuffer[j] = 0xCC; //0;
  }
      
  // Desired settings for UART protocol
  uartProtConfig.uartNum = UART_NUMBER; // Use UART 0
  uartProtConfig.START = 0;   // Start level low => Idle = high
  uartProtConfig.STOP = 1;    // Stop level high
  uartProtConfig.SPB = 0;     // Number of stop bits = 1
  uartProtConfig.PARITY = 0;  // Parity disabled
  uartProtConfig.BIT9 = 0;    // Enable 8-bit transfer
  uartProtConfig.D9 = 1;      // Don't know / care???
  uartProtConfig.FLOW = 0;    // Hardware flow control disabled
  uartProtConfig.ORDER = 0;   // LSB first
  // Baudrate = 57.6 kbps (U0BAUD.BAUD_M = 34, U0GCR.BAUD_E = 11)
  if (CRYSTALSPEED == 26) 
  {
    // Baudrate = 230400 (U0BAUD.BAUD_M = 34, U0GCR.BAUD_E = 13)
    // Baudrate = 115200 (U0BAUD.BAUD_M = 34, U0GCR.BAUD_E = 12)
    uartProtConfig.UART_BAUD_M = 34;
    uartProtConfig.UART_BAUD_E = 12;
  }
  else if (CRYSTALSPEED == 24) 
  {
    // Baudrate = 230400 (U0BAUD.BAUD_M = 59, U0GCR.BAUD_E = 13)
//    uartProtConfig.UART_BAUD_M = 59;
//    uartProtConfig.UART_BAUD_E = 13;
    
        // Baudrate = 115200 (U0BAUD.BAUD_M = 59, U0GCR.BAUD_E = 12)
    uartProtConfig.UART_BAUD_M = 59;
    uartProtConfig.UART_BAUD_E = 12;
  }
  

  setupUARTInterface(&uartProtConfig);  








   // Determine proper RSSI offset for receiver (freq and rate dependent)
   // and configure the radio frequency to use
   //perRssiOffset = 77;
   //PKTLEN = PACKET_LENGTH;     // Packet length.	*** ADJUST THIS TO THE MAX PACKET LENGTH ALLOWED ***

      
  // Wake the dog!!!
  //while(!watchdogStart(WDCTL_INT1_MSEC_250));
  
    // Choose the crystal oscillator as the system clock
    halPowerClkMgmtSetMainClkSrc(CRYSTAL);      // Simons clock wait code
    // set the tickspeed
    TICKSPD_SET(CLKSPD_DIV_2);
    
    /***************************************************************************
     * Setup test stuff
     */
    // Initialize P1_1/3 for SRF04EB LED1/3
	//P1_0 LED
	// P1_2 High Antenna 1, Low Antenna 2
	// P0_3 Debug
	// P0_4 Debug
	// P0_5 Debug 
    P1SEL &= ~(BIT5 | BIT4 | BIT3 | BIT2 | BIT0);    
    P1DIR |= (BIT5 | BIT4 | BIT3 | BIT2 | BIT0);
	P1_0 = 1; P1_2 = 1; P1_3 = 1; P1_4 = 1; P1_5 = 1;
    // initrialise push button P0_2
        INIT_BUTTON();
    // initialise LCD
            halBuiInitLcd();
    // joystick 
    /***************************************************************************
    * Setup interrupt
    */
    

    // Clear Timer 2 interrupt flags
    // CPU interrupt flag (IRCON.T2IF) is cleared automatically by hardware

    // Set individual interrupt enable bit in the peripherals SFR
    T2CTL |= T2CTL_INT;             // Enable interrupt by setting [T2CTL.INT=1]

    // Enable Timer 2 interrupts by setting [IEN1.T1IE]
    T2IE = 1;

    // Enable global interrupt by setting the [IEN0.EA=1]
    EA = 1;

    /***************************************************************************
     * Setup Timer settings
     *
     * By setting [T2CTL.TIG = 1], the Timer will be counting down in
     * free-running mode and wrap around when 0x00 is reached. By setting
     * [T2CTL.TIG = 0], the Timer will be counting down starting from the current
     * T2CT value and stop when 0x00 is reached. Obviously, if this value is
     * 0x00 at the beginning, the timer will do nothing.
     */

    // Set timer 2 interval / timeslot and start Timer 2

    // Set Timer 2 prescalar multiplier value (0x00 is interpreted as 256)
    T2PR = TIMER_MULTIPLE;
    
//     T2PR = (TIMER_MULTIPLE  * (MAX_NODES +1))/ TIMER_DIVIDE;

    // Set Timer 2 count 
//    T2CT  = (255/ (MAX_NODES + 1) )+ 1;
  //  T2count = T2CT;
    // Set [T2CTL.TIP] to 64 and start Timer 2 in free running mode
    T2CTL = (T2CTL & ~T2CTL_TIP) | T2CTL_TIP_64 | T2CTL_TIG;
    //T2CTL = (T2CTL & ~T2CTL_TIP) | T2CTL_TIP_128 | T2CTL_TIG;
     

    /* This gives the counter speed for T2CT of (1 * 64) / 13 * 10^6 = 0.0049 ms
       which then gives a period of 0.049 * 10^-3 * 256 = 1.26ms (793 Hz),
       since counter runs up to 256 */
  CalFlag = TRUE;      
//  uint8 ftest1 = 0;
//  uint8 ftest2 = 0;
//  uint8 ftest3 = 0;
   P1_3 = 0; P1_4 = 0; P1_5 = 0; 
  while(1)
  {   
    
    if (TX_ON)
    {    
      if (SendFlag) 
      {
      //     while(!tempResetWDT());                 // Kick the dog!!!
          P1_5 = 1;
                  //P1_2 ^= 1;
           SendPacket();         
          P1_5 = 0;
           SendFlag = FALSE;

        // send sqn via UART
         if(UART_DMA_clear) {  
          UART_DMA_clear = FALSE;        
          SendSQN2Uart();      
         }         
  //        P1_0 = 0; 
      }
      if (CalFlag) 
      {
        //P1_5 = 1; 
        radioCalIDLE();          
        //P1_5 = 0; 
        CalFlag = FALSE;
      } 
      
      if(halBuiButtonPushed){
        halBuiLcdUpdate("TX OFF", " ...");
        TX_ON = 0;
      }
    }
    
    // enter menu
    else{
      
      jd_current = halBuiGetJoystickDirection(PARTNUM);
      halBuiLcdUpdateLine(LINE,
      halBuiLcdUpdateLine(LINE1,
      
      
       switch (jd_latest) {
            case UP:
                // Go to previous menu entry in list if possible
                if (selectedOption > 0) {
                    selectedOption--;
                    updLcd = TRUE;
                }
                break;

            case DOWN:
                // Go to next menu entry in list if possible
                if (selectedOption < numOptions - 1) {
                    selectedOption++;
                    updLcd = TRUE;
                }
                break;

            case LEFT:
            case RIGHT:
            case CENTERED:
            default:
                // No functionality for these directions
                break;
      
      if(halBuiButtonPushed){
        halBuiLcdUpdate("TX ON", " ...");
        TX_ON = 1;
      }
    }
  } /* End while loop */

} // End function

uint8 tempResetWDT(void){
  
    WDCTL = WDCTL | 0xA0; // Write 1010 to WDCTL.CLR[3:0]
    WDCTL = WDCTL ^ 0xF0; // Followed by 0101 to reset
    
    return(1);
}

void clockSetMainSrc(uint8 source)
{
  register uint8 osc32k_bm = CLKCON & CLKCON_OSC32K_BM;
  // Source can have the following values:
  // CLOCK_SRC_XOSC   0x00  /*  High Speed Crystal Oscillator (XOSC) */
  // CLOCK_SRC_HFRC   0x01  /*  High Speed RC Oscillator (HS RCOSC) */
  
  if (source == CLOCK_SRC_HFRC)
  {
    SLEEP &= ~SLEEP_OSC_PD_BM;          		// power up both oscillators
    while (!(SLEEP & SLEEP_HFRC_STB_BM));            // Wait until HS RCOSC is stable
    CLKCON = (CLKCON & ~CLKCON_CLKSPD_BM) | CLKCON_OSC_BM | CLOCK_SRC_HFRC;            // change system clock source to HS RCOSC
    while (!(CLKCON & CLKCON_OSC_BM));  // wait until CLKCON.OSC = 1 
                                        // (system clock running on HS RCOSC)
    SLEEP |= SLEEP_OSC_PD_BM;           // power down the unused oscillator
  }
  else if (source == CLOCK_SRC_XOSC)
  {
    SLEEP &= ~SLEEP_OSC_PD_BM;          	// power up both oscillators
    while (!(SLEEP & SLEEP_XOSC_STB_BM));	// Wait until HS XOSC is stable
    CLKCON &= ~CLKCON_OSC_BM;           	// change system clock source to HS XOSC
    while (CLKCON & CLKCON_OSC_BM);    	// wait until CLKCON.OSC = 0 
                                        // (system clock running on HS XOSC)
    SLEEP |= SLEEP_OSC_PD_BM;         	// power down the unused oscillator
  }
} 

int move_motor(steps,step_inc){
  steps = stesps + step_inc;
  
  return steps;
}
void SendPacket()
{
   // Construct packet  
    txBuffer[0] = CTRL_LENGTH;
    txBuffer[1] = 0 | ctrlPkt;    
    txBuffer[1] = txBuffer[1] | INSTALL_ID;               // packet[0] = Paket type 1 = control packet       
    txBuffer[2] = txNode;                                 // packet[2] = txNode
    txBuffer[3] = (steps << 1 )| 1;                         // packet[3] = Maximum number of nodes
    txBuffer[4] = sequenceNumber >> 8;                   // packet[10] = sequenceNumber high byte
    //txBuffer[4] = TICKSPD_GET();                        // packet[11] = sequenceNumber low byte
    //txBuffer[4] = T2count;                   // packet[10] = sequenceNumber high byte
    txBuffer[5] = sequenceNumber ;                        // packet[11] = sequenceNumber low byte
     
    if (txRadioPacket(txBuffer,&dmaConfig[0]) == 1) {
      // Packet successfully transmitted, increment sequence number
      sequenceNumber++;
      //rnd = 0x00;
      //P1_0 ^= 1;
     
      
      while(!tempResetWDT());
    }else{
      //P1_3  ^= 1;
        // Add random wait?
      //sequenceNumber = 0; // Debugging reset to 0...
    }

}
  #pragma vector = T2_VECTOR
  __interrupt void t2_interrupt (void)
  {

    P1_0 ^= 1;  // Change antenna
   //SendPacket();
    txNode++;
    if (txNode == 10) {
	SendFlag = TRUE;
	// 
        txNode = 0;
	//P1_5 ^= 1;
    } 
    else if (txNode == 1 ){
      CalFlag = TRUE;
     // P1_5 ^= 1;
    }
    else {
      // 
      //P1_5 ^= 1;
     }   
     T2Sqn++;       

  }
      

/***********************************************
* @fn  SendSQN2Uart
*
* @brief
*     send the SQN to UART
*       
*
* Parameters:
*
* @param  void
*
* @return void
*
* !!!!!!!!!txBuffer[1] is the field length used with VLEN not txbuffer[0] !!!!!!!!!!
*
******************************************************************************/

void SendSQN2Uart()
{
  //memset(UartTxBuffer,0xCC,SIZE_OF_TX_BUFFER);
    UartTxBuffer[0] = 0x02;  // this tells me which buffer it is
    UartTxBuffer[1] = 0x05;
    UartTxBuffer[2] = sequenceNumber >> 8;
    UartTxBuffer[3] = sequenceNumber;    
    UartTxBuffer[4] = 0xFF;
    UartTxBuffer[5] = 0xFF;
//P1_0 ^= 1;
    uart0StartTxDmaChan(UART_NUMBER,&dmaConfig[1], 0, UartTxBuffer, SIZE_OF_TX_BUFFER);    




}

#pragma vector = DMA_VECTOR
__interrupt void DMA_ISR(void)
{
//  P1_0 = 1;
  IRCON &= ~0x01;
  // Clear the main DMA interrupt Request Flag (IRCON.DMAIF = 0)
  //DMAIF = 0;

  // Start a new UART RX session on DMA channel 1:
  if (DMAIRQ & DMAIRQ_DMAIF0)
  {
    // Indicate UART free
    UART_DMA_clear = TRUE;

    // Clear DMA Channel 0 Interrupt Request Flag (DMAIRQ.DMAIF0 = 0)
    DMAIRQ &= ~DMAIRQ_DMAIF0;
  }
}
