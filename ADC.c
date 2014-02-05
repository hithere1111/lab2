// ADC.c
// Bryan Brumm
// Contains functions for ADC collection
// Updated: 1/17/14

/* code adopted from Dr. Valvano's ADCT0ATrigger.c file on http://users.ece.utexas.edu/~valvano/ */

#include "ADC.h"

#define NVIC_EN0_R              (*((volatile unsigned long *)0xE000E100))  // IRQ 0 to 31 Set Enable Register
   
#define NVIC_PRI3_R             (*((volatile unsigned long *)0xE000E40C))  // IRQ 12 to 15 Priority Register
#define NVIC_PRI4_R             (*((volatile unsigned long *)0xE000E410))  // IRQ 16 to 19 Priority Register
   
#define NVIC_EN0_INT17          0x00020000   // Interrupt 17 enable ADC0 SS3
#define NVIC_EN0_INT16          0x00010000   // Interrupt 16 enable ADC0 SS2
#define NVIC_EN0_INT15          0x00008000   // Interrupt 15 enable ADC0 SS1
#define NVIC_EN0_INT14          0x00004000   // Interrupt 14 enable ADC0 SS0
   
#define GPIO_PORTB_DIR_R        (*((volatile unsigned long *)0x40005400))
#define GPIO_PORTB_AFSEL_R      (*((volatile unsigned long *)0x40005420))
#define GPIO_PORTB_DEN_R        (*((volatile unsigned long *)0x4000551C))
#define GPIO_PORTB_AMSEL_R      (*((volatile unsigned long *)0x40005528))
   
#define GPIO_PORTD_DIR_R        (*((volatile unsigned long *)0x40007400))
#define GPIO_PORTD_AFSEL_R      (*((volatile unsigned long *)0x40007420))
#define GPIO_PORTD_DEN_R        (*((volatile unsigned long *)0x4000751C))
#define GPIO_PORTD_AMSEL_R      (*((volatile unsigned long *)0x40007528))
   
#define GPIO_PORTE_DIR_R        (*((volatile unsigned long *)0x40024400))
#define GPIO_PORTE_AFSEL_R      (*((volatile unsigned long *)0x40024420))
#define GPIO_PORTE_DEN_R        (*((volatile unsigned long *)0x4002451C))
#define GPIO_PORTE_AMSEL_R      (*((volatile unsigned long *)0x40024528))
   
#define TIMER0_CFG_R            (*((volatile unsigned long *)0x40030000))
#define TIMER0_TAMR_R           (*((volatile unsigned long *)0x40030004))
#define TIMER0_CTL_R            (*((volatile unsigned long *)0x4003000C))
#define TIMER0_IMR_R            (*((volatile unsigned long *)0x40030018))
#define TIMER0_TAILR_R          (*((volatile unsigned long *)0x40030028))
#define TIMER0_TAPR_R           (*((volatile unsigned long *)0x40030038))
#define TIMER_CFG_16_BIT        0x00000004  // 16-bit timer configuration,
                                            // function is controlled by bits
                                            // 1:0 of GPTMTAMR and GPTMTBMR
#define TIMER_TAMR_TACDIR       0x00000010  // GPTM Timer A Count Direction
#define TIMER_TAMR_TAMR_PERIOD  0x00000002  // Periodic Timer mode
#define TIMER_CTL_TAOTE         0x00000020  // GPTM TimerA Output Trigger
                                            // Enable
#define TIMER_CTL_TAEN          0x00000001  // GPTM TimerA Enable
#define TIMER_IMR_TATOIM        0x00000001  // GPTM TimerA Time-Out Interrupt
                                            // Mask
#define TIMER_TAILR_TAILRL_M    0x0000FFFF  // GPTM TimerA Interval Load
                                            // Register Low
#define TIMER_PRESCALE_V         199        // prescale value for periodic timer
#define TIMER_PERIOD_V           199        // period value for periodic timer 
#define TIMER_PERIOD_D           800000     // dividend for easy period calculation
#define TIMER_PRESCALE_EASY_V    99         // when using easy period calculation
       
#define ADC0_ACTSS_R            (*((volatile unsigned long *)0x40038000))
#define ADC0_RIS_R              (*((volatile unsigned long *)0x40038004))
#define ADC0_IM_R               (*((volatile unsigned long *)0x40038008))
#define ADC0_ISC_R              (*((volatile unsigned long *)0x4003800C))
#define ADC0_EMUX_R             (*((volatile unsigned long *)0x40038014))
#define ADC0_SSPRI_R            (*((volatile unsigned long *)0x40038020))
#define ADC0_PSSI_R             (*((volatile unsigned long *)0x40038028))
#define ADC0_PC_R               (*((volatile unsigned long *)0x40038FC4))
   
#define ADC0_SSFSTAT0_R         (*((volatile unsigned long *)0x4003804C))

#define ADC0_SSFSTAT0_EMPTY     0x0001000  // FIFO is empty
   
#define ADC0_SSFIFO3_R          (*((volatile unsigned long *)0x400380A8))
#define ADC0_SSFIFO2_R          (*((volatile unsigned long *)0x40038088))
#define ADC0_SSFIFO1_R          (*((volatile unsigned long *)0x40038068))
#define ADC0_SSFIFO0_R          (*((volatile unsigned long *)0x40038048))

#define ADC0_SSCTL3_R           (*((volatile unsigned long *)0x400380A4))
#define ADC0_SSCTL2_R           (*((volatile unsigned long *)0x40038084))
#define ADC0_SSCTL1_R           (*((volatile unsigned long *)0x40038064))
#define ADC0_SSCTL0_R           (*((volatile unsigned long *)0x40038044))

#define ADC0_SSMUX3_R           (*((volatile unsigned long *)0x400380A0))
#define ADC0_SSMUX2_R           (*((volatile unsigned long *)0x40038080))
#define ADC0_SSMUX1_R           (*((volatile unsigned long *)0x40038060))
#define ADC0_SSMUX0_R           (*((volatile unsigned long *)0x40038040))

#define ADC_ACTSS_ASEN3         0x00000008  // ADC SS3 Enable
#define ADC_ACTSS_ASEN2         0x00000004  // ADC SS2 Enable
#define ADC_ACTSS_ASEN1         0x00000002  // ADC SS1 Enable
#define ADC_ACTSS_ASEN0         0x00000001  // ADC SS0 Enable

#define ADC_RIS_INR3            0x00000008  // SS3 Raw Interrupt Status
#define ADC_RIS_INR2            0x00000004  // SS2 Raw Interrupt Status
#define ADC_RIS_INR1            0x00000002  // SS1 Raw Interrupt Status
#define ADC_RIS_INR0            0x00000001  // SS0 Raw Interrupt Status

#define ADC_IM_MASK3            0x00000008  // SS3 Interrupt Mask
#define ADC_IM_MASK2            0x00000004  // SS2 Interrupt Mask
#define ADC_IM_MASK1            0x00000002  // SS1 Interrupt Mask
#define ADC_IM_MASK0            0x00000001  // SS0 Interrupt Mask

#define ADC_ISC_IN3             0x00000008  // SS3 Interrupt Status and Clear
#define ADC_ISC_IN2             0x00000004  // SS2 Interrupt Status and Clear
#define ADC_ISC_IN1             0x00000002  // SS1 Interrupt Status and Clear
#define ADC_ISC_IN0             0x00000001  // SS0 Interrupt Status and Clear

#define ADC_EMUX_EM3_M          0x0000F000  // SS3 Trigger Select mask
#define ADC_EMUX_EM2_M          0x00000F00  // SS2 Trigger Select mask
#define ADC_EMUX_EM1_M          0x000000F0  // SS1 Trigger Select mask
#define ADC_EMUX_EM0_M          0x0000000F  // SS0 Trigger Select mask

#define ADC_EMUX_EM3_TIMER      0x00005000  // Timer mask for SS3
#define ADC_EMUX_EM2_TIMER      0x00000500  // Timer mask for SS2 
#define ADC_EMUX_EM1_TIMER      0x00000050  // Timer mask for SS1
#define ADC_EMUX_EM0_TIMER      0x00000005  // Timer mask for SS0

#define ADC_SSMUX3_MUX0_M       0x0000000F  // 1st Sample Input Select mask
#define ADC_SSMUX0_MUX1_M       0x000000F0  // 2nd Sample Input Select mask
#define ADC_SSMUX0_MUX2_M       0x00000F00  // 3rd Sample Input Select mask
#define ADC_SSMUX0_MUX3_M       0x0000F000  // 4th Sample Input Select mask
#define ADC_SSMUX0_MUX4_M       0x000F0000  // 5th Sample Input Select mask
#define ADC_SSMUX0_MUX5_M       0x00F00000  // 6th Sample Input Select mask
#define ADC_SSMUX0_MUX6_M       0x0F000000  // 7th Sample Input Select mask
#define ADC_SSMUX0_MUX7_M       0xF0000000  // 8th Sample Input Select mask

#define ADC_SSCTL3_TS0          0x00000008  // 1st Sample Temp Sensor Select
#define ADC_SSCTL3_IE0          0x00000004  // 1st Sample Interrupt Enable
#define ADC_SSCTL3_END0         0x00000002  // 1st Sample is End of Sequence
#define ADC_SSCTL3_D0           0x00000001  // 1st Sample Diff Input Select

#define ADC_SSPRI_SS3_4TH       0x00003000  // fourth priority
#define ADC_SSPRI_SS2_3RD       0x00000200  // third priority
#define ADC_SSPRI_SS1_2ND       0x00000010  // second priority
#define ADC_SSPRI_SS0_1ST       0x00000000  // first priority

#define ADC0_ACTSS_BUSY_M       0x00010000
#define ADC_PSSI_SS3            0x00000008  // SS3 Initiate
#define ADC_SSFIFO3_DATA_M       0x00000FFF  // Conversion Result Data mask
#define ADC_PC_SR_M             0x0000000F  // ADC Sample Rate
#define ADC_PC_SR_125K          0x00000001  // 125 ksps
#define SYSCTL_RCGC0_R          (*((volatile unsigned long *)0x400FE100))
#define SYSCTL_RCGC1_R          (*((volatile unsigned long *)0x400FE104))
#define SYSCTL_RCGC0_ADC0       0x00010000  // ADC0 Clock Gating Control
#define SYSCTL_RCGC0_ADCSPD_M   0x00000300  // ADC Sample Speed mask
#define SYSCTL_RCGC1_TIMER0     0x00010000  // timer 0 Clock Gating Control
#define SYSCTL_RCGCGPIO_R       (*((volatile unsigned long *)0x400FE608))
#define SYSCTL_RCGCGPIO_R4      0x00000010  // GPIO Port E Run Mode Clock
                                            // Gating Control
#define SYSCTL_RCGCGPIO_R3      0x00000008  // GPIO Port D Run Mode Clock
                                            // Gating Control
#define SYSCTL_RCGCGPIO_R1      0x00000002  // GPIO Port B Run Mode Clock
                                            // Gating Control
#define FAIL                    0           // ADC could not initialize       
#define NO_FAIL                 1           // ADC initialized correctly
#define HALF                    FIFOSIZE/2  // ADC doesn't overflow

#define MAX_SAMPLES 200
int DataBuffer[MAX_SAMPLES];                // Shared Global to store Data
volatile unsigned long ADCvalue;
                                                                                       
void DisableInterrupts(void); // Disable interrupts
void EnableInterrupts(void);  // Enable interrupts
long StartCritical (void);    // previous I bit, disable interrupts
void EndCritical(long sr);    // restore I bit to previous value
void WaitForInterrupt(void);  // low power mode

//---------------ADC_Open----------------
// Initializes a specified ADC port
// [Placeholder]
// Inputs: [channelNum] is the port you wish to initialize
// Outputs: None
int ADC_Open(int channelNum) {
  volatile unsigned long delay;
  // **** GPIO pin initialization ****
  switch(channelNum){             // 1) activate clock
    case 0:
    case 1:
    case 2:
    case 3:
    case 8:
    case 9:                       //    these are on GPIO_PORTE
      SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R4; break;
    case 4:
    case 5:
    case 6:
    case 7:                       //    these are on GPIO_PORTD
      SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R3; break;
    case 10:
    case 11:                      //    these are on GPIO_PORTB
      SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R1; break;
    default: return FAIL;              //    0 to 11 are valid channels on the TM4C123
  }
  
  delay = SYSCTL_RCGCGPIO_R;      // 2) allow time for clock to stabilize
  delay = SYSCTL_RCGCGPIO_R;
  switch(channelNum){
    case 0:                       //      Ain0 is on PE3
      GPIO_PORTE_DIR_R &= ~0x08;  // 3.0) make PE3 input
      GPIO_PORTE_AFSEL_R |= 0x08; // 4.0) enable alternate function on PE3
      GPIO_PORTE_DEN_R &= ~0x08;  // 5.0) disable digital I/O on PE3
      GPIO_PORTE_AMSEL_R |= 0x08; // 6.0) enable analog functionality on PE3
      break;
    case 1:                       //      Ain1 is on PE2
      GPIO_PORTE_DIR_R &= ~0x04;  // 3.1) make PE2 input
      GPIO_PORTE_AFSEL_R |= 0x04; // 4.1) enable alternate function on PE2
      GPIO_PORTE_DEN_R &= ~0x04;  // 5.1) disable digital I/O on PE2
      GPIO_PORTE_AMSEL_R |= 0x04; // 6.1) enable analog functionality on PE2
      break;
    case 2:                       //      Ain2 is on PE1
      GPIO_PORTE_DIR_R &= ~0x02;  // 3.2) make PE1 input
      GPIO_PORTE_AFSEL_R |= 0x02; // 4.2) enable alternate function on PE1
      GPIO_PORTE_DEN_R &= ~0x02;  // 5.2) disable digital I/O on PE1
      GPIO_PORTE_AMSEL_R |= 0x02; // 6.2) enable analog functionality on PE1
      break;
    case 3:                       //      Ain3 is on PE0
      GPIO_PORTE_DIR_R &= ~0x01;  // 3.3) make PE0 input
      GPIO_PORTE_AFSEL_R |= 0x01; // 4.3) enable alternate function on PE0
      GPIO_PORTE_DEN_R &= ~0x01;  // 5.3) disable digital I/O on PE0
      GPIO_PORTE_AMSEL_R |= 0x01; // 6.3) enable analog functionality on PE0
      break;
    case 4:                       //      Ain4 is on PD3
      GPIO_PORTD_DIR_R &= ~0x08;  // 3.4) make PD3 input
      GPIO_PORTD_AFSEL_R |= 0x08; // 4.4) enable alternate function on PD3
      GPIO_PORTD_DEN_R &= ~0x08;  // 5.4) disable digital I/O on PD3
      GPIO_PORTD_AMSEL_R |= 0x08; // 6.4) enable analog functionality on PD3
      break;
    case 5:                       //      Ain5 is on PD2
      GPIO_PORTD_DIR_R &= ~0x04;  // 3.5) make PD2 input
      GPIO_PORTD_AFSEL_R |= 0x04; // 4.5) enable alternate function on PD2
      GPIO_PORTD_DEN_R &= ~0x04;  // 5.5) disable digital I/O on PD2
      GPIO_PORTD_AMSEL_R |= 0x04; // 6.5) enable analog functionality on PD2
      break;
    case 6:                       //      Ain6 is on PD1
      GPIO_PORTD_DIR_R &= ~0x02;  // 3.6) make PD1 input
      GPIO_PORTD_AFSEL_R |= 0x02; // 4.6) enable alternate function on PD1
      GPIO_PORTD_DEN_R &= ~0x02;  // 5.6) disable digital I/O on PD1
      GPIO_PORTD_AMSEL_R |= 0x02; // 6.6) enable analog functionality on PD1
      break;
    case 7:                       //      Ain7 is on PD0
      GPIO_PORTD_DIR_R &= ~0x01;  // 3.7) make PD0 input
      GPIO_PORTD_AFSEL_R |= 0x01; // 4.7) enable alternate function on PD0
      GPIO_PORTD_DEN_R &= ~0x01;  // 5.7) disable digital I/O on PD0
      GPIO_PORTD_AMSEL_R |= 0x01; // 6.7) enable analog functionality on PD0
      break;
    case 8:                       //      Ain8 is on PE5
      GPIO_PORTE_DIR_R &= ~0x20;  // 3.8) make PE5 input
      GPIO_PORTE_AFSEL_R |= 0x20; // 4.8) enable alternate function on PE5
      GPIO_PORTE_DEN_R &= ~0x20;  // 5.8) disable digital I/O on PE5
      GPIO_PORTE_AMSEL_R |= 0x20; // 6.8) enable analog functionality on PE5
      break;
    case 9:                       //      Ain9 is on PE4
      GPIO_PORTE_DIR_R &= ~0x10;  // 3.9) make PE4 input
      GPIO_PORTE_AFSEL_R |= 0x10; // 4.9) enable alternate function on PE4
      GPIO_PORTE_DEN_R &= ~0x10;  // 5.9) disable digital I/O on PE4
      GPIO_PORTE_AMSEL_R |= 0x10; // 6.9) enable analog functionality on PE4
      break;
    case 10:                      //       Ain10 is on PB4
      GPIO_PORTB_DIR_R &= ~0x10;  // 3.10) make PB4 input
      GPIO_PORTB_AFSEL_R |= 0x10; // 4.10) enable alternate function on PB4
      GPIO_PORTB_DEN_R &= ~0x10;  // 5.10) disable digital I/O on PB4
      GPIO_PORTB_AMSEL_R |= 0x10; // 6.10) enable analog functionality on PB4
      break;
    case 11:                      //       Ain11 is on PB5
      GPIO_PORTB_DIR_R &= ~0x20;  // 3.11) make PB5 input
      GPIO_PORTB_AFSEL_R |= 0x20; // 4.11) enable alternate function on PB5
      GPIO_PORTB_DEN_R &= ~0x20;  // 5.11) disable digital I/O on PB5
      GPIO_PORTB_AMSEL_R |= 0x20; // 6.11) enable analog functionality on PB5
      break;
  }
  return NO_FAIL;
}


//---------------ADC_In------------------
// Start the ADC collection
// [Placeholder]
// Inputs: None
// Outputs: Value from ADC
void ADC_In(int samples) {
   while(ADCvalue != samples){};
   ADC0_ACTSS_R &= ~ADC_ACTSS_ASEN3;           // disable sample sequencer 3
   ADC0_IM_R &= ~ADC_IM_MASK3;                 // disable SS3 interrupts
  
}

// calculate period value for periodic timer interrupts
// frequency range 100-10000 Hz
// can be inaccurate by 1 Hz when approaching 10000 Hz
int timerPeriod(int fs){
   return (TIMER_PERIOD_D / fs) - 1;
}

//---------------ADC_Init----------------
// Activates the ADC for a specified sequencer and number of samples/sec
// Currently only works for sequencer 0
// Inputs: [seq] the sequencer that should be activated; [numSample] number of samples/sec
//         [channelNum] the ADC channel being used
// Outputs: None
void ADC_Init(int channelNum){
  // **** ADC initialization ****
  ADC0_PC_R &= ~ADC_PC_SR_M;                // clear max sample rate field
  ADC0_PC_R |= ADC_PC_SR_125K;              // configure for 125K samples/sec
                                            // sequencer 0 is highest priority
                                            // sequencer 1 is second-highest priority
                                            // sequencer 2 is third-highest priority
                                            // sequencer 3 is lowest priority
  ADC0_SSPRI_R = (ADC_SSPRI_SS0_1ST|ADC_SSPRI_SS1_2ND|ADC_SSPRI_SS2_3RD|ADC_SSPRI_SS3_4TH);
  ADC0_ACTSS_R &= ~ADC_ACTSS_ASEN3;         // disable sample sequencer 3
  ADC0_EMUX_R &= ~ADC_EMUX_EM3_M;           // clear SS3 trigger select field
  ADC0_EMUX_R += ADC_EMUX_EM3_TIMER;        // configure for timer trigger event
  ADC0_SSMUX3_R &= ~ADC_SSMUX3_MUX0_M;      // clear SS3 1st sample input select field
                                            // configure for 'channelNum' as first sample input
  ADC0_SSMUX3_R += channelNum;
  ADC0_SSCTL3_R = ((0                        // settings for 1st sample:
                   & ~ADC_SSCTL3_TS0)        // read pin specified by ADC0_SSMUX3_R
                   |  ADC_SSCTL3_IE0         // raw interrupt asserted here
                   |  (ADC_SSCTL3_END0       // sample is end of sequence (hardwired)
                   & ~ADC_SSCTL3_D0));       // differential mode not used
  ADC0_IM_R |= ADC_IM_MASK3;                 // enable SS3 interrupts
  ADC0_ACTSS_R |= ADC_ACTSS_ASEN3;           // enable sample sequencer 3
}

volatile unsigned long ADCvalue;
//---------------ADC_Collect-------------
// Samples with the ADC from a specified channel, frequency, and number of samples
// [Placeholder]
// Inputs: [channelNum] the ADC channel that is going to be sampled; [fs] the frequency;
//         [buffer[]] where to store the data; [numberOfSamples] how many samples
// Outputs: the ADC data
int* ADC_Collect(int channelNum, int fs, 
 int buffer[], int numberOfSamples) {
  int i = 0;
  volatile unsigned long delay;
  ADC_Open(channelNum);                     // activate correct GPIO ADC pins/clock
  
  DisableInterrupts();
  // **** general initialization ****
  SYSCTL_RCGC0_R |= SYSCTL_RCGC0_ADC0;      // activate ADC0 (legacy code)
  SYSCTL_RCGC1_R |= SYSCTL_RCGC1_TIMER0;    // activate timer0 (legacy code)
  delay = SYSCTL_RCGC1_R;                   // allow time to finish activating
  TIMER0_CTL_R &= ~TIMER_CTL_TAEN;          // disable timer0A during setup
  TIMER0_CTL_R |= TIMER_CTL_TAOTE;          // enable timer0A trigger to ADC
  TIMER0_CFG_R = TIMER_CFG_16_BIT;          // configure for 16-bit timer mode
  // **** timer0A initialization ****
  TIMER0_TAMR_R = TIMER_TAMR_TAMR_PERIOD;   // configure for periodic mode, default down-count settings
  TIMER0_TAPR_R = TIMER_PRESCALE_EASY_V;    // prescale value for trigger
  TIMER0_TAILR_R = timerPeriod(fs);         // start value for trigger
                                            // Clock Freq * (period + 1) * (prescale + 1) = timer interrupt
  TIMER0_IMR_R &= ~TIMER_IMR_TATOIM;        // disable timeout (rollover) interrupt
  TIMER0_CTL_R |= TIMER_CTL_TAEN;           // enable timer0A 16-b, periodic, no interrupts
  // **** adc initialization ****  
  ADC_Init(channelNum);
  // **** interrupt initialization ****   
                                                     // ADC3=priority 2
  NVIC_PRI4_R = (NVIC_PRI4_R&0xFFFF00FF)|0x00004000; // bits 13-15
  NVIC_EN0_R |= NVIC_EN0_INT17;                      // enable interrupt 17 in NVIC
  EnableInterrupts();
  
  ADC_In(numberOfSamples);
  
  for(i=0;i<numberOfSamples;i++){
     buffer[i] = DataBuffer[i];
  }
  return buffer;
 }

//---------------ADC_Status--------------
// Find out if ADC is done sampling
// [Placeholder]
// Inputs: None
// Outputs: 1-not done, 0-done
int ADC_Status(void) {
   int status = (ADC0_RIS_R&0x01);
   if(status == 1){ return 1; }
   else { return 0; }
}

void ADC0Seq3_Handler(void){
  ADC0_ISC_R = ADC_ISC_IN3;                 // acknowledge ADC sequence 3 completion
  DataBuffer[ADCvalue] = ADC0_SSFIFO3_R&ADC_SSFIFO3_DATA_M;
  ADCvalue++;
}
