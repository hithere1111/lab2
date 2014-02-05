// ADC.h
// Bryan Brumm
// Contains the declarations of low-level ADC functions
// Updated: 1/17/14

int ADC_Open(int channelNum); 

void ADC_In(int ); 

int* ADC_Collect(int channelNum, int fs, 
 int buffer[], int numberOfSamples);

int ADC_Status(void);

void ADC_Init(int channelNum);

