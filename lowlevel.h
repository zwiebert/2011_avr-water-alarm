#ifndef LOWLEVEL_H_
#define LOWLEVEL_H_



int analogRead(uint8_t pin);


void analogReference(uint8_t mode);
#define INTERNAL 3
#define DEFAULT 1
#define EXTERNAL 0



void ADC_Init(void);
uint16_t ADC_Read(uint8_t channel);
uint16_t ADC_Read_Avg(uint8_t channel, uint8_t average);





#endif /* LOWLEVEL_H_ */
