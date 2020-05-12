#include "main.h"
#include "cmsis_os.h"
#include "adc.h"
ADC_HandleTypeDef hadc01;
uint32_t CHek_ADC[6];
extern uint32_t adcbuf[120];
 uint16_t adcx;  
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){
	(void)hadc;
	CHek_ADC[0]=adcbuf[0];
	CHek_ADC[1]=adcbuf[1];
	CHek_ADC[2]=adcbuf[2];
	CHek_ADC[3]=adcbuf[3];
	CHek_ADC[4]=adcbuf[4];
	adcx=(CHek_ADC[2]*3300L)/4096 ;
//	adcx=(uint32_t)adcx*3300/4096; 
}
