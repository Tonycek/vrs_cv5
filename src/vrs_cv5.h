/*
 * vrs_cv5.h
 *
 *  Created on: 18. 10. 2016
 *      Author: Tono
 */

#ifndef VRS_CV5_H_
#define VRS_CV5_H_

extern uint16_t value;

void initNVIC();

void initUSART();

void ADC1_IRQHandler (void);

void ledInit();

void adc_init(void);

void PutcUART1(char ch);

void RegisterCallbackUART1(void *callback);

void USART1_IRQHandler(void);

void stav(uint16_t hodnota);

#endif /* VRS_CV5_H_ */
