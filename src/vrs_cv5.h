/*
 * vrs_cv5.h
 *
 *  Created on: 18. 10. 2016
 *      Author: Tono
 */

#ifndef VRS_CV5_H_
#define VRS_CV5_H_

void initNVIC();

void ADC1_IRQHandler (void);

void ledInit();

void adc_init(void);



#endif /* VRS_CV5_H_ */
