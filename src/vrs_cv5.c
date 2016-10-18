/*
 * vrs_cv5.c
 *
 *  Created on: 18. 10. 2016
 *      Author: Tono
 */
#include "stm32l1xx.h"
#include "vrs_cv5.h"

void ledInit(){
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
	      //RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);
	GPIO_InitTypeDef gpioInitStruc;
	gpioInitStruc.GPIO_Mode = GPIO_Mode_OUT;
	gpioInitStruc.GPIO_OType = GPIO_OType_PP;
	gpioInitStruc.GPIO_Pin = GPIO_Pin_5;
	gpioInitStruc.GPIO_Speed = GPIO_Speed_400KHz;

	GPIO_Init(GPIOA, &gpioInitStruc);
}

void initNVIC(){
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = ADC1_IRQn; //zoznam preru�en� n�jdete v s�bore stm32l1xx.h
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	ADC_ITConfig(ADC1, ADC_IT_EOC, ENABLE);

}

void ADC1_IRQHandler (void){
	//uint16_t value;
	if(ADC1->SR & ADC_SR_EOC)
	{
		uint16_t value = ADC1->DR;
	}
}

void adc_init(void)
{
 GPIO_InitTypeDef GPIO_InitStructure;
 ADC_InitTypeDef ADC_InitStructure;
 /* Enable GPIO clock */
 RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);//Opravi� a upravi�
 /* Configure ADCx Channel 2 as analog input */
 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 ;
 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
 GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
 GPIO_Init(GPIOA, &GPIO_InitStructure);
/* Enable the HSI oscillator */
 RCC_HSICmd(ENABLE);
/* Check that HSI oscillator is ready */
 while(RCC_GetFlagStatus(RCC_FLAG_HSIRDY) == RESET);
 /* Enable ADC clock */
 RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
 /* Initialize ADC structure */
 ADC_StructInit(&ADC_InitStructure);
 /* ADC1 configuration */
 ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
 ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
 ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
 ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
 ADC_InitStructure.ADC_NbrOfConversion = 1;
 ADC_Init(ADC1, &ADC_InitStructure);
/* ADCx regular channel8 configuration */
 ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 1, ADC_SampleTime_16Cycles);
 /* Enable the ADC */
 ADC_Cmd(ADC1, ENABLE);
 /* Wait until the ADC1 is ready */
 while(ADC_GetFlagStatus(ADC1, ADC_FLAG_ADONS) == RESET)
 {
 }
 /* Start ADC Software Conversion */
 ADC_SoftwareStartConv(ADC1);
}
