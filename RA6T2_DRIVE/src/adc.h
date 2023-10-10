/*
 * adc.h
 *
 *  Created on: 29 sept. 2023
 *      Author: Ch.Leclercq
 */

#ifndef ADC_H_
#define ADC_H_
#include <_core/c_common.h>

#define ADC_CHANNEL_VIN     ADC_CHANNEL_14
#define ADC_CHANNEL_VBAT    ADC_CHANNEL_15
#define ADC_CHANNEL_VM      ADC_CHANNEL_2
#define ADC_CHANNEL_VHALL1  ADC_CHANNEL_26
#define ADC_CHANNEL_VHALL2  ADC_CHANNEL_27

void adc_init(void);
return_t adc_get_measures(void);


#endif /* ADC_H_ */
