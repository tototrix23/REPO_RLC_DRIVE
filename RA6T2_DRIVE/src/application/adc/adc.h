/*
 * adc.h
 *
 *  Created on: 11 oct. 2023
 *      Author: Ch.Leclercq
 */

#ifndef APPLICATION_ADC_ADC_H_
#define APPLICATION_ADC_ADC_H_

#include <stdint.h>
#include <hal_data.h>
#include <_core/c_common.h>
#include <return_codes.h>



typedef struct st_adc_t
{
    uint16_t iin;
    uint16_t vin;
    uint16_t vbatt;
    uint16_t vhall1;
    uint16_t vhall2;
}st_adc_t;

extern st_adc_t adc_inst;

return_t adc_init(void);
return_t adc_capture(void);
return_t adc_measures_update(void);








#endif /* APPLICATION_ADC_ADC_H_ */
