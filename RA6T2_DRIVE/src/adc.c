/*
 * adc.c
 *
 *  Created on: 29 sept. 2023
 *      Author: Ch.Leclercq
 */


#include <hal_data.h>
#include "adc.h"
#include "hal_data.h"



volatile adc_instance_t *ptr_adc=0x00;
volatile uint16_t adc_raw_buffer[5];



void adc_init(void)
{
   volatile motor_120_degree_extended_cfg_t * p_extended_cfg = (motor_120_degree_extended_cfg_t *) g_motor_120_degree0.p_cfg->p_extend;
   volatile motor_120_control_cfg_t * p_extended_hall_cfg = (motor_120_control_cfg_t*)p_extended_cfg->p_motor_120_control_instance->p_cfg;
   volatile motor_120_control_hall_extended_cfg_t * p_extended_control_hall_cfg = (motor_120_control_hall_extended_cfg_t *) p_extended_hall_cfg->p_extend;
   volatile motor_120_driver_cfg_t * p_driver_cfg = (motor_120_driver_cfg_t*)p_extended_control_hall_cfg->p_motor_120_driver_instance->p_cfg;
   volatile motor_120_driver_extended_cfg_t * p_driver_extended_cfg = (motor_120_driver_extended_cfg_t *) p_driver_cfg->p_extend;
   ptr_adc = (adc_instance_t*)p_driver_extended_cfg->p_adc_instance;

   memset(adc_raw_buffer,0x00,sizeof(adc_raw_buffer));
}

return_t adc_get_measures(void)
{
  return_t ret = X_RET_OK;

  ptr_adc->p_api->read(ptr_adc->p_ctrl,ADC_CHANNEL_VIN,&adc_raw_buffer[0]);
  ptr_adc->p_api->read(ptr_adc->p_ctrl,ADC_CHANNEL_VBAT,&adc_raw_buffer[1]);
  ptr_adc->p_api->read(ptr_adc->p_ctrl,ADC_CHANNEL_VM,&adc_raw_buffer[2]);
  ptr_adc->p_api->read(ptr_adc->p_ctrl,ADC_CHANNEL_VHALL1,&adc_raw_buffer[3]);
  ptr_adc->p_api->read(ptr_adc->p_ctrl,ADC_CHANNEL_VHALL2,&adc_raw_buffer[4]);

  volatile uint8_t i=0;

  return ret;
}
