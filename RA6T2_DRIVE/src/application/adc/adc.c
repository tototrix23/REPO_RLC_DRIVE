/*
 * adc.c
 *
 *  Created on: 11 oct. 2023
 *      Author: Ch.Leclercq
 */
#include <config.h>
#include <_core/c_protected/c_protected.h>
#include <motor/motor.h>
#include "adc.h"

st_adc_t adc_inst;

float adc_iin=0.0;
float adc_vin=0.0;
float adc_hall1=0.0;
float adc_hall2=0.0;
float adc_vbatt=0.0;
uint16_t adc_uiin;
return_t adc_init(void)
{
    return_t ret = X_RET_OK;

    memset(&adc_inst,0x00,sizeof(st_adc_t));
    volatile fsp_err_t err = FSP_SUCCESS;
    err = R_ADC_B_Open (&g_adc_external_ctrl, &g_adc_external_cfg);
    if (err != FSP_SUCCESS)
    {
        ERROR_SET_AND_RETURN(F_RET_ERROR_GENERIC);
    }
    err = R_ADC_B_ScanCfg (&g_adc_external_ctrl, &g_adc_external_scan_cfg);
    if (err != FSP_SUCCESS)
    {
        R_ADC_B_Close (&g_adc_external_ctrl);
        ERROR_SET_AND_RETURN(F_RET_ERROR_GENERIC);
    }

    err = R_ADC_B_Calibrate (&g_adc_external_ctrl, NULL);
    if (err != FSP_SUCCESS)
    {
        R_ADC_B_Close (&g_adc_external_ctrl);
        ERROR_SET_AND_RETURN(F_RET_ERROR_GENERIC);
    }
    R_BSP_SoftwareDelay (10, BSP_DELAY_UNITS_MILLISECONDS);
    err = R_ADC_B_ScanStart (&g_adc_external_ctrl);
    if (err != FSP_SUCCESS)
    {
        R_ADC_B_Close (&g_adc_external_ctrl);
        ERROR_SET_AND_RETURN(F_RET_ERROR_GENERIC);
    }

    return ret;

}

return_t adc_capture(void)
{
    return_t ret = X_RET_OK;
    fsp_err_t err = R_ADC_B_ScanGroupStart(&g_adc_external_ctrl,ADC_GROUP_MASK_8);
    if(err != FSP_SUCCESS)
    {
        ERROR_SET_AND_RETURN(F_RET_ERROR_GENERIC);
    }
    return ret;
}

return_t adc_measures_update(void)
{
    float f_iin;
    float f_vin;
    float f_vbatt;
    float f_vhall1;
    float f_vhall2;

    c_protected_get_float(&adc_iin, &f_iin);
    c_protected_get_float(&adc_vin, &f_vin);
    c_protected_get_float(&adc_vbatt, &f_vbatt);
    c_protected_get_float(&adc_hall1, &f_vhall1);
    c_protected_get_float(&adc_hall2, &f_vhall2);

    adc_inst.vin = (uint16_t)ADC_VIN_ADAPT(f_vin);
    adc_inst.vbatt = (uint16_t)ADC_VBATT_ADAPT(f_vbatt);
    adc_inst.iin = (uint16_t)ADC_IIN_ADAPT(f_iin);
    adc_inst.vhall1 = (uint16_t)ADC_VHALL1_ADAPT(f_vhall1);
    adc_inst.vhall2 = (uint16_t)ADC_VHALL2_ADAPT(f_vhall2);

    return X_RET_OK;
}

void custom_adc_interrupt(adc_callback_args_t *p_args)
{


    if (p_args->event == ADC_EVENT_SCAN_COMPLETE)
    {
        if (p_args->group_mask == ADC_GROUP_MASK_0)
        {
            p_args->p_context = motors_instance.motorH->motor_driver_instance;//  motor0.motor_driver_instance;//  &g_mot_120_driver0;
            rm_motor_120_driver_cyclic(p_args);
        }
        if (p_args->group_mask == ADC_GROUP_MASK_2)
        {
            p_args->p_context = motors_instance.motorL->motor_driver_instance;//motor1.motor_driver_instance;//&g_mot_120_driver1;
            rm_motor_120_driver_cyclic(p_args);
        }
        if (p_args->group_mask == ADC_GROUP_MASK_8)
        {
            uint16_t data[5] = {0};
            R_ADC_B_Read(&g_adc_external_ctrl, ADC_CHANNEL_VIN,&data[0]);
            R_ADC_B_Read(&g_adc_external_ctrl, ADC_CHANNEL_VBATT,&data[1]);
            R_ADC_B_Read(&g_adc_external_ctrl, ADC_CHANNEL_IIN,&data[2]);
            R_ADC_B_Read(&g_adc_external_ctrl, ADC_CHANNEL_VHALL1,&data[3]);
            R_ADC_B_Read(&g_adc_external_ctrl, ADC_CHANNEL_VHALL2,&data[4]);


            volatile float v_vin = (((float)data[0]*3300.0f)/4096.0f);
            adc_vin = (adc_vin*(ADC_VIN_AVERAGE-1.0f))/(ADC_VIN_AVERAGE);
            adc_vin = adc_vin+(v_vin/ADC_VIN_AVERAGE);

            volatile float v_vbatt = (float)(((float)data[1]*3300.0f)/4096.0f);
            adc_vbatt = (adc_vbatt*(ADC_VBATT_AVERAGE-1.0f))/(ADC_VBATT_AVERAGE);
            adc_vbatt = adc_vbatt+(v_vbatt/ADC_VBATT_AVERAGE);

            volatile float v_iin = (float)(((float)data[2]*3300.0f)/4096.0f);
            adc_iin = (adc_iin*(ADC_IIN_AVERAGE-1.0f))/(ADC_IIN_AVERAGE);
            adc_iin = adc_iin+(v_iin/ADC_IIN_AVERAGE);

            volatile float v_vhall1 = (float)(((float)data[3]*3300.0f)/4096.0f);
            adc_hall1 = (adc_hall1*(ADC_VHALL1_AVERAGE-1.0f))/(ADC_VHALL1_AVERAGE);
            adc_hall1 = adc_hall1+(v_vhall1/ADC_VHALL1_AVERAGE);

            volatile float v_vhall2 = (float)(((float)data[4]*3300.0f)/4096.0f);
            adc_hall2 = (adc_hall2*(ADC_VHALL2_AVERAGE-1.0f))/(ADC_VHALL2_AVERAGE);
            adc_hall2 = adc_hall2+(v_vhall2/ADC_VHALL2_AVERAGE);
        }
    }
    else if(p_args->event == ADC_EVENT_CALIBRATION_COMPLETE || p_args->event == ADC_EVENT_CALIBRATION_REQUEST)
    {
        return;
    }
    else
    {

    }
}


