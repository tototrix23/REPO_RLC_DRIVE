/***********************************************************************************************************************
* DISCLAIMER
* This software is supplied by Renesas Electronics Corporation and is only intended for use with Renesas products. No
* other uses are authorized. This software is owned by Renesas Electronics Corporation and is protected under all
* applicable laws, including copyright laws.
* THIS SOFTWARE IS PROVIDED "AS IS" AND RENESAS MAKES NO WARRANTIES REGARDING
* THIS SOFTWARE, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING BUT NOT LIMITED TO WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT. ALL SUCH WARRANTIES ARE EXPRESSLY DISCLAIMED. TO THE MAXIMUM
* EXTENT PERMITTED NOT PROHIBITED BY LAW, NEITHER RENESAS ELECTRONICS CORPORATION NOR ANY OF ITS AFFILIATED COMPANIES
* SHALL BE LIABLE FOR ANY DIRECT, INDIRECT, SPECIAL, INCIDENTAL OR CONSEQUENTIAL DAMAGES FOR ANY REASON RELATED TO THIS
* SOFTWARE, EVEN IF RENESAS OR ITS AFFILIATES HAVE BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGES.
* Renesas reserves the right, without notice, to make changes to this software and to discontinue the availability of
* this software. By using this software, you agree to the additional terms and conditions found by accessing the
* following link:
* http://www.renesas.com/disclaimer
*
* Copyright (C) 2021 Renesas Electronics Corporation. All rights reserved.
***********************************************************************************************************************/
/***********************************************************************************************************************
* File Name   : mtr_main.c
* Description : The main function and the processes of motor control application layer
***********************************************************************************************************************/
/**********************************************************************************************************************
* History : DD.MM.YYYY Version
*         : 09.12.2021 1.00
***********************************************************************************************************************/

/***********************************************************************************************************************
* Includes <System Includes> , "Project Includes"
***********************************************************************************************************************/
#include <stdint.h>
#include "mtr_main.h"
#include "rm_motor_api.h"

#include <hal_data.h>
#include <_core/c_common.h>
#include <_core/c_protected/c_protected.h>
#include <_interfaces/i_spi/i_spi.h>
#include <_interfaces/i_time/i_time.h>
#include <_hal/h_drv8316/h_drv8316.h>
#include <_hal/h_time/h_time.h>
#include <_lib_impl_cust/impl_spi_motors/impl_spi_motors.h>
#include <_lib_impl_cust/impl_time/impl_time.h>
#include <_lib_impl_cust/impl_log/impl_log.h>
#include <adc/adc.h>
#include <motor/motor.h>
#include <motor/drive_process/drive_process.h>
#include <remotectrl/remotectrl.h>


#undef  LOG_LEVEL
#define LOG_LEVEL     LOG_LVL_DEBUG
#undef  LOG_MODULE
#define LOG_MODULE    "main"



#define     CONF_MOTOR_TYPE ("Brushless DC Motor")
#define     CONF_CONTROL ("Hall 120 degree control (Speed control)")
#define     CONF_INVERTER ("MCI-LV-1")
#define     CONF_MOTOR_TYPE_LEN (18)
#define     CONF_CONTROL_LEN (39)
#define     CONF_INVERTER_LEN (8)
/***********************************************************************************************************************
* Global variables
***********************************************************************************************************************/
float       g_f4_speed_ref = 0;
uint8_t     g_u1_motor0_status;                    /* Motor status */
uint8_t     g_u1_motor1_status;                    /* Motor status */

uint8_t     com_u1_sw_userif;                     /* User interface switch */
uint8_t     g_u1_sw_userif;                       /* User interface switch */
uint8_t     com_u1_mode_system;                   /* System mode */
uint8_t     g_u1_mode_system;                     /* System mode */
float       g_f4_max_speed_rpm;
uint8_t     g_u1_stop_req;
uint16_t    g_u2_chk_error0;
uint16_t    g_u2_chk_error1;
uint16_t    g_u2_conf_hw;
uint16_t    g_u2_conf_sw;
uint16_t    g_u2_conf_tool;
uint8_t     g_u1_conf_motor_type_len;
uint8_t     g_u1_conf_control_len;
uint8_t     g_u1_conf_inverter_len;
uint8_t     g_u1_conf_motor_type[CONF_MOTOR_TYPE_LEN];
uint8_t     g_u1_conf_control[CONF_CONTROL_LEN];
uint8_t     g_u1_conf_inverter[CONF_INVERTER_LEN];
uint8_t     g_u1_reset_req;                       /* Reset request flag */
uint8_t     g_u1_sw_cnt;                          /* Counter to remove chattering */

motor_cfg_t g_user_motor_cfg;
motor_120_degree_extended_cfg_t g_user_motor_120_degree_extended_cfg;
motor_120_control_cfg_t g_user_motor_120_control_cfg;
motor_120_control_hall_extended_cfg_t g_user_motor_120_control_extended_cfg;
motor_120_driver_cfg_t g_user_motor_120_driver_cfg;
motor_120_driver_extended_cfg_t g_user_motor_120_driver_extended_cfg;

motor_cfg_t g_user_motor1_cfg;
motor_120_degree_extended_cfg_t g_user_motor1_120_degree_extended_cfg;
motor_120_control_cfg_t g_user_motor1_120_control_cfg;
motor_120_control_hall_extended_cfg_t g_user_motor1_120_control_extended_cfg;
motor_120_driver_cfg_t g_user_motor1_120_driver_cfg;
motor_120_driver_extended_cfg_t g_user_motor1_120_driver_extended_cfg;


motor_wait_stop_flag_t g_wait_flag;



h_drv8316_t drv_mot1;
h_drv8316_t drv_mot2;
i_spi_t interface_mot1;
i_spi_t interface_mot2;



/***********************************************************************************************************************
* Private functions
***********************************************************************************************************************/
static void     motor_fsp_init (void);

static uint8_t  mtr_remove_sw_chattering (uint8_t u1_sw, uint8_t u1_on_off);
static void     board_ui (void);           /* Board user interface */


void custom_adc_interrupt(adc_callback_args_t * p_args);




i_time_t i_time_interface_t;
c_timespan_t ts0;
c_timespan_t ts1;


/***********************************************************************************************************************
* Function Name : mtr_init
* Description   : Initialization for Motor Control
* Arguments     : None
* Return Value  : None
***********************************************************************************************************************/
void mtr_init(void)
{
   volatile int i=0;

   i_log.write_e = impl_log_write_e;
   i_log.write_d = impl_log_write_d;
   i_log.write_i = impl_log_write_i;
   i_log.write_w = impl_log_write_w;

   impl_time_init();
   i_time_init(&i_time_interface_t,impl_time_init, impl_time_update);
   h_time_init(&i_time_interface_t);

   R_BSP_SoftwareDelay(500, BSP_DELAY_UNITS_MILLISECONDS);
   R_IOPORT_PinWrite(&g_ioport_ctrl, VM_CMD,BSP_IO_LEVEL_HIGH );
   R_BSP_SoftwareDelay(200, BSP_DELAY_UNITS_MILLISECONDS);



   LOG_D(LOG_STD,"START");


   i_spi_init(&interface_mot1, spi_motor_open, spi_motor_close, spi_motor_read, spi_motor_write, spi_motor_mot1_cs_inactive, spi_motor_mot1_cs_active);
   i_spi_init(&interface_mot2, spi_motor_open, spi_motor_close, spi_motor_read, spi_motor_write, spi_motor_mot2_cs_inactive, spi_motor_mot2_cs_active);

  volatile return_t ret = h_drv8316_init(&drv_mot1,&interface_mot1,FALSE);
  ret = h_drv8316_init(&drv_mot2,&interface_mot2,FALSE);

  ret = h_drv8316_read_all_registers(&drv_mot1);
  drv_mot1.registers.ctrl2.bits.SLEW = 3;
  drv_mot1.registers.ctrl2.bits.PWM_MODE = 0;
  drv_mot1.registers.ctrl4.bits.OCP_MODE = 0;
  drv_mot1.registers.ctrl4.bits.OCP_DEG = 0;
  drv_mot1.registers.ctrl4.bits.OCP_LVL = 0;
  drv_mot1.registers.ctrl5.bits.CSA_GAIN = 0;
  drv_mot1.registers.ctrl5.bits.EN_AAR = 0;
  drv_mot1.registers.ctrl5.bits.EN_ASR = 0;
  drv_mot1.registers.ctrl10.bits.DLY_TARGET = 0x5;
  drv_mot1.registers.ctrl10.bits.DLYCMP_EN = 1;
  ret = h_drv8316_write_all_registers(&drv_mot1);
  ret = h_drv8316_read_all_registers(&drv_mot1);


  ret = h_drv8316_read_all_registers(&drv_mot2);
  drv_mot2.registers.ctrl2.bits.SLEW = 3;
  drv_mot2.registers.ctrl2.bits.PWM_MODE = 0;
  drv_mot2.registers.ctrl4.bits.OCP_MODE = 0;
  drv_mot2.registers.ctrl4.bits.OCP_DEG = 2;
  drv_mot2.registers.ctrl5.bits.CSA_GAIN = 0;
  drv_mot2.registers.ctrl5.bits.EN_AAR = 0;
  drv_mot2.registers.ctrl5.bits.EN_ASR = 0;
  drv_mot2.registers.ctrl10.bits.DLY_TARGET = 5;
  drv_mot2.registers.ctrl10.bits.DLYCMP_EN = 1;
  ret = h_drv8316_write_all_registers(&drv_mot2);
  ret = h_drv8316_read_all_registers(&drv_mot2);

  motor_fsp_init();

} /* End of function mtr_init() */



/***********************************************************************************************************************
* Function Name : mtr_main
* Description   : Main routine for Motor Control
* Arguments     : None
* Return Value  : None
***********************************************************************************************************************/
void mtr_main(void)
{

    c_timespan_t ts;
    c_timespan_init(&ts);
    h_time_update(&ts);
    impl_time_start_adc();
    while(1)
    {
    	remotectrl_process();
        board_ui();

        bool_t res;
        h_time_is_elapsed_ms(&ts,1000, &res);
        if(res == TRUE)
        {
            h_time_update(&ts);
            adc_measures_update();
            //LOG_D(LOG_STD,"IIN: %5d; VIN: %5d; VBATT: %5d; VH1: %5d; VH2: %5d",adc_inst.iin,adc_inst.vin,adc_inst.vbatt,adc_inst.vhall1,adc_inst.vhall2);
            int32_t pulsesH=0;
            int32_t pulsesL=0;

            motors_instance.motorH->motor_ctrl_instance->p_api->pulsesGet( motors_instance.motorH->motor_ctrl_instance->p_ctrl,&pulsesH);
            motors_instance.motorL->motor_ctrl_instance->p_api->pulsesGet( motors_instance.motorL->motor_ctrl_instance->p_ctrl,&pulsesL);
            LOG_D(LOG_STD,"pulsesH: %05d - pulsesL: %05d",pulsesH,pulsesL);


        }
    }

} /* End of function mtr_main() */


static void board_ui(void)
{

	drive_process();
    remotectrl_process();
    /*
	volatile return_t ret;
    static uint8_t mot0_error = 0;
    static uint8_t mot1_error = 0;

    static uint8_t state0 = 0;
    motor_ext_settings_t settings;
    settings.current_max = 0.0f;
    settings.timeout_hall_ms = 0;
    uint8_t u1_temp_sw_signal;
    motor_120_control_wait_stop_flag_t u1_temp_flg_wait_stop = MOTOR_120_CONTROL_WAIT_STOP_FLAG_SET;


    g_mot_120_degree0.p_api->statusGet(g_mot_120_degree0.p_ctrl, &g_u1_motor0_status);
    switch (g_u1_motor0_status)
    {
        case MOTOR_120_DEGREE_CTRL_STATUS_STOP:
        {

        	ret = motor_wait_stop(&motor0);

            while (MOTOR_120_CONTROL_WAIT_STOP_FLAG_SET == u1_temp_flg_wait_stop)
            {

                g_mot_120_degree0.p_api->waitStopFlagGet(g_mot_120_degree0.p_ctrl, &u1_temp_flg_wait_stop);
            }
            R_IOPORT_PinWrite(&g_ioport_ctrl, LED1,BSP_IO_LEVEL_LOW );
            LOG_I(LOG_STD,"mot0 RUN");
            mot0_error = 0;
            g_mot_120_degree0.p_api->run(g_mot_120_degree0.p_ctrl);
            settings.percent = -50;
            g_mot_120_degree0.p_api->speedSetOpenLoop (g_mot_120_degree0.p_ctrl, settings);
        }
        break;

        case MOTOR_120_DEGREE_CTRL_STATUS_RUN:
        {
           bool_t res;
           //
           motor_is_speed_achieved(&motor0,&res);
           if(res == TRUE)
           {
        	   R_IOPORT_PinWrite(&g_ioport_ctrl, LED1,BSP_IO_LEVEL_HIGH );
           }
           else
           {
        	   R_IOPORT_PinWrite(&g_ioport_ctrl, LED1,BSP_IO_LEVEL_LOW );
           }
        }
        break;

        case MOTOR_120_DEGREE_CTRL_STATUS_ERROR:
        {
            if(mot0_error == 0)
            {
            	R_IOPORT_PinWrite(&g_ioport_ctrl, LED1,BSP_IO_LEVEL_LOW );
                mot0_error = 1;
                g_mot_120_degree0.p_api->errorCheck(g_mot_120_degree0.p_ctrl, &g_u2_chk_error0);
                LOG_E(LOG_STD,"mot0 ERROR: %d",g_u2_chk_error0);
            }


        }
        break;

        default:
        {

        }
        break;
    }
    g_f4_speed_ref = 800.0;
    g_u1_stop_req = MTR_FLG_CLR;
    //g_mot_120_degree0.p_api->speedSet(g_mot_120_degree0.p_ctrl, (float)g_f4_speed_ref);




    bool_t result = FALSE;
    if (state0 == 0)
    {
        h_time_is_elapsed_ms (&ts0, 5000, &result);
        if (result == TRUE)
        {
            settings.percent = -50;
            g_mot_120_degree0.p_api->speedSetOpenLoop (g_mot_120_degree0.p_ctrl, settings);
            h_time_update (&ts0);
            state0 = 1;
        }
    }
    else if (state0 == 1)
    {
        h_time_is_elapsed_ms (&ts0, 5000, &result);
        if (result == TRUE)
        {
            settings.percent = -70;
            g_mot_120_degree0.p_api->speedSetOpenLoop (g_mot_120_degree0.p_ctrl, settings);
            h_time_update (&ts0);
            state0 = 2;
        }
    }
    else if (state0 == 2)
    {
        h_time_is_elapsed_ms (&ts0, 5000, &result);
        if (result == TRUE)
        {
            //settings.percent = -35;
            //g_mot_120_degree0.p_api->speedSetOpenLoop (g_mot_120_degree0.p_ctrl, settings);
            g_f4_speed_ref = -1500;
            g_mot_120_degree0.p_api->speedSet (g_mot_120_degree0.p_ctrl, (float) g_f4_speed_ref);
            h_time_update (&ts0);
            state0 = 3;
        }
    }
    else if (state0 == 3)
    {
        h_time_is_elapsed_ms (&ts0, 5000, &result);
        if (result == TRUE)
        {
            //settings.percent = -35;
            //g_mot_120_degree0.p_api->speedSetOpenLoop (g_mot_120_degree0.p_ctrl, settings);
            g_f4_speed_ref = -3000;
            g_mot_120_degree0.p_api->speedSet (g_mot_120_degree0.p_ctrl, (float) g_f4_speed_ref);
            h_time_update (&ts0);
            state0 = 0;
        }
    }




    u1_temp_flg_wait_stop = MOTOR_120_CONTROL_WAIT_STOP_FLAG_SET;
    g_mot_120_degree1.p_api->statusGet(g_mot_120_degree1.p_ctrl, &g_u1_motor1_status);
    switch (g_u1_motor1_status)
    {
        case MOTOR_120_DEGREE_CTRL_STATUS_STOP:
        {
            while (MOTOR_120_CONTROL_WAIT_STOP_FLAG_SET == u1_temp_flg_wait_stop)
            {

                g_mot_120_degree1.p_api->waitStopFlagGet(g_mot_120_degree1.p_ctrl, &u1_temp_flg_wait_stop);
            }
            mot1_error = 0;
            //g_mot_120_degree1.p_api->run(g_mot_120_degree1.p_ctrl);
        }
        break;

        case MOTOR_120_DEGREE_CTRL_STATUS_RUN:
        {

        }
        break;

        case MOTOR_120_DEGREE_CTRL_STATUS_ERROR:
        {
            if(mot1_error == 0)
            {
                mot1_error = 1;
                g_mot_120_degree1.p_api->errorCheck(g_mot_120_degree1.p_ctrl, &g_u2_chk_error1);
                LOG_E(LOG_STD,"mot1 ERROR: %d",g_u2_chk_error1);
            }

        }
        break;

        default:
        {

        }
        break;
    }
    g_f4_speed_ref = 800.0;
    g_u1_stop_req = MTR_FLG_CLR;
    //g_mot_120_degree1.p_api->speedSet(g_mot_120_degree1.p_ctrl, (float)g_f4_speed_ref);


    settings.current_max = 0.0f;
    settings.timeout_hall_ms = 0;
    settings.percent = 20;
    g_mot_120_degree1.p_api->speedSetOpenLoop(g_mot_120_degree1.p_ctrl,settings);
    */
}


/***********************************************************************************************************************
* Function Name : software_init
* Description   : Initialize private global variables
* Arguments     : None
* Return Value  : None
***********************************************************************************************************************/

/***********************************************************************************************************************
* Function Name : g_poe_overcurrent
* Description   : POEG2 Interrupt callback function
* Arguments     : p_args - Callback argument
* Return Value  : None
***********************************************************************************************************************/
void g_poe_overcurrent(poeg_callback_args_t *p_args)
{
    if (NULL != p_args)
    {
        R_POEG_Reset(g_poeg0.p_ctrl);
#ifdef ALT_MOT
        g_mot_120_degree0.p_api->errorSet(g_mot_120_degree0.p_ctrl, MOTOR_ERROR_OVER_CURRENT_HW);
#else
        g_motor_120_degree0.p_api->errorSet(g_motor_120_degree0.p_ctrl, MOTOR_ERROR_OVER_CURRENT_HW);
#endif
        g_u2_chk_error0 |= MOTOR_ERROR_OVER_CURRENT_HW;
    }
} /* End of function g_poe_overcurrent */















/***********************************************************************************************************************
* Function Name : motor_fsp_init
* Description   : Initialize Motor FSP modules
* Arguments     : None
* Return Value  : None
***********************************************************************************************************************/
static void motor_fsp_init(void)
{

    c_timespan_init(&ts0);
    c_timespan_init(&ts1);
    h_time_update(&ts0);
    h_time_update(&ts1);
    R_POEG_Open(g_poeg0.p_ctrl, g_poeg0.p_cfg);
    motor_structures_init();
    motor_init_type(MOTOR_TYPE_RM_ITOH_BRAKE);
    adc_init();
    motor_init_fsp();

    motors_instance.motorH->motor_ctrl_instance->p_api->pulsesSet( motors_instance.motorH->motor_ctrl_instance->p_ctrl,0);
    motors_instance.motorL->motor_ctrl_instance->p_api->pulsesSet( motors_instance.motorL->motor_ctrl_instance->p_ctrl,0);
}



/***********************************************************************************************************************
* Function Name : mtr_remove_sw_chattering
* Description   : Get switch status with removing chattering
* Arguments     : u1_sw - Board interface switch signal
*                 u1_on_off - Detected status (ON/OFF)
* Return Value  : u1_remove_chattering_flg - Detection result
***********************************************************************************************************************/
static uint8_t mtr_remove_sw_chattering(uint8_t u1_sw, uint8_t u1_on_off)
{
    uint8_t u1_remove_chattering_flg;

    u1_remove_chattering_flg = 0;
    if (u1_on_off == u1_sw)
    {
        g_u1_sw_cnt++;
        if (CHATTERING_CNT < g_u1_sw_cnt)
        {
            u1_remove_chattering_flg = MTR_FLG_SET;
            g_u1_sw_cnt = 0;
        }
    }
    else
    {
        g_u1_sw_cnt = 0;
    }

    return (u1_remove_chattering_flg);
} /* End of function mtr_remove_sw_chattering */




