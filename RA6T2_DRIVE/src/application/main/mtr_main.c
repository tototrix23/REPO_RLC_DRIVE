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
#include "hal_data.h"
#include "rm_motor_api.h"

#include <hal_data.h>
#include <_core/c_common.h>
#include <_interfaces/i_spi/i_spi.h>
#include <_hal/h_drv8316/h_drv8316.h>
#include <_lib_impl_cust/impl_spi_motors/impl_spi_motors.h>
#include <adc.h>

#define ALT_MOT



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


extern uint8_t  com_u1_enable_write;           /* ICS write enable flag */


volatile h_drv8316_t drv_mot1;
volatile h_drv8316_t drv_mot2;

volatile i_spi_t interface_mot1;
volatile i_spi_t interface_mot2;


/***********************************************************************************************************************
* Private functions
***********************************************************************************************************************/
static void     motor_fsp_init (void);

static uint8_t  mtr_remove_sw_chattering (uint8_t u1_sw, uint8_t u1_on_off);
static void     board_ui (void);           /* Board user interface */

static void     software_init (void);      /* Software initialize */
static void gpt_periodset (timer_ctrl_t * const p_ctrl, uint32_t const period_counts, uint32_t const value);

void custom_adc_interrupt(adc_callback_args_t * p_args);


volatile motor_instance_t g_motor_120_degree0_ext;
volatile motor_instance_t g_motor_120_degree1_ext;
volatile motor_120_driver_cfg_t conf0;
volatile motor_120_driver_extended_cfg_t ext0;
volatile motor_120_driver_cfg_t conf1;
volatile motor_120_driver_extended_cfg_t ext1;

timer_instance_t mot0_g_timer0;
timer_cfg_t mot0_g_timer0_cfg;
timer_instance_t mot0_g_timer1;
timer_cfg_t mot0_g_timer1_cfg;
motor_120_driver_extended_cfg_t g_mot_120_driver0_extend;
motor_120_driver_cfg_t g_mot_120_driver0_cfg;
motor_120_driver_instance_ctrl_t g_mot_120_driver0_ctrl;
motor_120_driver_instance_t g_mot_120_driver0;
motor_120_control_hall_instance_ctrl_t g_mot_120_control_hall0_ctrl;
extern const motor_120_control_hall_extended_cfg_t g_motor_120_control_hall0_extend;
motor_120_control_hall_extended_cfg_t g_mot_120_control_hall0_extend;
motor_120_control_cfg_t g_mot_120_control_hall0_cfg;
motor_120_control_instance_t g_mot_120_control_hall0;
motor_120_degree_instance_ctrl_t g_mot_120_degree0_ctrl;
extern const motor_120_degree_extended_cfg_t g_motor_120_degree0_extend;
motor_120_degree_extended_cfg_t g_mot_120_degree0_extend;
motor_cfg_t g_mot_120_degree0_cfg;
motor_instance_t g_mot_120_degree0;

timer_instance_t mot1_g_timer0;
timer_cfg_t mot1_g_timer0_cfg;
timer_instance_t mot1_g_timer1;
timer_cfg_t mot1_g_timer1_cfg;
motor_120_driver_extended_cfg_t g_mot_120_driver1_extend;
motor_120_driver_cfg_t g_mot_120_driver1_cfg;
motor_120_driver_instance_ctrl_t g_mot_120_driver1_ctrl;
motor_120_driver_instance_t g_mot_120_driver1;
motor_120_control_hall_instance_ctrl_t g_mot_120_control_hall1_ctrl;
extern const motor_120_control_hall_extended_cfg_t g_motor_120_control_hall1_extend;
motor_120_control_hall_extended_cfg_t g_mot_120_control_hall1_extend;
motor_120_control_cfg_t g_mot_120_control_hall1_cfg;
motor_120_control_instance_t g_mot_120_control_hall1;
motor_120_degree_instance_ctrl_t g_mot_120_degree1_ctrl;
extern const motor_120_degree_extended_cfg_t g_motor_120_degree1_extend;
motor_120_degree_extended_cfg_t g_mot_120_degree1_extend;
motor_cfg_t g_mot_120_degree1_cfg;
motor_instance_t g_mot_120_degree1;
void motor_structures_init(void);


/***********************************************************************************************************************
* Function Name : mtr_init
* Description   : Initialization for Motor Control
* Arguments     : None
* Return Value  : None
***********************************************************************************************************************/
void mtr_init(void)
{
   volatile int i=0;

   R_BSP_SoftwareDelay(500, BSP_DELAY_UNITS_MILLISECONDS);
   R_IOPORT_PinWrite(&g_ioport_ctrl, VM_CMD,BSP_IO_LEVEL_HIGH );
   R_BSP_SoftwareDelay(200, BSP_DELAY_UNITS_MILLISECONDS);






   i_spi_init(&interface_mot1, spi_motor_open, spi_motor_close, spi_motor_read, spi_motor_write, spi_motor_mot1_cs_inactive, spi_motor_mot1_cs_active);
   i_spi_init(&interface_mot2, spi_motor_open, spi_motor_close, spi_motor_read, spi_motor_write, spi_motor_mot2_cs_inactive, spi_motor_mot2_cs_active);

  volatile return_t ret = h_drv8316_init(&drv_mot1,&interface_mot1,FALSE);
  ret = h_drv8316_init(&drv_mot2,&interface_mot2,FALSE);

  ret = h_drv8316_read_all_registers(&drv_mot1);
  drv_mot1.registers.ctrl2.bits.SLEW = 2;
  drv_mot1.registers.ctrl2.bits.PWM_MODE = 0;
  drv_mot1.registers.ctrl4.bits.OCP_MODE = 0;
  drv_mot1.registers.ctrl4.bits.OCP_DEG = 0;
  drv_mot1.registers.ctrl4.bits.OCP_LVL = 0;
  drv_mot1.registers.ctrl5.bits.CSA_GAIN = 0;
  drv_mot1.registers.ctrl5.bits.EN_AAR = 0;
  drv_mot1.registers.ctrl5.bits.EN_ASR = 0;
  drv_mot1.registers.ctrl10.bits.DLY_TARGET = 0x8;
  drv_mot1.registers.ctrl10.bits.DLYCMP_EN = 1;
  ret = h_drv8316_write_all_registers(&drv_mot1);
  ret = h_drv8316_read_all_registers(&drv_mot1);


  ret = h_drv8316_read_all_registers(&drv_mot2);
  drv_mot2.registers.ctrl2.bits.SLEW = 2;
  drv_mot2.registers.ctrl2.bits.PWM_MODE = 0;
  drv_mot2.registers.ctrl4.bits.OCP_MODE = 0;
  drv_mot2.registers.ctrl4.bits.OCP_DEG = 2;
  drv_mot2.registers.ctrl5.bits.CSA_GAIN = 0;
  drv_mot2.registers.ctrl5.bits.EN_AAR = 0;
  drv_mot2.registers.ctrl5.bits.EN_ASR = 0;
  drv_mot2.registers.ctrl10.bits.DLY_TARGET = 8;
  drv_mot2.registers.ctrl10.bits.DLYCMP_EN = 1;
  ret = h_drv8316_write_all_registers(&drv_mot2);
  ret = h_drv8316_read_all_registers(&drv_mot2);
  volatile uint8_t x=0;


    //int i;
    uint8_t u1_conf_motor_type[] = CONF_MOTOR_TYPE;
    uint8_t u1_conf_control[] = CONF_CONTROL;
    uint8_t u1_conf_inverter[] = CONF_INVERTER;
    g_u1_conf_motor_type_len = CONF_MOTOR_TYPE_LEN;
    g_u1_conf_control_len    = CONF_CONTROL_LEN;
    g_u1_conf_inverter_len   = CONF_INVERTER_LEN;
    for (i = 0; i < g_u1_conf_motor_type_len; i++)
    {
        g_u1_conf_motor_type[i] = u1_conf_motor_type[i];
    }
    for (i = 0; i < g_u1_conf_control_len; i++)
    {
        g_u1_conf_control[i] = u1_conf_control[i];
    }
    for (i = 0; i < g_u1_conf_inverter_len; i++)
    {
        g_u1_conf_inverter[i] = u1_conf_inverter[i];
    }
    g_u2_conf_hw = 0x0008;                        /* 0000000000001000b */
    g_u2_conf_sw = 0x0044;                        /* 0000000001000100b */
    g_u2_conf_tool = 0x0200;                      /* 0000001000000000b */

    motor_fsp_init();
    software_init();                              /* Initialize private global variables */


} /* End of function mtr_init() */



/***********************************************************************************************************************
* Function Name : mtr_main
* Description   : Main routine for Motor Control
* Arguments     : None
* Return Value  : None
***********************************************************************************************************************/
void mtr_main(void)
{
    board_ui();
} /* End of function mtr_main() */

#ifdef ALT_MOT
static void board_ui(void)
{
    uint8_t u1_temp_sw_signal;
    motor_120_control_wait_stop_flag_t u1_temp_flg_wait_stop = MOTOR_120_CONTROL_WAIT_STOP_FLAG_SET;


    g_mot_120_degree0.p_api->statusGet(g_mot_120_degree0.p_ctrl, &g_u1_motor0_status);
    switch (g_u1_motor0_status)
    {
        case MOTOR_120_DEGREE_CTRL_STATUS_STOP:
        {
            while (MOTOR_120_CONTROL_WAIT_STOP_FLAG_SET == u1_temp_flg_wait_stop)
            {

                g_mot_120_degree0.p_api->waitStopFlagGet(g_mot_120_degree0.p_ctrl, &u1_temp_flg_wait_stop);
            }
            g_mot_120_degree0.p_api->run(g_mot_120_degree0.p_ctrl);
        }
        break;

        case MOTOR_120_DEGREE_CTRL_STATUS_RUN:
        {

        }
        break;

        case MOTOR_120_DEGREE_CTRL_STATUS_ERROR:
        {
            //R_IOPORT_PinWrite(&g_ioport_ctrl, VM_CMD,BSP_IO_LEVEL_LOW );
            //volatile return_t ret = h_drv8316_read_all_registers(&drv_mot1);
            g_mot_120_degree0.p_api->errorCheck(g_mot_120_degree0.p_ctrl, &g_u2_chk_error0);


                volatile uint8_t i=0;

        }
        break;

        default:
        {

        }
        break;
    }
    g_f4_speed_ref = -3000.0;
    g_u1_stop_req = MTR_FLG_CLR;
    g_mot_120_degree0.p_api->speedSet(g_mot_120_degree0.p_ctrl, (float)g_f4_speed_ref);



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
            g_mot_120_degree1.p_api->run(g_mot_120_degree1.p_ctrl);
        }
        break;

        case MOTOR_120_DEGREE_CTRL_STATUS_RUN:
        {

        }
        break;

        case MOTOR_120_DEGREE_CTRL_STATUS_ERROR:
        {
            //R_IOPORT_PinWrite(&g_ioport_ctrl, VM_CMD,BSP_IO_LEVEL_LOW );
            //volatile return_t ret = h_drv8316_read_all_registers(&drv_mot1);
            g_mot_120_degree1.p_api->errorCheck(g_mot_120_degree1.p_ctrl, &g_u2_chk_error1);


                volatile uint8_t i=0;

        }
        break;

        default:
        {

        }
        break;
    }
    g_f4_speed_ref = 1000.0;
    g_u1_stop_req = MTR_FLG_CLR;
    g_mot_120_degree1.p_api->speedSet(g_mot_120_degree1.p_ctrl, (float)g_f4_speed_ref);

}
#else
static void board_ui(void)
{
    uint8_t u1_temp_sw_signal;
    motor_120_control_wait_stop_flag_t u1_temp_flg_wait_stop = MOTOR_120_CONTROL_WAIT_STOP_FLAG_SET;

    /* Get status of motor control system */
    g_motor_120_degree0.p_api->statusGet(g_motor_120_degree0.p_ctrl, &g_u1_motor0_status);
    switch (g_u1_motor0_status)
    {
        case MOTOR_120_DEGREE_CTRL_STATUS_STOP:
        {
            while (MOTOR_120_CONTROL_WAIT_STOP_FLAG_SET == u1_temp_flg_wait_stop)
            {
                /* waiting for motor stop */
                g_motor_120_degree0.p_api->waitStopFlagGet(g_motor_120_degree0.p_ctrl, &u1_temp_flg_wait_stop);
            }
            g_motor_120_degree0.p_api->run(g_motor_120_degree0.p_ctrl);
        }
        break;

        case MOTOR_120_DEGREE_CTRL_STATUS_RUN:
        {
            /*u1_temp_sw_signal = get_sw1();

            if ((MTR_FLG_SET == mtr_remove_sw_chattering(u1_temp_sw_signal, SW1_OFF)) || (MTR_FLG_CLR != g_u1_stop_req))
            {
                g_motor_120_degree0.p_api->stop(g_motor_120_degree0.p_ctrl);
            }*/
        }
        break;

        case MOTOR_120_DEGREE_CTRL_STATUS_ERROR:
        {
            R_IOPORT_PinWrite(&g_ioport_ctrl, VM_CMD,BSP_IO_LEVEL_LOW );
            //volatile return_t ret = h_drv8316_read_all_registers(&drv_mot1);
            g_motor_120_degree0.p_api->errorCheck(g_motor_120_degree0.p_ctrl, &g_u2_chk_error0);
            while(1)
            {

                volatile uint8_t i=0;
            }
        }
        break;

        default:
        {
            /* Do nothing */
        }
        break;
    }
    g_f4_speed_ref = 1000.0;
    g_u1_stop_req = MTR_FLG_CLR;
    g_motor_120_degree0.p_api->speedSet(g_motor_120_degree0.p_ctrl, (float)g_f4_speed_ref);
}
#endif

/***********************************************************************************************************************
* Function Name : software_init
* Description   : Initialize private global variables
* Arguments     : None
* Return Value  : None
***********************************************************************************************************************/
static void software_init(void)
{
    g_u1_motor0_status            = MOTOR_120_DEGREE_CTRL_STATUS_STOP;
    g_f4_max_speed_rpm           = MTR_MAX_SPEED_RPM;
    g_u1_sw_userif               = CONFIG_DEFAULT_UI;
    g_u1_mode_system             = MOTOR_120_DEGREE_CTRL_EVENT_STOP;
    g_u1_reset_req               = SW_OFF;
    g_u1_stop_req                = MTR_FLG_SET;

    /* ICS variables initialization */
    com_u1_sw_userif             = CONFIG_DEFAULT_UI;
    com_u1_mode_system           = MOTOR_120_DEGREE_CTRL_EVENT_STOP;
} /* End of function software_init */

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



void generic_timer_init(void)
{
    R_GPT_Open(g_timer_generic.p_ctrl,&g_timer_generic_cfg);
    R_GPT_Start(g_timer_generic.p_ctrl);
}

void timer_generic_callback (timer_callback_args_t * p_args)
{

    custom_adc_capture();
}

void custom_adc_init(void)
{
    volatile fsp_err_t err = FSP_SUCCESS;
    //err = R_ADC_B_Close(&g_adc_external_ctrl);
    err = R_ADC_B_Open(&g_adc_external_ctrl, &g_adc_external_cfg);
    err = R_ADC_B_ScanCfg(&g_adc_external_ctrl, &g_adc_external_scan_cfg);
    err = R_ADC_B_Calibrate(&g_adc_external_ctrl, NULL);
    R_BSP_SoftwareDelay(10, BSP_DELAY_UNITS_MILLISECONDS);
    err= R_ADC_B_ScanStart(&g_adc_external_ctrl);
}

void custom_adc_capture(void)
{
    volatile fsp_err_t err = R_ADC_B_ScanGroupStart(&g_adc_external_ctrl,ADC_GROUP_MASK_8);
    volatile uint8_t i=0;
}

void custom_adc_interrupt(adc_callback_args_t *p_args)
{


    if (p_args->event == ADC_EVENT_SCAN_COMPLETE)
    {
        if (p_args->group_mask == ADC_GROUP_MASK_0)
        {
#ifdef ALT_MOT
            p_args->p_context = &g_mot_120_driver0;
#else
            p_args->p_context = &g_motor_120_driver0;
#endif
            rm_motor_120_driver_cyclic(p_args);
        }
        if (p_args->group_mask == ADC_GROUP_MASK_2)
        {
#ifdef ALT_MOT
            p_args->p_context = &g_mot_120_driver1;
#else
            p_args->p_context = &g_motor_120_driver1;
#endif
            rm_motor_120_driver_cyclic(p_args);
        }
        if (p_args->group_mask == ADC_GROUP_MASK_8)
        {
            volatile uint8_t i=0;
            volatile uint16_t data[2] = {0};
            R_ADC_B_Read(&g_adc_external_ctrl, 14,&data[0]);
            R_ADC_B_Read(&g_adc_external_ctrl, 15,&data[1]);
            i=0;

        }
    }
    else if(p_args->event == ADC_EVENT_CALIBRATION_COMPLETE || p_args->event == ADC_EVENT_CALIBRATION_REQUEST)
    {
        return;
    }
    else
    {
        volatile uint8_t i=0;
    }
}




void motor_structures_init(void)
{

    //---------------------------------------------------------------------
    // MOTOR0
    //---------------------------------------------------------------------
    memcpy(&mot0_g_timer0_cfg,&g_timer3_cfg,sizeof(timer_cfg_t));
    mot0_g_timer0_cfg.p_context = &g_mot_120_control_hall0;
    memcpy(&mot0_g_timer0,&g_timer3,sizeof(timer_instance_t));
    mot0_g_timer0.p_cfg  =&mot0_g_timer0_cfg;

    memcpy(&mot0_g_timer1_cfg,&g_timer4_cfg,sizeof(timer_cfg_t));
    mot0_g_timer1_cfg.p_context = &g_mot_120_control_hall0;
    memcpy(&mot0_g_timer1,&g_timer4,sizeof(timer_instance_t));
    mot0_g_timer1.p_cfg  =&mot0_g_timer1_cfg;

    // DRIVER
    memcpy(&g_mot_120_driver0_extend,&g_motor_120_driver0_extend,sizeof(motor_120_driver_extended_cfg_t));
    g_mot_120_driver0_extend.p_adc_instance = &g_adc_external;

    memcpy(&g_mot_120_driver0_cfg,&g_motor_120_driver0_cfg,sizeof(motor_120_driver_cfg_t));
    g_mot_120_driver0_cfg.p_extend = &g_mot_120_driver0_extend;
    g_mot_120_driver0_cfg.p_context = &g_mot_120_control_hall0;
    g_mot_120_driver0.p_ctrl = &g_mot_120_driver0_ctrl;
    g_mot_120_driver0.p_cfg = &g_mot_120_driver0_cfg;
    g_mot_120_driver0.p_api = &g_motor_120_driver_on_motor_120_driver;

    //HALL
    memcpy(&g_mot_120_control_hall0_extend,&g_motor_120_control_hall0_extend,sizeof(motor_120_control_hall_extended_cfg_t));
    g_mot_120_control_hall0_extend.p_motor_120_driver_instance = &g_mot_120_driver0;
    g_mot_120_control_hall0_extend.p_speed_cyclic_timer_instance = &mot0_g_timer0;
    g_mot_120_control_hall0_extend.p_speed_calc_timer_instance = &mot0_g_timer1;
    memcpy(&g_mot_120_control_hall0_cfg,&g_motor_120_control_hall0_cfg,sizeof(motor_120_control_cfg_t));

    g_mot_120_control_hall0_cfg.p_context = &g_mot_120_degree0;
    g_mot_120_control_hall0_cfg.p_extend = &g_mot_120_control_hall0_extend;

    memcpy(&g_mot_120_control_hall0,&g_motor_120_control_hall0,sizeof(motor_120_control_instance_t));
    g_mot_120_control_hall0.p_ctrl = &g_mot_120_control_hall0_ctrl;
    g_mot_120_control_hall0.p_cfg = &g_mot_120_control_hall0_cfg;
    g_mot_120_control_hall0.p_api = &g_motor_120_control_on_motor_120_control_hall;

    //MOTOR
    memcpy(&g_mot_120_degree0_extend,&g_motor_120_degree0_extend,sizeof(motor_120_degree_extended_cfg_t));
    g_mot_120_degree0_extend.p_motor_120_control_instance = &g_mot_120_control_hall0;

    memcpy(&g_mot_120_degree0_cfg,&g_motor_120_degree0_cfg,sizeof(motor_cfg_t));
    g_mot_120_degree0_cfg.p_extend = &g_mot_120_degree0_extend;

    g_mot_120_degree0.p_ctrl = &g_mot_120_degree0_ctrl;
    g_mot_120_degree0.p_cfg = &g_mot_120_degree0_cfg;
    g_mot_120_degree0.p_api =  &g_motor_on_motor_120_degree;

    //---------------------------------------------------------------------
    // MOTOR1
    //---------------------------------------------------------------------
    memcpy(&mot1_g_timer0_cfg,&g_timer8_cfg,sizeof(timer_cfg_t));
    mot1_g_timer0_cfg.p_context = &g_mot_120_control_hall1;
    memcpy(&mot1_g_timer0,&g_timer8,sizeof(timer_instance_t));
    mot1_g_timer0.p_cfg  =&mot1_g_timer0_cfg;

    memcpy(&mot1_g_timer1_cfg,&g_timer9_cfg,sizeof(timer_cfg_t));
    mot1_g_timer1_cfg.p_context = &g_mot_120_control_hall1;
    memcpy(&mot1_g_timer1,&g_timer9,sizeof(timer_instance_t));
    mot1_g_timer1.p_cfg  =&mot1_g_timer1_cfg;
    // DRIVER
    memcpy(&g_mot_120_driver1_extend,&g_motor_120_driver1_extend,sizeof(motor_120_driver_extended_cfg_t));
    g_mot_120_driver1_extend.p_adc_instance = &g_adc_external;

    memcpy(&g_mot_120_driver1_cfg,&g_motor_120_driver1_cfg,sizeof(motor_120_driver_cfg_t));
    g_mot_120_driver1_cfg.p_extend = &g_mot_120_driver1_extend;
    g_mot_120_driver1_cfg.p_context = &g_mot_120_control_hall1;

    g_mot_120_driver1.p_ctrl = &g_mot_120_driver1_ctrl;
    g_mot_120_driver1.p_cfg = &g_mot_120_driver1_cfg;
    g_mot_120_driver1.p_api = &g_motor_120_driver_on_motor_120_driver;

    //HALL
    memcpy(&g_mot_120_control_hall1_extend,&g_motor_120_control_hall1_extend,sizeof(motor_120_control_hall_extended_cfg_t));
    g_mot_120_control_hall1_extend.p_motor_120_driver_instance = &g_mot_120_driver1;
    g_mot_120_control_hall1_extend.p_speed_cyclic_timer_instance = &mot1_g_timer0;
    g_mot_120_control_hall1_extend.p_speed_calc_timer_instance = &mot1_g_timer1;
    memcpy(&g_mot_120_control_hall1_cfg,&g_motor_120_control_hall1_cfg,sizeof(motor_120_control_cfg_t));
    g_mot_120_control_hall1_cfg.p_context = &g_mot_120_degree1;
    g_mot_120_control_hall1_cfg.p_extend = &g_mot_120_control_hall1_extend;

    memcpy(&g_mot_120_control_hall1,&g_motor_120_control_hall1,sizeof(motor_120_control_instance_t));
    g_mot_120_control_hall1.p_ctrl = &g_mot_120_control_hall1_ctrl;
    g_mot_120_control_hall1.p_cfg = &g_mot_120_control_hall1_cfg;
    g_mot_120_control_hall1.p_api = &g_motor_120_control_on_motor_120_control_hall;

    //MOTOR
    memcpy(&g_mot_120_degree1_extend,&g_motor_120_degree1_extend,sizeof(motor_120_degree_extended_cfg_t));
    g_mot_120_degree1_extend.p_motor_120_control_instance = &g_mot_120_control_hall1;

    memcpy(&g_mot_120_degree1_cfg,&g_motor_120_degree1_cfg,sizeof(motor_cfg_t));
    g_mot_120_degree1_cfg.p_extend = &g_mot_120_degree1_extend;

    g_mot_120_degree1.p_ctrl = &g_mot_120_degree1_ctrl;
    g_mot_120_degree1.p_cfg = &g_mot_120_degree1_cfg;
    g_mot_120_degree1.p_api =  &g_motor_on_motor_120_degree;



}

/***********************************************************************************************************************
* Function Name : motor_fsp_init
* Description   : Initialize Motor FSP modules
* Arguments     : None
* Return Value  : None
***********************************************************************************************************************/
static void motor_fsp_init(void)
{

    motor_structures_init();


    R_POEG_Open(g_poeg0.p_ctrl, g_poeg0.p_cfg);

    /* ADC, GPT Three Phase */
    custom_adc_init();
#ifdef ALT_MOT
    g_mot_120_degree0.p_api->open(g_mot_120_degree0.p_ctrl, g_mot_120_degree0.p_cfg);
    g_mot_120_degree0.p_api->statusGet(g_mot_120_degree0.p_ctrl, &g_u1_motor0_status);
    g_mot_120_degree1.p_api->open(g_mot_120_degree1.p_ctrl, g_mot_120_degree1.p_cfg);
    //g_mot_120_degree1.p_api->statusGet(g_mot_120_degree1.p_ctrl, &g_u1_motor1_status);
#else
    g_motor_120_degree0.p_api->open(g_motor_120_degree0.p_ctrl, g_motor_120_degree0.p_cfg);
    g_motor_120_degree0.p_api->statusGet(g_motor_120_degree0.p_ctrl, &g_u1_motor0_status);
    g_motor_120_degree1.p_api->open(g_motor_120_degree1.p_ctrl, g_motor_120_degree1.p_cfg);
    //g_motor_120_degree1.p_api->statusGet(g_motor_120_degree1.p_ctrl, &g_u1_motor0_status);
#endif

    //custom_adc_init();

    generic_timer_init();



    R_GPT_THREE_PHASE_Stop(g_three_phase0.p_ctrl);
    R_GPT_THREE_PHASE_Stop(g_three_phase1.p_ctrl);

    R_GPT_THREE_PHASE_Reset(g_three_phase0.p_ctrl);
    R_GPT_THREE_PHASE_Reset(g_three_phase1.p_ctrl);
    gpt_periodset(g_timer0.p_ctrl,g_timer0.p_cfg->period_counts,(uint32_t)(g_timer0.p_cfg->period_counts));
    gpt_periodset(g_timer1.p_ctrl,g_timer1.p_cfg->period_counts,(uint32_t)(g_timer1.p_cfg->period_counts));
    gpt_periodset(g_timer2.p_ctrl,g_timer2.p_cfg->period_counts,(uint32_t)(g_timer2.p_cfg->period_counts));
    gpt_periodset(g_timer5.p_ctrl,g_timer5.p_cfg->period_counts,(uint32_t)((float)g_timer5.p_cfg->period_counts*1.5f));
    gpt_periodset(g_timer6.p_ctrl,g_timer6.p_cfg->period_counts,(uint32_t)((float)g_timer6.p_cfg->period_counts*1.5f));
    gpt_periodset(g_timer7.p_ctrl,g_timer7.p_cfg->period_counts,(uint32_t)((float)g_timer7.p_cfg->period_counts*1.5f));
    R_GPT_THREE_PHASE_Start(g_three_phase0.p_ctrl);
    R_GPT_THREE_PHASE_Start(g_three_phase1.p_ctrl);
    g_mot_120_degree0.p_api->reset(g_mot_120_degree0.p_ctrl);
    g_mot_120_degree1.p_api->reset(g_mot_120_degree1.p_ctrl);
    R_BSP_SoftwareDelay(200, BSP_DELAY_UNITS_MILLISECONDS);
    volatile uint8_t x=0;



    /* RMW */
    /*g_user_motor_cfg = *(g_mot_120_degree0_ctrl.p_cfg);
    g_user_motor_120_degree_extended_cfg = *(motor_120_degree_extended_cfg_t *)g_user_motor_cfg.p_extend;
    g_user_motor_cfg.p_extend = &g_user_motor_120_degree_extended_cfg;
    g_mot_120_degree0_ctrl.p_cfg = &g_user_motor_cfg;

    g_user_motor_120_control_cfg = *(g_mot_120_control_hall0_ctrl.p_cfg);
    g_user_motor_120_control_extended_cfg =
        *(motor_120_control_hall_extended_cfg_t *)g_user_motor_120_control_cfg.p_extend;
    g_user_motor_120_control_cfg.p_extend = &g_user_motor_120_control_extended_cfg;

    g_user_motor_120_driver_cfg = *(g_mot_120_driver0_ctrl.p_cfg);
    g_user_motor_120_driver_extended_cfg = *(motor_120_driver_extended_cfg_t *)g_user_motor_120_driver_cfg.p_extend;
    g_user_motor_120_driver_cfg.p_extend = &g_user_motor_120_driver_extended_cfg;*/



/*
    g_user_motor1_cfg = *(g_motor_120_degree1_ctrl.p_cfg);
    g_user_motor1_120_degree_extended_cfg = *(motor_120_degree_extended_cfg_t *)g_user_motor1_cfg.p_extend;
    g_user_motor1_cfg.p_extend = &g_user_motor1_120_degree_extended_cfg;
    g_motor_120_degree1_ctrl.p_cfg = &g_user_motor1_cfg;

    g_user_motor1_120_control_cfg = *(g_motor_120_control_hall1_ctrl.p_cfg);
    g_user_motor1_120_control_extended_cfg =
        *(motor_120_control_hall_extended_cfg_t *)g_user_motor1_120_control_cfg.p_extend;
    g_user_motor1_120_control_cfg.p_extend = &g_user_motor1_120_control_extended_cfg;

    g_user_motor1_120_driver_cfg = *(g_motor_120_driver1_ctrl.p_cfg);
    g_user_motor1_120_driver_extended_cfg = *(motor_120_driver_extended_cfg_t *)g_user_motor1_120_driver_cfg.p_extend;
    g_user_motor1_120_driver_cfg.p_extend = &g_user_motor1_120_driver_extended_cfg;*/



} /* End of function motor_fsp_init */

/***********************************************************************************************************************
* Function Name : mtr_callback_120_degree
* Description   : Callback function of Less 120 Control
* Arguments     : p_args - Callback argument
* Return Value  : None
***********************************************************************************************************************/
void mtr0_callback_120_degree(motor_callback_args_t * p_args)
{
    switch (p_args->event)
    {
        case MOTOR_CALLBACK_EVENT_ADC_FORWARD:
        {
            /* Do nothing */
        }
        break;

        case MOTOR_CALLBACK_EVENT_ADC_BACKWARD:
        {
            if (MOTOR_120_DEGREE_CTRL_STATUS_ERROR != g_u1_motor0_status)
            {
#ifdef ALT_MOT
                g_mot_120_degree0.p_api->errorCheck(g_mot_120_degree0.p_ctrl, &g_u2_chk_error0);
#else
                g_motor_120_degree0.p_api->errorCheck(g_motor_120_degree0.p_ctrl, &g_u2_chk_error0);
#endif
            }

            //mtr_ics_interrupt_process();
        }
        break;

        case MOTOR_CALLBACK_EVENT_CYCLE_FORWARD:
        {
            /* Do nothing */
        }
        break;

        case MOTOR_CALLBACK_EVENT_CYCLE_BACKWARD:
        {
            /* Do nothing */
        }
        break;



        default:
        {
            /* Do nothing */
        }
        break;
    }
} /* End of function mtr_callback_120_degree */



void mtr1_callback_120_degree(motor_callback_args_t * p_args)
{
    switch (p_args->event)
    {
        case MOTOR_CALLBACK_EVENT_ADC_FORWARD:
        {
            /* Do nothing */
        }
        break;

        case MOTOR_CALLBACK_EVENT_ADC_BACKWARD:
        {
            if (MOTOR_120_DEGREE_CTRL_STATUS_ERROR != g_u1_motor1_status)
            {
#ifdef ALT_MOT
                g_mot_120_degree1.p_api->errorCheck(g_mot_120_degree1.p_ctrl, &g_u2_chk_error1);
#else
                g_motor_120_degree1.p_api->errorCheck(g_motor_120_degree1.p_ctrl, &g_u2_chk_error1);
#endif
            }

            //mtr_ics_interrupt_process();
        }
        break;

        case MOTOR_CALLBACK_EVENT_CYCLE_FORWARD:
        {
            /* Do nothing */
        }
        break;

        case MOTOR_CALLBACK_EVENT_CYCLE_BACKWARD:
        {
            /* Do nothing */
        }
        break;



        default:
        {
            /* Do nothing */
        }
        break;
    }
} /* End of function mtr_callback_120_degree */


/***********************************************************************************************************************
* Function Name : mtr_board_led_control
* Description   : Set LED pattern depend on motor status
* Arguments     : u1_motor_status - Motor control status
* Return Value  : None
***********************************************************************************************************************/


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


static void gpt_periodset (timer_ctrl_t * const p_ctrl, uint32_t const period_counts, uint32_t const value)
{
    gpt_instance_ctrl_t * p_instance_ctrl = (gpt_instance_ctrl_t *) p_ctrl;

    p_instance_ctrl->p_reg->GTPBR = period_counts;          /* Set period to buffer register */
    p_instance_ctrl->p_reg->GTPR = (uint32_t)(value);
}

