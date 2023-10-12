/*
 * moteur.c
 *
 *  Created on: 11 oct. 2023
 *      Author: Ch.Leclercq
 */


#include "moteur.h"


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
st_motor_t motor0;

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
st_motor_t motor1;

static void gpt_periodset (timer_ctrl_t * const p_ctrl, uint32_t const period_counts, uint32_t const value);

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

    motor0.motor_ctrl_instance = &g_mot_120_degree0;
    motor0.motor_driver_instance = &g_mot_120_driver0;
    motor0.motor_hall_instance = &g_mot_120_control_hall0;
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

    motor1.motor_ctrl_instance = &g_mot_120_degree1;
    motor1.motor_driver_instance = &g_mot_120_driver1;
    motor1.motor_hall_instance = &g_mot_120_control_hall1;

}


void motor_init_fsp(void)
{



      motor_ext_cfg_t mot_ext_cfg;
      mot_ext_cfg.motor_type = MOTOR_TYPE_BLDC;
      mot_ext_cfg.pulses_counting_reverse = 0;

      g_mot_120_degree0.p_api->open(g_mot_120_degree0.p_ctrl, g_mot_120_degree0.p_cfg);
      g_mot_120_degree0.p_api->configSet(g_mot_120_degree0.p_ctrl,mot_ext_cfg);
      g_mot_120_degree0.p_api->pulsesSet(g_mot_120_degree0.p_ctrl,0);
      g_mot_120_degree1.p_api->open(g_mot_120_degree1.p_ctrl, g_mot_120_degree1.p_cfg);
      g_mot_120_degree1.p_api->configSet(g_mot_120_degree1.p_ctrl,mot_ext_cfg);
      g_mot_120_degree1.p_api->pulsesSet(g_mot_120_degree1.p_ctrl,0);

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
      R_BSP_SoftwareDelay(20, BSP_DELAY_UNITS_MILLISECONDS);

      motor0.motor_ctrl_instance->p_api->errorCheck(motor0.motor_ctrl_instance->p_ctrl,&motor0.status);
      motor1.motor_ctrl_instance->p_api->errorCheck(motor1.motor_ctrl_instance->p_ctrl,&motor1.status);


}

return_t motor_is_speed_achieved(st_motor_t *mot,bool_t *res)
{
    return_t ret = X_RET_OK;

#if FW_CHECK_PARAM_ENABLE == 1
    ASSERT(mot  != NULL)
    ASSERT(res  != NULL)
#endif

    motor_120_control_hall_instance_ctrl_t * p_instance_ctrl = (motor_120_control_hall_instance_ctrl_t *) mot->motor_hall_instance;

    if(p_instance_ctrl->active == MOTOR_120_CONTROL_STATUS_ACTIVE)
    {
        if(p_instance_ctrl->extSettings->active == TRUE)
        {
            float diff = fabsf(p_instance_ctrl->f4_v_ref - p_instance_ctrl->extSettings->voltage);
            if(diff <= 0.02f)
            {
                *res = TRUE;
            }
            else
            {
                *res = FALSE;
            }
        }
        else
        {
            float delta = p_instance_ctrl->f4_ref_speed_rad * 0.02f;
            if(  (p_instance_ctrl->f4_speed_rad >= (p_instance_ctrl->f4_ref_speed_rad-delta)) &&
                 (p_instance_ctrl->f4_speed_rad <= (p_instance_ctrl->f4_ref_speed_rad+delta)))
            {
                *res = TRUE;
            }
            else
            {
                *res = FALSE;
            }
        }
    }
    else
    {
        *res = FALSE;
    }

    return ret;
}
static void gpt_periodset (timer_ctrl_t * const p_ctrl, uint32_t const period_counts, uint32_t const value)
{
    gpt_instance_ctrl_t * p_instance_ctrl = (gpt_instance_ctrl_t *) p_ctrl;

    p_instance_ctrl->p_reg->GTPBR = period_counts;          /* Set period to buffer register */
    p_instance_ctrl->p_reg->GTPR = (uint32_t)(value);
}



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
            if (MOTOR_120_DEGREE_CTRL_STATUS_ERROR != motor0.status)
            {

                motor0.motor_ctrl_instance->p_api->errorCheck(motor0.motor_ctrl_instance->p_ctrl, &motor0.check_error);
            }
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
            if (MOTOR_120_DEGREE_CTRL_STATUS_ERROR != motor1.status)
            {
                motor1.motor_ctrl_instance->p_api->errorCheck(motor1.motor_ctrl_instance->p_ctrl, &motor1.check_error);
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

