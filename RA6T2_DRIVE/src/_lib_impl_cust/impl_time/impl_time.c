/*
 * h_time.c
 *
 *  Created on: 11 oct. 2023
 *      Author: Ch.Leclercq
 */
#include <hal_data.h>
#include <r_gpt.h>
#include "impl_time.h"

static uint64_t impl_time_ms_global = 0;

return_t impl_time_init(void)
{
    return_t err = X_RET_OK;
    fsp_err_t err_fsp = FSP_SUCCESS;
    err = R_GPT_Open (g_timer_generic.p_ctrl, &g_timer_generic_cfg);
    ERROR_RETURN(err_fsp == FSP_SUCCESS, I_RET_ERROR_GENERIC);
    err = R_GPT_Start (g_timer_generic.p_ctrl);
    ERROR_RETURN(err_fsp == FSP_SUCCESS, I_RET_ERROR_GENERIC);
    return err;
}

return_t impl_time_update(c_timespan_h handler)
{
#if FW_CHECK_PARAM_ENABLE == 1
    ASSERT(handler != NULL);
#endif
    return_t err = X_RET_OK;

    uint64_t v1,v2;
    do{
        v1 = impl_time_ms_global;
        v2 = impl_time_ms_global;
    }while(v1 != v2);

    handler->ms = v1;
    return err;
}


void timer_generic_callback (timer_callback_args_t * p_args)
{
    volatile uint8_t xxx=0;
    if(NULL != p_args)
    {

        if (TIMER_EVENT_CYCLE_END  == p_args->event)
        {
            impl_time_ms_global += TIMER_LIB_MS;
        }
    }

}
