/*
 * motor_process.c
 *
 *  Created on: 13 oct. 2023
 *      Author: Ch.Leclercq
 */
#include "drive_process.h"

#undef  LOG_LEVEL
#define LOG_LEVEL     LOG_LVL_DEBUG
#undef  LOG_MODULE
#define LOG_MODULE    "drive"

return_t motor_process(st_motor_t *mot);

return_t drive_process(void)
{


    motor_process(motors_instance.motorH);
    motor_process(motors_instance.motorL);
    return X_RET_OK;
}




return_t motor_process(st_motor_t *mot)
{
	return_t ret = X_RET_OK;

#if FW_CHECK_PARAM_ENABLE == 1
    ASSERT(mot  != NULL)
#endif


	mot->motor_ctrl_instance->p_api->statusGet(mot->motor_ctrl_instance->p_ctrl, &mot->status);
    switch(mot->status)
    {
        case MOTOR_120_DEGREE_CTRL_STATUS_STOP:
        {

        }
        break;

        case MOTOR_120_DEGREE_CTRL_STATUS_RUN:
		{

		}
		break;

        case MOTOR_120_DEGREE_CTRL_STATUS_BRAKE:
		{

		}
		break;

        case MOTOR_120_DEGREE_CTRL_STATUS_ERROR:
		{

		}
		break;
    }
	return ret;
}
