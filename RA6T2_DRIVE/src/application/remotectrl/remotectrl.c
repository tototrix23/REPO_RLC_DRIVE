/*
 * telecommande.c
 *
 *  Created on: 13 oct. 2023
 *      Author: Ch.Leclercq
 */

#include <motor/motor.h>
#include <motor/drive_mode.h>
#include "remotectrl.h"

bsp_io_level_t m12_auto=0;
bsp_io_level_t m12_enrl=0;
bsp_io_level_t m12_enrh=0;
bsp_io_level_t m12_derh=0;


return_t remotectrl_enter_manual(void);
return_t remotectrl_exit_manual(void);


return_t remotectrl_process(void)
{



	R_IOPORT_PinRead(&g_ioport_ctrl, M12_AUTO,&m12_auto );
	R_IOPORT_PinRead(&g_ioport_ctrl, M12_ENRL,&m12_enrl );
	R_IOPORT_PinRead(&g_ioport_ctrl, M12_ENRH,&m12_enrh );
	R_IOPORT_PinRead(&g_ioport_ctrl, M12_DERH,&m12_derh );


	if((m12_auto == REMOTECTRL_ACTIVE_LEVEL) ||
	   (m12_enrl == REMOTECTRL_ACTIVE_LEVEL) ||
	   (m12_enrh == REMOTECTRL_ACTIVE_LEVEL) ||
	   (m12_derh == REMOTECTRL_ACTIVE_LEVEL))
	{
	    if(motors_instance.mode != MOTOR_MANUAL_MODE)
	    {
	        remotectrl_enter_manual();
	        set_drive_mode(MOTOR_MANUAL_MODE);
	    }

	}
	else if(motors_instance.mode == MOTOR_MANUAL_MODE)
	{
	    remotectrl_exit_manual();
	    set_drive_mode(MOTOR_INIT_MODE);
	}


	return X_RET_OK;
}


return_t remotectrl_enter_manual(void)
{
    LOG_D(LOG_STD,"Enter manual mode");
    return X_RET_OK;
}

return_t remotectrl_exit_manual(void)
{
    LOG_D(LOG_STD,"Exit manual mode");
    return X_RET_OK;
}
