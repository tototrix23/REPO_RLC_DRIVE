/*
 * telecommande.c
 *
 *  Created on: 13 oct. 2023
 *      Author: Ch.Leclercq
 */


#include "remotectrl.h"

bsp_io_level_t m12_auto=0;
bsp_io_level_t m12_enrl=0;
bsp_io_level_t m12_enrh=0;
bsp_io_level_t m12_derh=0;

return_t remotectrl_process(void)
{



	R_IOPORT_PinRead(&g_ioport_ctrl, M12_AUTO,&m12_auto );
	R_IOPORT_PinRead(&g_ioport_ctrl, M12_ENRL,&m12_enrl );
	R_IOPORT_PinRead(&g_ioport_ctrl, M12_ENRH,&m12_enrh );
	R_IOPORT_PinRead(&g_ioport_ctrl, M12_DERH,&m12_derh );



	return X_RET_OK;
}
