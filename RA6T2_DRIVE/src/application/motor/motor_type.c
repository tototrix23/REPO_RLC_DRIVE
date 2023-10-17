/*
 * motor_type.c
 *
 *  Created on: Oct 13, 2023
 *      Author: Christophe
 */

#include <_core/c_salloc/c_salloc.h>
#include <rm_motor_extension.h>
#include <motor/motor.h>
#include "motor_type.h"
#include <return_codes.h>

#undef  LOG_LEVEL
#define LOG_LEVEL     LOG_LVL_DEBUG
#undef  LOG_MODULE
#define LOG_MODULE    "drive"


void motor_itoh_brake_init(void);


return_t motor_init_type(motor_type_t type)
{
    return_t ret = X_RET_OK;

    if(type == MOTOR_TYPE_UNKNOWN || type >= MOTOR_TYPE_COUNT)
        ERROR_SET_AND_RETURN(F_RET_MOTOR_BAD_TYPE);


    if(motors_instance.profil.initialised  == MOTOR_PROFIL_INITIALISED)
        return X_RET_OK;

    switch(type)
    {
        case MOTOR_TYPE_RM_ITOH_BRAKE:
            motor_itoh_brake_init();
            break;

        case MOTOR_TYPE_RM_ALCOM:
            ERROR_SET_AND_RETURN(F_RET_MOTOR_BAD_TYPE);
            break;

        default:
            ERROR_SET_AND_RETURN(F_RET_MOTOR_BAD_TYPE);
            break;
    }


    return ret;
}


void motor_itoh_brake_init(void)
{
    motor_profil_t *ptr = &motors_instance.profil;
    memset(ptr,0x00,sizeof(motor_profil_t));
    ptr->initialised = MOTOR_PROFIL_INITIALISED;
    c_linked_list_init(&ptr->sequences.manual.off_sequence);
    c_linked_list_init(&ptr->sequences.manual.enrh_sequence);
    c_linked_list_init(&ptr->sequences.manual.enrl_sequence);
    c_linked_list_init(&ptr->sequences.manual.enrh_sequence);


    ptr->type = MOTOR_TYPE_RM_ITOH_BRAKE;
    ptr->technology = MOTOR_TECH_BLDC;

    //=====================================================================
    // initialisation des paramètres du mode manuel
    //=====================================================================
    motor_phase_t *phase;
    // OFF
    C_SALLOC(sizeof(motor_phase_t),&phase);
    phase->params_motorH.mode = MOTOR_BRAKE_MODE;
    phase->params_motorL.mode = MOTOR_BRAKE_MODE;
    c_linked_list_append(&ptr->sequences.manual.off_sequence,phase);
    C_SALLOC(sizeof(motor_phase_t),&phase);
    phase->params_motorH.mode = MOTOR_BRAKE_MODE;
    phase->params_motorL.mode = MOTOR_BRAKE_MODE;
    c_linked_list_append(&ptr->sequences.manual.off_sequence,phase);


    // ENRH
    C_SALLOC(sizeof(motor_phase_t),&phase);
    phase->params_motorH.mode = MOTOR_NON_REGULATED_MODE;
    phase->params_motorH.non_regulated.settings.current_max = 0.0f;
    phase->params_motorH.non_regulated.settings.timeout_hall_ms = 0;
    phase->params_motorH.non_regulated.settings.percent = 50;

    phase->params_motorL.mode = MOTOR_NON_REGULATED_MODE;
    phase->params_motorL.non_regulated.settings.current_max = 0.0f;
    phase->params_motorL.non_regulated.settings.timeout_hall_ms = 0;
    phase->params_motorL.non_regulated.settings.percent = 0;
    c_linked_list_append(&ptr->sequences.manual.enrh_sequence,phase);

    // ENRL
    C_SALLOC(sizeof(motor_phase_t),&phase);
    phase->params_motorH.mode = MOTOR_NON_REGULATED_MODE;
    phase->params_motorH.non_regulated.settings.current_max = 0.0f;
    phase->params_motorH.non_regulated.settings.timeout_hall_ms = 0;
    phase->params_motorH.non_regulated.settings.percent = 0;

    phase->params_motorL.mode = MOTOR_NON_REGULATED_MODE;
    phase->params_motorL.non_regulated.settings.current_max = 0.0f;
    phase->params_motorL.non_regulated.settings.timeout_hall_ms = 0;
    phase->params_motorL.non_regulated.settings.percent = -50;
    c_linked_list_append(&ptr->sequences.manual.enrl_sequence,phase);


    // DERH
    C_SALLOC(sizeof(motor_phase_t),&phase);
    phase->params_motorH.mode = MOTOR_NON_REGULATED_MODE;
    phase->params_motorH.non_regulated.settings.current_max = 0.0f;
    phase->params_motorH.non_regulated.settings.timeout_hall_ms = 0;
    phase->params_motorH.non_regulated.settings.percent = -20;

    phase->params_motorL.mode = MOTOR_NON_REGULATED_MODE;
    phase->params_motorL.non_regulated.settings.current_max = 0.0f;
    phase->params_motorL.non_regulated.settings.timeout_hall_ms = 0;
    phase->params_motorL.non_regulated.settings.percent = 0;
    c_linked_list_append(&ptr->sequences.manual.derh_sequence,phase);

}
