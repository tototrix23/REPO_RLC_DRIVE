/*
 * moteur.h
 *
 *  Created on: 11 oct. 2023
 *      Author: Ch.Leclercq
 */

#ifndef APPLICATION_MOTEUR_MOTEUR_H_
#define APPLICATION_MOTEUR_MOTEUR_H_

#include <stdint.h>
#include "rm_motor_api.h"
#include <hal_data.h>
#include <_core/c_common.h>


extern motor_instance_t g_mot_120_degree0;
extern motor_120_driver_instance_t g_mot_120_driver0;
extern motor_instance_t g_mot_120_degree1;
extern motor_120_driver_instance_t g_mot_120_driver1;

typedef struct st_motor_t
{
    motor_instance_t *motor_ctrl_instance;
    motor_120_driver_instance_t *motor_driver_instance;
    uint8_t status;
    uint16_t check_error;
}st_motor_t;


extern st_motor_t motor0;
extern st_motor_t motor1;


void motor_structures_init(void);
void motor_init_fsp(void);
#endif /* APPLICATION_MOTEUR_MOTEUR_H_ */
