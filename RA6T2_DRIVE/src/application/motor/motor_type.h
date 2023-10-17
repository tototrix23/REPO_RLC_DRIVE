/*
 * motor_type.h
 *
 *  Created on: Oct 13, 2023
 *      Author: Christophe
 */

#ifndef APPLICATION_MOTOR_MOTOR_TYPE_H_
#define APPLICATION_MOTOR_MOTOR_TYPE_H_

#include <stdint.h>
#include <hal_data.h>
#include <_core/c_common.h>
#include <_core/c_linked_list/c_linked_list.h>

#define MOTOR_PROFIL_INITIALISED     0x12345678U

typedef enum e_motor_type
{
    MOTOR_TYPE_UNKNOWN = 0,
    MOTOR_TYPE_RM_ITOH_BRAKE = 1,
    MOTOR_TYPE_RM_ALCOM = 2,
    MOTOR_TYPE_COUNT    = 3,
}motor_type_t;








/*

               ___________________________
              /                           \
             / |                         | \
            /  |                         |  \
           /   |                         |   \
          /    |                         |    \
         /     |                         |     \_______________
        /      |                         |                     \
_______/       |                         |     |              | \____
               |                         |     |              |
       |       |                         |     |              | |    |
       |       |                         |     |              | |    |
       |       |                         |     |              | |    |
           S1               S2              S3        S4       S5  S6

*/

typedef enum e_motor_control_type
{
    MOTOR_REGULATED_MODE = 0,
    MOTOR_NON_REGULATED_MODE = 1,
    MOTOR_BRAKE_MODE = 2,
}motor_control_type_t;

typedef struct st_motor_control
{
    motor_control_type_t mode;
    struct
    {
        motor_ext_settings_t settings;
    }non_regulated;

    struct
    {
        float rpm;
    }regulated;
}motor_control_t;


typedef struct st_motor_phase
{
    motor_control_t params_motorH;
    motor_control_t params_motorL;
}motor_phase_t;


typedef struct st_motor_profil_t
{
    uint32_t initialised;
    motor_type_t type;
    motor_ext_technology_t technology;

    struct
    {
        struct
        {
            c_linked_list_t off_sequence;
            c_linked_list_t enrl_sequence;
            c_linked_list_t enrh_sequence;
            c_linked_list_t derh_sequence;
        }manual;

        struct
        {

        }init;

        struct
        {

        }automatic;
    }sequences;


}motor_profil_t;

return_t motor_init_type(motor_type_t type);


#endif /* APPLICATION_MOTOR_MOTOR_TYPE_H_ */
