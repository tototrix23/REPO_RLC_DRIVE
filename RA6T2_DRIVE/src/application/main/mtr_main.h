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
* File Name   : mtr_main.h
* Description : Definitions for the application layer
***********************************************************************************************************************/
/**********************************************************************************************************************
* History : DD.MM.YYYY Version
*         : 09.12.2021 1.00
***********************************************************************************************************************/

/* Guard against multiple inclusion */
#ifndef MTR_MAIN_H
#define MTR_MAIN_H

/***********************************************************************************************************************
* Includes <System Includes> , "Project Includes"
***********************************************************************************************************************/
#include <stdint.h>

/***********************************************************************************************************************
* Macro definitions
***********************************************************************************************************************/
#define SW_ON                    (0)                            /* Active level of SW */
#define SW_OFF                   (1)                            /* Inactive level of SW */
#define SW1_ON                   (1)                            /* Active level of SW */
#define SW1_OFF                  (0)                            /* Inactive level of SW */
#define SW2_ON                   (0)                            /* Active level of SW */
#define SW2_OFF                  (1)                            /* Inactive level of SW */
#define CHATTERING_CNT           (10)                           /* Counts to remove chattering */
#define MTR_LED_ON               (BSP_IO_LEVEL_LOW)             /* Active level of LED */
#define MTR_LED_OFF              (BSP_IO_LEVEL_HIGH)            /* Inactive level of LED */
#define MTR_CW                   (0)                            /* CW */
#define MTR_CCW                  (1)                            /* CCW */
#define ICS_UI                   (0)                            /* ICS (Analyzer) */
#define BOARD_UI                 (1)                            /* Board */
#define MTR_MAX_SPEED_RPM        (2400.0f)
#define STOP_RPM                 (600.0f)
#define VR1_SCALING              ((MTR_MAX_SPEED_RPM + 100 ) / (MTR_AD12BIT_DATA * 0.5f))
                                                                /* scaling factor for speed reference (A/D) */
                                                                /* (MAX SPEED+margin)/AD(12bit) */
#define ADJUST_OFFSET            (0x7FF)                        /* Adjusting offset for reference */
#define MTR_FLG_CLR              (0)                            /* For flag clear */
#define MTR_FLG_SET              (1)                            /* For flag set */
/* Defines the UI used as default UI (BOARD_UI/ICS_UI)*/
#define CONFIG_DEFAULT_UI        (BOARD_UI)

#define MTR_AD12BIT_DATA         (4095.0f)                      /* A/D 12Bit data */
#define MTR_ADCH_VR1             (8)                            /* A/D channel of vr1 */

#define MTR_PORT_SW1             (BSP_IO_PORT_13_PIN_04)        /* Input port of SW1 */
#define MTR_PORT_SW2             (BSP_IO_PORT_13_PIN_07)        /* Input port of SW2 */

#define MTR_PORT_LED1            (BSP_IO_PORT_13_PIN_01)        /* Output port of LED1 */
#define MTR_PORT_LED2            (BSP_IO_PORT_13_PIN_02)        /* Output port of LED2 */
#define MTR_PORT_LED3            (BSP_IO_PORT_13_PIN_03)        /* Output port of LED3 */

/***********************************************************************************************************************
* global functions
***********************************************************************************************************************/
/***********************************************************************************************************************
* Function Name : mtr_init
* Description   : Initialization for Motor Control
* Arguments     : None
* Return Value  : None
***********************************************************************************************************************/
void mtr_init( void );

/***********************************************************************************************************************
* Function Name : mtr_main
* Description   : Initialization and main routine
* Arguments     : None
* Return Value  : None
***********************************************************************************************************************/
void mtr_main(void);

#endif /* MTR_MAIN_H */
