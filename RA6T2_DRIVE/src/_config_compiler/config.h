/*
 * config.h
 *
 *  Created on: Aug 28, 2023
 *      Author: Christophe
 */

#ifndef CONFIG_COMPILER_CONFIG_H_
#define CONFIG_COMPILER_CONFIG_H_

#include <_target/t_targets.h>
#include <_target/t_operating_systems.h>
#include <__config/config.h>




#undef DEBUG_MODE


#define R_LIB_UNIT_TEST_MODE
#define TARGET                        TARGET_RENESAS_RA
#define OPERATING_SYSTEM              T_OS_BARE_METAL

#define LOG_ENABLE                    0
#define R_LIB_CHECK_PARAM_ENABLE      1
#define R_LIB_ASSERT_MODE             R_LIB_ASSERT_RETURN_CODE
#define R_LIB_LOG_LEVEL               LOG_LVL_WARN

#define USE_SALLOC                    1
#define SALLOC_SIZE_BYTES             4000U


// FIRMWARE
#define FW_CHECK_PARAM_ENABLE         1
#define FW_LOG_CHANNEL                1

#endif /* CONFIG_COMPILER_CONFIG_H_ */
