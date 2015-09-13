/*
 * Copyright (c) 2013 - 2015, Freescale Semiconductor, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of Freescale Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <stdint.h>
#include <stdbool.h>
#include "test_sdhc_board.h"
#include "board.h"
#include "fsl_os_abstraction.h"
#include "fsl_lptmr_hal.h"
#include "fsl_clock_manager.h"




bool card_detect_pin_dectect_card_inserted_Level(void)
{
    return true;
}

bool card_detect_pin_has_level_changed_interrupt(void)
{
    return true;
}
bool card_detect_pin_clear_level_changed_interrupt(void)
{
    return true;
}

void PORT_IRQHandler(PORT_Type *base)
{
    /* Get interrupt flag */
    if (card_detect_pin_has_level_changed_interrupt())
    {
        if (card_detect_pin_dectect_card_inserted_Level())
        {
            detect_card_inserted();
        }
        else
        {
            detect_card_removed();
        }
    }
    
    /* Clear interrupt flag.*/
    card_detect_pin_has_level_changed_interrupt();
}

static void set_sdhc_pin_mux()
{
    configure_sdhc_pins(0);
} 

extern void init_bm_callback(void);

void init_sdhc_hardware(void)
{

    /* Disable MPU */
//    MPU->CESR &= (uint32_t) ~(0x1); 
    
    hardware_init();

    set_sdhc_pin_mux();
    
    OSA_Init();
    
//        lptmr_working_mode_user_config_t working_mode_config;
//    lptmr_prescaler_user_config_t prescaler_config;
//    LPTMR_Type *base = LPTMR0;
//
//    CLOCK_SYS_EnableLptmrClock(0);
//    LPTMR_HAL_Disable(base);
//    LPTMR_HAL_ClearIntFlag(base);
//
//    working_mode_config.timerModeSelect = kLptmrTimerModeTimeCounter;
//    working_mode_config.freeRunningEnable = true;
//    working_mode_config.pinPolarity = kLptmrPinPolarityActiveHigh;
//    working_mode_config.pinSelect = kLptmrPinSelectInput0;
//    LPTMR_HAL_SetTimerWorkingMode(base, working_mode_config);
//
//    prescaler_config.prescalerValue = kLptmrPrescalerDivide2;
//    prescaler_config.prescalerBypass = true;
//    prescaler_config.prescalerClockSelect = kLptmrPrescalerClock1;
//    LPTMR_HAL_SetPrescalerMode(base, prescaler_config);
//    LPTMR_HAL_Enable(base);
    
    //init_bm_callback();
}
