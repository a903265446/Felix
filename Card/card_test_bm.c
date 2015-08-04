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

//#include <string.h>

//#include "fsl_debug_console.h"

//#include "fsl_clock_manager.h"
//#include "sdhc.h" 
//#include "card.h"
//#include "board.h"

#include <stdint.h>
#include <stdbool.h>

#include "fsl_clock_manager.h"
#include "fsl_lptmr_hal.h"

#include "card_test_function.h"


static volatile uint32_t g_cmdEvent = 0, g_dataEvent = 0, 
                         g_cardDetectEvent = 0, g_lastMsValue = 0;
#define LPTMR_COUNTER_RANGE (1U<<16U - 1U)

bool createCmdEvent()
{
    g_cmdEvent = 0;
    return true;
}

bool waitCmdEvent(uint32_t timeout)
{
    while (g_cmdEvent == 0) {};
    return true;
}

bool notifyCmdEvent()
{
    g_cmdEvent = 1;
    return true;
}

bool deleteCmdEvent()
{
    g_cmdEvent = 0;
    return true;
}

bool createDataEvent()
{
    g_dataEvent = 0;
    return true;
}

bool waitDataEvent(uint32_t timeoutMsec)
{
    while (g_dataEvent == 0) {};
    return true;
}

bool notifyDataEvent()
{
    g_dataEvent = 1;
    return true;
}

bool deleteDataEvent()
{
    g_dataEvent = 0;
    return true;
}

bool createCardDetectEvent()
{
    g_cardDetectEvent = 0;
    return true;
}

bool waitCardDetectEvent(uint32_t timeout)
{
    while (g_cardDetectEvent == 0) {};
    return true;
}

bool notifyCardDetectEvent()
{
    g_cardDetectEvent = 1;
    return true;
}

bool deleteCardDetectEvent()
{
    g_cardDetectEvent = 0;
    return true;
}

void markStartTimeMsec()
{
    LPTMR_Type *base = LPTMR0;
    g_lastMsValue = LPTMR_HAL_GetCounterValue(base);
}

uint32_t getElapsedTimeMsec()
{
    LPTMR_Type *base = LPTMR0;

    uint32_t currentMsValue = LPTMR_HAL_GetCounterValue(base);

    if (currentMsValue < g_lastMsValue)
    {
        return (currentMsValue += (LPTMR_COUNTER_RANGE - g_lastMsValue));
    }
    return (currentMsValue - g_lastMsValue);
}

void delayTimeMsec(uint32_t msec)
{
    markStartTimeMsec();
    while (getElapsedTimeMsec() < msec);
}

void init_lptmr(void)
{
    lptmr_working_mode_user_config_t working_mode_config;
    lptmr_prescaler_user_config_t prescaler_config;
    LPTMR_Type *base = LPTMR0;

    CLOCK_SYS_EnableLptmrClock(0);
    LPTMR_HAL_Disable(base);
    LPTMR_HAL_ClearIntFlag(base);

    working_mode_config.timerModeSelect = kLptmrTimerModeTimeCounter;
    working_mode_config.freeRunningEnable = true;
    working_mode_config.pinPolarity = kLptmrPinPolarityActiveHigh;
    working_mode_config.pinSelect = kLptmrPinSelectInput0;
    LPTMR_HAL_SetTimerWorkingMode(base, working_mode_config);

    prescaler_config.prescalerValue = kLptmrPrescalerDivide2;
    prescaler_config.prescalerBypass = true;
    prescaler_config.prescalerClockSelect = kLptmrPrescalerClock1;
    LPTMR_HAL_SetPrescalerMode(base, prescaler_config);
    LPTMR_HAL_Enable(base);
}

int main(void)
{
    init_hardware();
    init_lptmr();
    //test_card_detection();
    test_data_access();
    return 0;
}



