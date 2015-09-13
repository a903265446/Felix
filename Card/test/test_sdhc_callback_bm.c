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

#include "test_sdhc_callback.h"
#include "test_sdhc_board.h"
#include "fsl_os_abstraction.h"



static volatile uint32_t g_cmdEvent = 0, g_dataEvent = 0, g_cardDetectEvent = 0;
#define LPTMR_COUNTER_RANGE (65535U)

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

uint32_t LPTMR_GetCountValue(LPTMR_Type *base)
{
    return 0;
}

uint32_t getCurrentTimeMsec()
{
    LPTMR_Type *base = LPTMR0;
    return LPTMR_GetCountValue(base);
}

uint32_t getTimeRangeMsec()
{
    return LPTMR_COUNTER_RANGE;
}

void delayTimeMsec(uint32_t msec)
{
    uint32_t lastMs, currentMs, elapsedMs;
    LPTMR_Type *base = LPTMR0;
    
    lastMs = LPTMR_GetCountValue(base);
    elapsedMs = 0;
    while (elapsedMs < msec)
    {
        currentMs = LPTMR_GetCountValue(base);
        if (currentMs < lastMs)
        {
            currentMs += LPTMR_COUNTER_RANGE;
        }
        elapsedMs = currentMs - lastMs;
    }    
}

void init_bm_callback(void)
{
    /* Set LPTMR to 1HZ clock source, freeRunning mode. */
    /* Enable LPTMR */
    //OSA_Init();
    //delayTimeMsec(1);
}




