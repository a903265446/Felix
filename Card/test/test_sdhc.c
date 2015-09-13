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

#include "sdhc.h"
#include "test_sdhc_callback.h" 
#include "test_sdhc_board.h"
#include "test_sdhc.h"

/*******************************Test register write/read*************************/
/* This kind of test mainly test SDHC register read/write function */
void test_sdhc_read_write_register(void)
{
    return;
}


/*******************************Test normal case**************************************************/
sdhc_host_t g_host;
static sdhc_capability_t g_sdhcCapability; 
#define ADMA_TABLE_MAX_ENTRIES 2
static uint32_t g_admaTable[ADMA_TABLE_MAX_ENTRIES];

#if defined CD_USING_GPIO || defined CD_USING_DAT3
void detect_card_inserted(void)
{
    notifyCardDetectEvent();
}
void detect_card_removed(void)
{
    notifyCardDetectEvent();
}
#endif

static bool test_card_detect(void)
{
    uint32_t loop_time = 10;
    sdhc_status_t ret;
    createCardDetectEvent();

    while (loop_time)
    {
#if defined CD_USING_GPIO || defined CD_USING_DAT3
        waitCardDetectEvent(FSL_SDHC_WAIT_FOREVER);
#elif defined CD_USING_POLL_DAT3
        do
        {
            ret = SDHC_DetectCard(&g_host);
            switch (ret)
            {
                case kStatus_SDHC_UnknownError:
                    return;
                case kStatus_SDHC_NoCardInsertedError:
                    /* delay 300ms */
                case kStatus_Success:
                    break;
            }            
        } while (1)
#endif
        loop_time--;
    }
    
    return true;
}

static bool test_SDHC_InitHost(void)
{
    sdhc_host_user_config_t config;

    config.base = SDHC;
#if defined CD_USING_GPIO
    config.cardDetectMode = kSDHCCardDetectGpio;
#elif defined CD_USING_DAT3
    config.cardDetectMode = kSDHCCardDetectDat3;
#elif defined CD_USING_POLL_DAT3
    config.cardDetectMode = kSDHCCardDetectPollDat3;
#endif

    config.transferMode = kSDHCTransModeAdma2;
    config.capability = &g_sdhcCapability;
    config.admaTableAddress = g_admaTable;
    config.admaTableMaxEntries = ADMA_TABLE_MAX_ENTRIES;
    config.createCmdEvent = createCmdEvent;
    config.waitCmdEvent = waitCmdEvent;
    config.notifyCmdEvent = notifyCmdEvent;
    config.deleteCmdEvent = deleteCmdEvent;
    config.createDataEvent = createDataEvent;
    config.waitDataEvent = waitDataEvent;
    config.notifyDataEvent = notifyDataEvent;
    config.deleteDataEvent = deleteDataEvent;
    config.getCurrentTimeMsec = getCurrentTimeMsec;
    config.timeRangeMsec = getTimeRangeMsec();
    //config.delayTimeMsec = delayTimeMsec;
    config.cardIntCallback = NULL;
#if defined CD_USING_DAT3
    config.cardInsertedCallback = detectCardInserted;
    config.cardRemovedCallback = detectCardRemoved;
#elif defined CD_USING_GPIO || defined CD_USING_POLL_DAT3
    config.cardInsertedCallback = NULL;
    config.cardRemovedCallback = NULL;
#endif
    config.blockGapCallback = NULL;

    if (kStatus_Success != SDHC_InstallStateCallback(&g_host, &config))
    {
        return false;
    }
    return true;
}

static bool test_SDHC_DeInitHost(void)
{
    if (kStatus_Success != SDHC_DeInit())
    {
        return false;
    }
    return true;
}

/*
Mainly test following cases:
•   Init, DeInit
•   Card detection 
 */
void test_sdhc_normal_case(void)
{
    init_sdhc_hardware();

    test_SDHC_InitHost();

    test_card_detect();

    test_SDHC_DeInitHost();
}

/*******************************Test exception case***********************************************/
static void test_param_null(void)
{
  
}

void SDHC_IRQHandler(void)
{
    SDHC_IrqHandler(&g_host);
}

static void test_set_force_event_error_prompt(void)
{
    SDHC_SetForceEvent(SDHC, 0xFFFFFFFFU);
    /* Check driver code error prompt */
}

static bool test_not_supported_transfer_type(void)
{
    return true;
}

static void test_not_supported_function(void)
{
    if (!test_not_supported_transfer_type())
    {
        return;
    }
}
/*
Mainly test following case:
•   Function parameter is NULL.
•   Error trigged by set force event function.
•   Error’s prompt when enables the configuration items not supported yet such as SDMA transfer type.
 */
void test_sdhc_exception_case(void)
{
    test_param_null();
    test_set_force_event_error_prompt();
    test_not_supported_function();
}

/**********************************Test performance***********************************************/

/**********************************Sress test***********************************************/

int main(void)
{
    test_sdhc_read_write_register();
    test_sdhc_normal_case();
    test_sdhc_exception_case();
    
    return 0;
}