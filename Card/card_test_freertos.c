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

#include "FreeRTOS.h"
#include "semphr.h"
#include "event_groups.h"

#include "fsl_clock_manager.h"
#include "fsl_debug_console.h"
   
#include "card_test_function.h"

static volatile xSemaphoreHandle g_cmdEvent = {0}, g_dataEvent = {0}, 
                                 g_cardDetectEvent = {0};
static uint32_t g_lastMsValue = {0};

#define FREERTOS_TIME_RANGE 0xFFFFFFFFU
/* task prio */
#define TASK_TEST_SDHC_CARD_PRIO         6U
/* task size */
#define TASK_TEST_SDHC_CARD_STACK_SIZE   1024U

portSTACK_TYPE* task_test_sdhc_card_stack = NULL;   
TaskHandle_t task_test_sdhc_card_task_handler;

/* Converts milliseconds to ticks */
#define MSEC_TO_TICK(msec)  (((uint32_t)(msec)+500uL/(uint32_t)configTICK_RATE_HZ) \
                             *(uint32_t)configTICK_RATE_HZ/1000uL)
#define TICKS_TO_MSEC(tick) ((tick)*1000uL/(uint32_t)configTICK_RATE_HZ)

bool createCmdEvent()
{
    g_cmdEvent = xSemaphoreCreateCounting(0xFF, 0);
    if (g_cmdEvent == NULL)
    {
        return false;
    }
    return true;
}

bool waitCmdEvent(uint32_t timeout)
{
    uint32_t timeoutTicks;
    if (timeout == FSL_HOST_WAIT_FOREVER)
    {
        timeoutTicks = portMAX_DELAY;
    }
    else
    {
        timeoutTicks = MSEC_TO_TICK(timeout);
    }
    if (xSemaphoreTake(g_cmdEvent, timeoutTicks)==pdFALSE)
    {
        return false; /* timeout */
    }
    else
    {
        return true; /* semaphore taken */
    }
}

bool notifyCmdEvent()
{
    portBASE_TYPE taskToWake = pdFALSE;
    if (pdTRUE==xSemaphoreGiveFromISR(g_cmdEvent, &taskToWake))
    {
        if (pdTRUE == taskToWake)
        {
            vPortYieldFromISR();
        }
        return true;
    }
    else
    {
        return false;
    }
}

bool deleteCmdEvent()
{
    vSemaphoreDelete(g_cmdEvent);
    return true;
}

bool createDataEvent()
{
    g_dataEvent = xSemaphoreCreateCounting(0xFF, 0);
    if (g_dataEvent == NULL)
    {
        return false;
    }
    return true;
}

bool waitDataEvent(uint32_t timeoutMsec)
{
    uint32_t timeoutTicks;
    if (timeoutMsec == FSL_HOST_WAIT_FOREVER)
    {
        timeoutTicks = portMAX_DELAY;
    }
    else
    {
        timeoutTicks = MSEC_TO_TICK(timeoutMsec);
    }
    if (xSemaphoreTake(g_dataEvent, timeoutTicks)==pdFALSE)
    {
        return false; /* timeout */
    }
    else
    {
        return true; /* semaphore taken */
    }
}

bool notifyDataEvent()
{
    portBASE_TYPE taskToWake = pdFALSE;
    if (pdTRUE==xSemaphoreGiveFromISR(g_dataEvent, &taskToWake))
    {
        if (pdTRUE == taskToWake)
        {
            vPortYieldFromISR();
        }
        return true;
    }
    else
    {
        return false;
    }
}

bool deleteDataEvent()
{
    vSemaphoreDelete(g_dataEvent);
    return true;
}

bool createCardDetectEvent()
{
    g_cardDetectEvent = xSemaphoreCreateCounting(0xFF, 0);
    if (g_cardDetectEvent == NULL)
    {
        return false;
    }
    return true;
}

bool waitCardDetectEvent(uint32_t timeout)
{
    uint32_t timeoutTicks;
    if (timeout == FSL_HOST_WAIT_FOREVER)
    {
        timeoutTicks = portMAX_DELAY;
    }
    else
    {
        timeoutTicks = MSEC_TO_TICK(timeout);
    }
    if (xSemaphoreTake(g_cardDetectEvent, timeoutTicks)==pdFALSE)
    {
        return false; /* timeout */
    }
    else
    {
        return true; /* semaphore taken */
    }
}

bool notifyCardDetectEvent()
{
    portBASE_TYPE taskToWake = pdFALSE;
    if (pdTRUE==xSemaphoreGiveFromISR(g_cardDetectEvent, &taskToWake))
    {
        if (pdTRUE == taskToWake)
        {
            vPortYieldFromISR();
        }
        return true;
    }
    else
    {
        return false;
    }
}

bool deleteCardDetectEvent()
{
    vSemaphoreDelete(g_cardDetectEvent);
    return true;
}

void markStartTimeMsec()
{
    portTickType ticks;

    if (__get_IPSR())
    {
        ticks = xTaskGetTickCountFromISR();
    }
    else
    {
        ticks = xTaskGetTickCount();
    }

    g_lastMsValue = TICKS_TO_MSEC(ticks);
}

uint32_t getElapsedTimeMsec()
{
    uint32_t currentMsValue;
    portTickType ticks;

    if (__get_IPSR())
    {
        ticks = xTaskGetTickCountFromISR();
    }
    else
    {
        ticks = xTaskGetTickCount();
    }

    currentMsValue = TICKS_TO_MSEC(ticks);

    if (currentMsValue < g_lastMsValue)
    {
        return (currentMsValue += (FREERTOS_TIME_RANGE - g_lastMsValue));
    }
    return (currentMsValue - g_lastMsValue);
}

void delayTimeMsec(uint32_t msec)
{
    vTaskDelay(MSEC_TO_TICK(msec));
}

static void task_test_sdhc_card(void *param)
{
    //test_card_detection();
    test_data_access();
}

int main(void)
{
    BaseType_t xReturn = pdFAIL;
    
    init_hardware();

    NVIC_SetPriority(SDHC_IRQn, 6U);
//#if defined CD_USING_GPIO
    NVIC_SetPriority(PORTA_IRQn, 6U);
#if (PORT_INSTANCE_COUNT > 1)
    NVIC_SetPriority(PORTB_IRQn, 6U);
#endif       
#if (PORT_INSTANCE_COUNT > 2)
    NVIC_SetPriority(PORTC_IRQn, 6U);
#endif   
#if (PORT_INSTANCE_COUNT > 3)
    NVIC_SetPriority(PORTD_IRQn, 6U);
#endif   
#if (PORT_INSTANCE_COUNT > 4)
    NVIC_SetPriority(PORTE_IRQn, 6U);
#endif   
#if (PORT_INSTANCE_COUNT > 5)
    NVIC_SetPriority(PORTF_IRQn, 6U);
#endif   
//#endif /* CD_USING_GPIO */
    // create app tasks
    xReturn = xTaskCreate(task_test_sdhc_card, 
                          "test_sdhc_card", 
                          TASK_TEST_SDHC_CARD_STACK_SIZE/sizeof(portSTACK_TYPE), 
                          NULL, 
                          (configMAX_PRIORITIES - 2), 
                          &task_test_sdhc_card_task_handler);
    if (xReturn == pdFAIL)
    {
        PRINTF("Failed to create test-sdhc-card task\n\n");
        return -1;
    }

    vTaskStartScheduler();

    for(;;) {}                    // Should not achieve here
}
