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

#include "fsl_debug_console.h"
#include "fsl_os_abstraction.h"
#include "fsl_gpio_driver.h"
#include "fsl_clock_manager.h"
#include "sdhc.h" 
#include "card.h"
#include "board.h"
#include "card_test.h"
#include <string.h>

/* Host controller instance number */
#define HOST_INSTANCE 0
#define CD_USING_GPIO

host_t g_host = {0};
sd_t g_sd = {0};

#define TEST_BLOCK_NUM 8
static uint8_t g_refData[FSL_CARD_DEFAULT_BLOCK_SIZE*TEST_BLOCK_NUM];
static uint8_t g_testData[FSL_CARD_DEFAULT_BLOCK_SIZE*TEST_BLOCK_NUM];

#if defined FSL_CARD_DRIVER_USING_IRQ
void SDHC_IRQHandler(void)
{
    SDMMC_IrqHandler(&g_host);
}
#endif

//#if defined CD_USING_GPIO

static semaphore_t cd;
volatile uint32_t g_sdInsertedFlag = 0, g_sdInitFlag = 0;

void sdhc_cd_irqhandler(void)
{
    if (GPIO_DRV_ReadPinInput(kGpioSdhc0Cd))
#if defined FRDM_K64F
        g_sdInsertedFlag = 1;
    else
        g_sdInsertedFlag = 0;
#elif defined TWR_K64F120M || defined TWR_K60D100M || defined TWR_K21F120M || defined TWR_K65F180M || defined TWR_K80F150M
        g_sdInsertedFlag = 0;
    else
        g_sdInsertedFlag = 1;
    OSA_SemaPost(&cd);
#else
#error unknown board
#endif
}

#if defined TWR_K64F120M
#define SDHC_CD_GPIO_PORT GPIOB_IDX
#define SDHC_CD_GPIO_PIN  20
void PORTB_IRQHandler()
{
    if(PORT_HAL_GetPortIntFlag(PORTB) == (1<<SDHC_CD_GPIO_PIN))
    {
        sdhc_cd_irqhandler();
    }
    /* Clear interrupt flag.*/
    PORT_HAL_ClearPortIntFlag(PORTB);
}  

#endif /* defined TWR_K64F120M */
#if defined TWR_K64F120M
#define SDHC_CD_GPIO_PORT GPIOE_IDX
#define SDHC_CD_GPIO_PIN  28
void PORTB_IRQHandler()
{
    if(PORT_HAL_GetPortIntFlag(PORTE) == (1<<SDHC_CD_GPIO_PIN))
    {
        sdhc_cd_irqhandler();
    }
    /* Clear interrupt flag.*/
    PORT_HAL_ClearPortIntFlag(PORTE);
}  

#endif /* defined TWR_K64F120M */

//#endif /* CD_USING_GPIO */

static bool test_card_detection(void)
{
    uint32_t loop_time = 10, status;
    sdmmc_status_t ret = kStatus_SDMMC_NoError;
    while (loop_time)
    {
        if (g_sdInsertedFlag && (!g_sdInitFlag))
        {
            PRINTF("A card is detected\n\r");
            ret = SD_IndentifyCard(&g_host, &g_sd);
            if (kStatus_SDMMC_NoError != ret)
            {
                if (ret == kStatus_SDMMC_NotSupportYet)
                {
                    PRINTF("Oops, this card is not supported yet\r\n");
                }
                else
                {
                    PRINTF("SD_IndentifyCard failed\r\n");
                }
                SDMMC_DeInitHost(&g_host);
                return false;
            }
            g_sdInitFlag = 1;

            //show_card_info(&g_sd, false);
            loop_time--;
            PRINTF("Card is initialized, %d times left\r\n", loop_time);
        }
        else if (!g_sdInsertedFlag && g_sdInitFlag)
        {
            SD_Shutdown(&g_sd);
            g_sdInitFlag = 0;
            PRINTF("Card is removed\n\r");
        }
#if defined CD_USING_GPIO
        do
        {
            status = OSA_SemaWait(&cd, OSA_WAIT_FOREVER);
        } while (status == kStatus_OSA_Idle);
#else
        do
        {
            if (g_sdInsertedFlag && (status == kStatus_SDMMC_NoCardInsertedError))
            {
                g_sdInsertedFlag = 0;
                break;
            }
            status = SD_DetectCard(&g_host);
            if (status == kStatus_SDMMC_CardDetectNotSupportYet)
                return false;
            OSA_TimeDelay(100);
        } while (status == kStatus_SDMMC_NoCardInsertedError);
        if (status == kStatus_SDMMC_NoError)
        {
            g_sdInsertedFlag = 1;
        }
#endif
    }
    return true;
}
#if ! defined FSL_USE_SDSPI
uint32_t init_sdhc_hardware(uint32_t instance)
{
    uint32_t err = 0;

    switch(instance)
    {
        case 0:
            configure_sdhc_pins(0);
#if defined (CD_USING_DAT3) || defined (CD_USING_POLL_DAT3)
            /* PTE4      D3 */
            PORT_HAL_SetMuxMode(PORTE, 4, kPortMuxAlt4);
            PORT_HAL_SetPullCmd(PORTE, 4, true);
            PORT_HAL_SetDriveStrengthMode(PORTE, 4, kPortHighDriveStrength);
#else
            /* PTE4      D3 */
            PORT_HAL_SetMuxMode(PORTE, 7, kPortMuxAlt4);
            PORT_HAL_SetPullMode(PORTE, 7, kPortPullUp);
            PORT_HAL_SetPullCmd(PORTE, 7, true);
            PORT_HAL_SetDriveStrengthMode(PORTE, 7, kPortHighDriveStrength);
#endif
            break;
        default:
            err = 1;
            break;
    }
    return err;
}
#endif
void task_test_sdhc_card(task_param_t param)
{
    SDHC_Type* base = &SDHC[HOST_INSTANCE];
    host_t *host = &g_host;
    sd_t *sd = &g_sd;

    host->instance = 0;
    host->cardDetectMode = kSdmmcHostCardDetectGpio;
    //host->transferMode = kSdmmcHostTransModePio;
    //host->transferMode = kSdmmcHostTransModeAdma1;
    host->transferMode = kSdmmcHostTransModeAdma2;
    //init_sdhc_hardware(host->instance);
    configure_sdhc_pins(0);
    
    SDMMC_InitHost(host);

//#if defined CD_USING_GPIO
    //configure_gpio_pins(SDHC_CD_GPIO_PORT);
    GPIO_DRV_Init(sdhcCdPin, NULL);
    if (kStatus_OSA_Success != OSA_SemaCreate(&cd, 0))
    {
        return;
    }
//#endif
    if (kStatus_SDMMC_NoError != SD_IndentifyCard(host, sd))
    {
        return;
    }
    while(1)
    {
        //init_sdhc_hardware(host->instance);
        //SDMMC_InitHost(host);
        //CLOCK_SYS_EnableSdhcClock(0);
       // if (kStatus_SDMMC_NoError != SD_IndentifyCard(host, sd))
       // {
        //    break;
       // }
        //SD_Shutdown(sd);
        //memset(sd, 0, sizeof(sd_t));
        //memset(host, 0, sizeof(host_t));
        memset(g_refData, 1, sizeof(g_refData));
        memset(g_testData, 1, sizeof(g_testData));
        
        if (kStatus_SDMMC_NoError != SD_WriteBlocks(sd, g_testData, 2, 1))
        {
            break;
        }
        if (kStatus_SDMMC_NoError != SD_ReadBlocks(sd, g_testData, 2, 1))
        {
            break;
        }
        if (memcmp(g_testData, g_refData, FSL_CARD_DEFAULT_BLOCK_SIZE))
        {
            break;
        }
        if (kStatus_SDMMC_NoError != SD_WriteBlocks(sd, g_testData, 2, TEST_BLOCK_NUM))
        {
            break;
        }
        if (kStatus_SDMMC_NoError != SD_ReadBlocks(sd, g_testData, 2, TEST_BLOCK_NUM))
        {
            break;
        }
        if (memcmp(g_testData, g_refData, FSL_CARD_DEFAULT_BLOCK_SIZE * TEST_BLOCK_NUM))
        {
            break;
        }
        //CLOCK_SYS_DisableSdhcClock(0);
        SDMMC_DelayMsec(1000);
    }
    
    
    while(1);
    if (!test_card_detection())
    {
        OSA_SemaDestroy(&cd);
        return;
    }
//#if defined CD_USING_GPIO
    OSA_SemaDestroy(&cd);
//#endif

    return;
}

int test_sd(void)
{
    osa_status_t result = kStatus_OSA_Error;
    NVIC_SetPriority(SDHC_IRQn, 6U);
//#if defined FSL_UNIT_TEST_DISABLE_MPU
    MPU->CESR &= (uint32_t) ~(0x1); /* Disable MPU */
//#endif
    
    OSA_Init();
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
    result = OSA_TaskCreate(task_test_sdhc_card,
                    (uint8_t *)"test_sdhc_card",
                    TASK_TEST_SDHC_CARD_STACK_SIZE,
                    task_test_sdhc_card_stack,
                    TASK_TEST_SDHC_CARD_PRIO,
                    (task_param_t)0,
                    false,
                    &task_test_sdhc_card_task_handler);
    if(result != kStatus_OSA_Success)
    {
        PRINTF("Failed to create test-sdhc-card task\n\n");
#if (FSL_RTOS_MQX) && (MQX_COMMON_CONFIG != MQX_LITE_CONFIG)
        return;
#else
        return -1;
#endif
    }

    OSA_Start();

    for(;;) {}                    // Should not achieve here
}

