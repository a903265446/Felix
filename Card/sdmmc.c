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

 
 #include <assert.h>
#include "fsl_clock_manager.h"
#include "sdhc.h"
#include "card.h"


/* Following part is the common function between SD card and MMC card */

#if !defined FSL_CARD_DRIVER_USING_DYNALLOC
#define FSL_HOST_ADMA_TABLE_MAX_ENTRY       16
static uint32_t g_admaTableAddress[SDHC_INSTANCE_COUNT][FSL_HOST_ADMA_TABLE_MAX_ENTRY >> 1];
static semaphore_t g_cmdComplete = {0};
static semaphore_t g_dataComplete = {0};
host_capability_t g_hostCapability = {0};
sdmmc_status_t SDMMC_AllocStaticMemory(host_t * host)
{
    assert(host);
    SDHC_Type* base = &SDHC[host->instance];
    host->admaTableAddress = g_admaTableAddress[host->instance];
    //SDHC_SetAdmaAddress(base, (uint32_t)host->admaTableAddress);
    host->capability = &g_hostCapability;
    return kStatus_SDMMC_NoError;
}
#endif

sdmmc_status_t SDMMC_InitHost(host_t *host)
{
    uint32_t irqEnabled;
    assert(host);
    SDHC_Type *base = &SDHC[host->instance];
    sdhc_config_t sdhcConfig = {0};

    if (host->cardDetectMode == kSdmmcHostCardDetectDat3 ||
            host->cardDetectMode == kSdmmcHostCardDetectPollDat3)
    {
        sdhcConfig.dat3AsCardDetectPinEnable = true;
    }
#if defined FSL_HOST_USING_BIG_ENDIAN
    sdhcConfig.endianMode = kSdhcEndianBig;
#else
    sdhcConfig.endianMode = kSdhcEndianLittle;
#endif
    sdhcConfig.readWatermarkLevel = 0x80;
    sdhcConfig.writeWatermarkLevel = 0x80;
    switch (host->transferMode)
    {
        case kSdmmcHostTransModePio:
        case kSdmmcHostTransModeSdma:
            sdhcConfig.dmaMode = kSdhcDmaNoOrSimple;
            break;
        case kSdmmcHostTransModeAdma1:
            sdhcConfig.dmaMode = kSdhcDmaAdma1;
            break;
        case kSdmmcHostTransModeAdma2:
            sdhcConfig.dmaMode = kSdhcDmaAdma2;
            break;
        default:
            return kStatus_SDMMC_Failed;
            break;
    }
    CLOCK_SYS_EnableSdhcClock(host->instance);
    SDHC_Reset(base, SDHC_RST_TYPE_ALL, 100);
/* Enable all interrupts */
    SDHC_SetIntState(base, false, (uint32_t)-1);
    SDHC_SetIntSignal(base, false, (uint32_t)-1);
    irqEnabled = SDHC_CMD_INDEX_ERR_INT | SDHC_CMD_CRC_ERR_INT |
                 SDHC_CMD_END_BIT_ERR_INT | SDHC_CMD_TIMEOUT_ERR_INT |
                 SDHC_DATA_TIMEOUT_ERR_INT | SDHC_DATA_CRC_ERR_INT |
                 SDHC_DATA_END_BIT_ERR_INT | SDHC_CMD_COMPLETE_INT |
                 SDHC_DATA_COMPLETE_INT;
#if defined FSL_SDHC_ENABLE_AUTOCMD12
    irqEnabled |= SDHC_AUTO_CMD12_ERR_INT;
#endif
    if ((host->cardDetectMode == kSdmmcHostCardDetectCdPin) ||
            (host->cardDetectMode == kSdmmcHostCardDetectDat3))
    {
        irqEnabled |= (SDHC_CARD_INSERTION_INT |
                       SDHC_CARD_REMOVAL_INT);
    }
    SDHC_SetIntState(base, true, irqEnabled);
#if defined FSL_CARD_DRIVER_USING_IRQ
    SDHC_SetIntSignal(base, true, irqEnabled);
#endif
    
    SDHC_InitHost(host->instance, &sdhcConfig);
    
#if !defined FSL_CARD_DRIVER_USING_DYNALLOC
    SDMMC_AllocStaticMemory(host);
#endif
    
    SDHC_GetCapability(base, host->capability);
    SDHC_SetCardActive(base, 100);
    return kStatus_SDMMC_NoError;
}
sdmmc_status_t SDMMC_DeInitHost(host_t *host)
{
    assert(host);
    SDHC_Type *base = &SDHC[host->instance];
    SDHC_DeInitHost(host->instance);
    return kStatus_SDMMC_NoError;
}
/*FUNCTION****************************************************************
 *
 * Function Name: SDMMC_DelayMsec
 * Description: blocking delay msecond
 *
 *END*********************************************************************/
void SDMMC_DelayMsec(uint32_t msec)
{
    uint32_t startTime, elapsedTime;
    assert(msec);

    startTime = OSA_TimeGetMsec();
    do
    {
        elapsedTime = OSA_TimeGetMsec() - startTime;
    } while(elapsedTime < msec);
}
//#define FSL_HOST_CLKMGMT_ENABLED 
static sdmmc_status_t SDMMC_SetClock(uint32_t instance, bool enable)
{
    assert(instance < SDHC_INSTANCE_COUNT);
#if defined FSL_HOST_CLKMGMT_ENABLED
    if (enable)
    {
         CLOCK_SYS_EnableSdhcClock(instance);
    }
    else
    {
         CLOCK_SYS_DisableSdhcClock(instance);
    }
   
#endif
    return kStatus_SDMMC_NoError;
}
/*FUNCTION****************************************************************
 *
 * Function Name: SDMMC_PrepareDmaData
 * Description: Prepare data for transferring
 *
 *END*********************************************************************/
static sdmmc_status_t SDMMC_PrepareDmaData(host_t *host) 
{
    sdmmc_status_t ret;
    uint32_t totalSize, entries;
#if defined FSL_CARD_DRIVER_USING_DYNALLOC
    sdhc_adma1_descriptor_t * adma1TableAddress = NULL;
    sdhc_adma2_descriptor_t * adma2TableAddress = NULL;
#endif
    SDHC_Type *base = &SDHC[host->instance];
    card_data_t *cardData = host->currentData;

    assert(cardData);
    assert(cardData->buffer);
    assert(cardData->blockCount);
    assert(cardData->blockSize);

    ret = kStatus_SDMMC_NoError;
    if ((host->transferMode != kSdmmcHostTransModeAdma2) && (host->transferMode != kSdmmcHostTransModeAdma1))
    {
        return ret;
    }
    totalSize = (cardData->blockSize * cardData->blockCount);
    /*entries = SDHC_GetAdmaEntryNumber(base, totalSize);
    if (entries == 0)
    {
        return kStatus_SDMMC_PrepareHostDmaDataError;
    }*/
    if (host->transferMode == kSdmmcHostTransModeAdma1)
    {
         /* Check data length alignment */
        if (((uint32_t)cardData->buffer % SDHC_ADMA1_ADDR_ALIGN) || (totalSize % SDHC_ADMA1_LEN_ALIGN))
        {
            return kStatus_SDMMC_PrepareHostDmaDataError;
        }
        entries = (totalSize / SDHC_ADMA1_DESC_MAX_LEN_PER_ENTRY) + 1;
        /* ADMA1 needs two descritors to finish a transfer */
        entries *= 2;
    }
    else
    {
        /* Check data length alignment */
        if (((uint32_t)cardData->buffer % SDHC_ADMA2_ADDR_ALIGN) || (totalSize % SDHC_ADMA2_LEN_ALIGN))
        {
            return kStatus_SDMMC_PrepareHostDmaDataError;
        }
        entries = ((totalSize / SDHC_ADMA2_DESC_MAX_LEN_PER_ENTRY) + 1);
    }
    
#if defined FSL_CARD_DRIVER_USING_DYNALLOC
    if (entries > host->admaTableMaxEntries)
    {
        /* Larger table is needed */
        if (host->admaTableAddress)
        {
            OSA_MemFree(host->admaTableAddress);
            host->admaTableAddress = NULL;
            host->admaTableMaxEntries = 0;
        }
        if (host->transferMode == kSdmmcHostTransModeAdma1)
        {
            adma1TableAddress = (sdhc_adma1_descriptor_t *)OSA_MemAllocZero(entries * sizeof(sdhc_adma1_descriptor_t));
            if (adma1TableAddress == NULL)
            {
                host->admaTableMaxEntries = 0;
                /* Failed to alloc memory for ADMA descriptor table */
                return kStatus_SDMMC_AllocHostAdmaTableFailed;
            }
        } 
        else if (host->transferMode == kSdmmcHostTransModeAdma2)
        {
            adma2TableAddress = (sdhc_adma2_descriptor_t *)OSA_MemAllocZero(entries * sizeof(sdhc_adma2_descriptor_t));
            if (adma2TableAddress == NULL)
            {
                host->admaTableMaxEntries = 0;
                /* Failed to alloc memory for ADMA descriptor table */
                return kStatus_SDMMC_AllocHostAdmaTableFailed;
            }
        }
        if (host->admaTableAddress == NULL)
        {
            /* Update ADMA table address */
            host->admaTableAddress = (uint32_t *)adma2TableAddress;
            /* Update ADMA table capacity */
            host->admaTableMaxEntries = entries;
        }
    }
    //SDHC_SetAdmaAddress(base, (uint32_t)host->admaTableAddress);
#else
    if (entries > FSL_HOST_ADMA_TABLE_MAX_ENTRY)
    {
        return kStatus_SDMMC_OutOfMemory;
    }   
#endif  

    SDHC_SetAdmaTable(base, host->admaTableAddress, cardData->buffer, totalSize);
    //SDHC_SetAdmaAddress(base, (uint32_t)host->admaTableAddress);
    return ret;
}

/*FUNCTION****************************************************************
 *
 * Function Name: SDMMC_SendCommand
 * Description: Send command to card
 *
 *END*********************************************************************/
static sdmmc_status_t SDMMC_SendCommand(host_t* host)
{
    uint32_t flags = 0;
    sdhc_card_cmd_config_t cardCmdConfig;
    sdmmc_status_t ret = kStatus_SDMMC_NoError;
    SDHC_Type * base = &SDHC[host->instance];
    card_cmd_t *cardCmd = host->currentCmd;
    card_data_t *cardData = host->currentData;

    if (cardData)
    {
        flags |= SDHC_DATA_PRESENT;

        SDHC_SetIntState(base, false, (SDHC_DMA_ERR_INT | SDHC_DMA_INT | SDHC_BUF_READ_READY_INT | SDHC_BUF_WRITE_READY_INT));
        SDHC_SetIntSignal(base, false, (SDHC_DMA_ERR_INT | SDHC_DMA_INT | SDHC_BUF_READ_READY_INT | SDHC_BUF_WRITE_READY_INT));

        if (cardData->flags & CARD_DATA_FLAGS_USE_DMA)
        {
            flags |= SDHC_ENABLE_DMA;
            SDHC_SetIntState(base, true, (SDHC_DMA_ERR_INT | SDHC_DMA_INT));
#if defined FSL_CARD_DRIVER_USING_IRQ
            SDHC_SetIntSignal(base, true, (SDHC_DMA_ERR_INT | SDHC_DMA_INT));
#endif
        }
        else
        {
            SDHC_SetIntState(base, true, (SDHC_BUF_READ_READY_INT | SDHC_BUF_WRITE_READY_INT));
#if defined FSL_CARD_DRIVER_USING_IRQ
            SDHC_SetIntSignal(base, true, (SDHC_BUF_READ_READY_INT | SDHC_BUF_WRITE_READY_INT));
#endif
        }

        if (cardData->flags & CARD_DATA_FLAGS_DATA_READ)
        {
            flags |= SDHC_ENABLE_DATA_READ;
        }
    }
    /* Defines the flag corresponding to each response type. */
    switch (cardCmd->respType)
    {
        case kSdmmcRespTypeNone:
            break;
        case kSdmmcRespTypeR1:
            flags |= (SDHC_RESP_LEN_48 | SDHC_ENABLE_CRC_CHECK | SDHC_ENABLE_INDEX_CHECK);    /* Response 1 */
            break;
        case kSdmmcRespTypeR1b:
            flags |= (SDHC_RESP_LEN_48_BC | SDHC_ENABLE_CRC_CHECK | SDHC_ENABLE_INDEX_CHECK);  /* Response 1 with busy */
            break;
        case kSdmmcRespTypeR2:
            flags |= (SDHC_RESP_LEN_136 | SDHC_ENABLE_CRC_CHECK);     /* Response 2 */
            break;
        case kSdmmcRespTypeR3:
            flags |= (SDHC_RESP_LEN_48);
            break;
        case kSdmmcRespTypeR4:
            flags |= (SDHC_RESP_LEN_48);
            break;
        case kSdmmcRespTypeR5:
            flags |= (SDHC_RESP_LEN_48 | SDHC_ENABLE_CRC_CHECK);/* Response 5 */
            break;
        case kSdmmcRespTypeR5b:
            flags |= (SDHC_RESP_LEN_48_BC | SDHC_ENABLE_CRC_CHECK | SDHC_ENABLE_INDEX_CHECK);  /* Response 5 with busy */
            break;
        case kSdmmcRespTypeR6:
            flags |= (SDHC_RESP_LEN_48 | SDHC_ENABLE_CRC_CHECK | SDHC_ENABLE_INDEX_CHECK);    /* Response 6 */
            break;
        case kSdmmcRespTypeR7:
            flags |= (SDHC_RESP_LEN_48 | SDHC_ENABLE_CRC_CHECK | SDHC_ENABLE_INDEX_CHECK);     /* Response 7 */
            break;
        default:
            break;
    }

    while((SDHC_GetPresentState(base) & SDHC_CMD_INHIBIT)) {}

    if(cardCmd->flags & CARD_CMD_FLAGS_STOP_TRANS)
    {
        flags |= SDHC_CMD_TYPE_ABORT;
    }
    else if ((cardData) || (cardCmd->respType == kSdmmcRespTypeR1b) || (cardCmd->respType == kSdmmcRespTypeR5b))
    {
        while((SDHC_GetPresentState(base) & SDHC_DAT_INHIBIT)) {}
    }

    if (cardData)
    {
        if (cardData->blockCount > 1)
        {
            flags |= SDHC_MULTIPLE_BLOCK;
            flags |= SDHC_ENABLE_BLOCK_COUNT;
#ifdef FSL_CARD_DIVER_ENABLE_HOST_AUTOCMD12
            /* Enable Auto CMD12 */
            flags |= SDHC_ENABLE_AUTO_CMD12;
#endif
        }
        if (cardData->blockCount > SDHC_MAX_BLOCK_COUNT)
        {
            cardCmdConfig.dataBlockSize  = cardData->blockSize;
            cardCmdConfig.dataBlockCount = SDHC_MAX_BLOCK_COUNT;
            flags &= ~SDHC_ENABLE_BLOCK_COUNT;
        }
        else
        {
            cardCmdConfig.dataBlockSize  = cardData->blockSize;
            cardCmdConfig.dataBlockCount = cardData->blockCount;
        }
    }
    else
    {
        cardCmdConfig.dataBlockSize  = 0;
        cardCmdConfig.dataBlockCount = 0;
    }

    cardCmdConfig.argument  = cardCmd->argument;
    cardCmdConfig.cmdIndex = cardCmd->cmdIndex;
    cardCmdConfig.cmdFlags = flags;
    SDHC_SetCardCommand(base, &cardCmdConfig);
    return ret;
}

/*FUNCTION****************************************************************
 *
 * Function Name: SDMMC_SetReqeustError
 * Description: Set error flags for a given request according to irq flags
 *
 *END*********************************************************************/
/*static void SDMMC_SetReqeustError(host_request_t *req, uint32_t irqFlags)
{
    assert(req);
    if ((!irqFlags) || (!(irqFlags & SDHC_ALL_ERR_INT)))
    {
        return;
    }

    if (irqFlags & SDHC_CMD_CRC_ERR_INT)
    {
        req->error |= HOST_REQ_ERR_CMD_CRC;
    }
    if (irqFlags & SDHC_CMD_INDEX_ERR_INT)
    {
        req->error |= HOST_REQ_ERR_CMD_INDEX;
    }
    if (irqFlags & SDHC_CMD_END_BIT_ERR_INT)
    {
        req->error |= HOST_REQ_ERR_CMD_END_BIT;
    }
    if (irqFlags & SDHC_CMD_TIMEOUT_ERR_INT)
    {
        req->error |= HOST_REQ_ERR_CMD_TIMEOUT;
    }

    if (irqFlags & SDHC_DATA_TIMEOUT_ERR_INT)
    {
        req->error |= HOST_REQ_ERR_DAT_TIMEOUT;
    }
    if (irqFlags & SDHC_DATA_CRC_ERR_INT)
    {
        req->error |= HOST_REQ_ERR_DATA_CRC;
    }
    if (irqFlags & SDHC_DATA_END_BIT_ERR_INT)
    {
        req->error |= HOST_REQ_ERR_DATA_END_BIT;
    }
    if (irqFlags & SDHC_AUTO_CMD12_ERR_INT)
    {
        req->error |= HOST_REQ_ERR_AUTO_CMD12;
    }
    if (irqFlags & SDHC_DMA_ERR_INT)
    {
        req->error |= HOST_REQ_ERR_DMA;
    }
}*/

static void SDMMC_SetCmdError(card_cmd_t *cardCmd, uint32_t irqFlags)
{
    if ((!irqFlags) || (!(irqFlags & SDHC_CMD_ERR_INT)))
    {
        return;
    }
    if (irqFlags & SDHC_CMD_CRC_ERR_INT)
    {
        cardCmd->errors |= CARD_CMD_ERR_CMD_CRC;
    }
    if (irqFlags & SDHC_CMD_INDEX_ERR_INT)
    {
        cardCmd->errors |= CARD_CMD_ERR_CMD_INDEX;
    }
    if (irqFlags & SDHC_CMD_END_BIT_ERR_INT)
    {
        cardCmd->errors |= CARD_CMD_ERR_CMD_END_BIT;
    }
    if (irqFlags & SDHC_CMD_TIMEOUT_ERR_INT)
    {
        cardCmd->errors |= CARD_CMD_ERR_CMD_TIMEOUT;
    }
}
static void SDMMC_SetCmdDataError(card_data_t *cardData, uint32_t irqFlags)
{
    if (irqFlags & SDHC_DATA_TIMEOUT_ERR_INT)
    {
        cardData->errors |= CARD_DATA_ERR_DATA_TIMEOUT;
    }
    if (irqFlags & SDHC_DATA_CRC_ERR_INT)
    {
        cardData->errors |= CARD_DATA_ERR_DATA_CRC;
    }
    if (irqFlags & SDHC_DATA_END_BIT_ERR_INT)
    {
        cardData->errors |= CARD_DATA_ERR_DATA_END_BIT;
    }
    if (irqFlags & SDHC_AUTO_CMD12_ERR_INT)
    {
        cardData->errors |= CARD_DATA_ERR_AUTO_CMD12;
    }
    if (irqFlags & SDHC_DMA_ERR_INT)
    {
        cardData->errors |= CARD_DATA_ERR_DMA;
    }
}

/*FUNCTION****************************************************************
 *
 * Function Name: SDMMC_PioReadBlock
 * Description: Read a block using PIO
 *
 *END*********************************************************************/
static void SDMMC_PioReadBlock(host_t * host)
{
    uint32_t blockSize, blockCount;
    card_data_t *cardData;
    SDHC_Type * base = &SDHC[host->instance];
    cardData = host->currentData;

    blockCount = cardData->blockCount;
    while (SDHC_GetPresentState(base) & SDHC_BUFF_READ_ENABLED)
    {
        blockSize = cardData->blockSize;
        while (blockSize)
        {
            cardData->buffer[cardData->bytesTransferred >> 2] = SDHC_GetData(base);
            cardData->bytesTransferred += 4;
            blockSize -= 4;
        }
        blockCount--;
        if (!blockCount)
        {
            break;
        }
    }
}

/*FUNCTION****************************************************************
 *
 * Function Name: SDMMC_PioWriteBlock
 * Description: Write a block using PIO
 *
 *END*********************************************************************/
static void SDMMC_PioWriteBlock(host_t * host)
{
    uint32_t blockSize, blockCount;
    card_data_t *cardData = host->currentData;
    SDHC_Type * base = &SDHC[host->instance];

    blockCount = cardData->blockCount;
    while (SDHC_GetPresentState(base) & SDHC_BUFF_WRITE_ENABLED)
    {
        blockSize = cardData->blockSize;
        while (blockSize)
        {
            SDHC_SetData(base, cardData->buffer[cardData->bytesTransferred >> 2]);
            cardData->bytesTransferred += 4;
            blockSize -= 4;
        }
        blockCount--;
        if (!blockCount)
        {
            break;
        }
    }
}

#if ! defined FSL_CARD_DRIVER_USING_IRQ
/*FUNCTION****************************************************************
 *
 * Function Name: SDMMC_WaitInt
 * Description: Wait for specific interrupts
 *
 *END*********************************************************************/
static sdmmc_status_t SDMMC_WaitInt(host_t * host, uint32_t mask, uint32_t *irq, uint32_t timeoutInMs)
{
    sdmmc_status_t status = kStatus_SDMMC_NoError;
    uint32_t startTime, currentTime, elapsedTime = 0;
    assert(timeoutInMs <= FSL_OSA_TIME_RANGE);
    SDHC_Type * base = &SDHC[host->instance];
    do
    {
        startTime = OSA_TimeGetMsec();
        *irq = (SDHC_GetIntFlags(base) & mask);
        if (*irq)
        {
            break;
        }
        currentTime = OSA_TimeGetMsec();
        if (currentTime < startTime)
        {
            currentTime += FSL_OSA_TIME_RANGE;
        }
        elapsedTime += currentTime - startTime;
    }
    while (elapsedTime < timeoutInMs);

    if (!(*irq))
    {
        status = kStatus_SDMMC_TimeoutError;
    }

    return status;
}

/*FUNCTION****************************************************************
 *
 * Function Name: SDMMC_TransferDataPio
 * Description: transfer data using PIO mode
 *
 *END*********************************************************************/
static sdmmc_status_t SDMMC_TransferDataPio(host_t * host, uint32_t timeoutInMs)
{
    uint32_t opMask, mask, i, j, irqFlags, status;
    assert(host);
    SDHC_Type* base = &SDHC[host->instance];
    card_data_t *cardData = host->currentData;

    mask = SDHC_DATA_COMPLETE_INT | SDHC_DATA_ERR_INT;
    if ((cardData->flags & CARD_DATA_FLAGS_DATA_READ))
    {
        opMask = SDHC_BUF_READ_READY_INT;
    }
    else
    {
        opMask = SDHC_BUF_WRITE_READY_INT;
    }
    for (i = 0; i < cardData->blockCount; i++)
    {
        status = SDMMC_WaitInt(host, mask | opMask, &irqFlags, timeoutInMs);
        if (status != kStatus_SDMMC_NoError)
        {
            cardData->errors |= CARD_DATA_ERR_DATA_TIMEOUT;
            host->currentData = 0;
            SDMMC_SetClock(host->instance, false);
            SDMMC_SetCmdDataError(cardData, irqFlags);
            return kStatus_SDMMC_Failed;
        }
        if (irqFlags & SDHC_DATA_ERR_INT)
        {
            SDHC_ClearIntFlags(base, mask);
            host->currentData = 0;
            SDMMC_SetClock(host->instance, false);
            SDMMC_SetCmdDataError(cardData, irqFlags);
            return kStatus_SDMMC_Failed;
        }
        if (irqFlags & opMask)
        {
            if ((cardData->flags & CARD_DATA_FLAGS_DATA_READ))
            {
                SDMMC_PioReadBlock(host);
            }
            else
            {
                SDMMC_PioWriteBlock(host);
            }
            SDHC_ClearIntFlags(base, opMask);
        }
    }

    do
    {
        status = SDMMC_WaitInt(host, mask, &irqFlags, timeoutInMs);
        if (status != kStatus_SDMMC_NoError)
        {
            cardData->errors |= CARD_DATA_ERR_DATA_TIMEOUT;
            host->currentData = 0;
            SDMMC_SetClock(host->instance, false);
            SDMMC_SetCmdDataError(cardData, irqFlags);
            return kStatus_SDMMC_Failed;
        }
    } while (!(irqFlags & SDHC_DATA_COMPLETE_INT));

    SDHC_ClearIntFlags(base, mask);
    return kStatus_SDMMC_NoError;
}

/*FUNCTION****************************************************************
 *
 * Function Name: SDMMC_TransferDataDma
 * Description: transfer data using DMA mode
 *
 *END*********************************************************************/
static sdmmc_status_t SDMMC_TransferDataDma(host_t * host, uint32_t timeoutInMs)
{
    uint32_t mask, irqFlags;
    sdmmc_status_t status;
    card_data_t *cardData = host->currentData;
    SDHC_Type *base = &SDHC[host->instance];

    if (host->transferMode == kSdmmcHostTransModeSdma)
    {
        return kStatus_SDMMC_NotSupportYet;
    }

    mask = SDHC_DATA_COMPLETE_INT | SDHC_DMA_ERR_INT;
    do
    {
        status = SDMMC_WaitInt(host, mask, &irqFlags, timeoutInMs);
        if (status != kStatus_SDMMC_NoError)
        {
            cardData->errors |= CARD_DATA_ERR_DATA_TIMEOUT;
            host->currentData = 0;
            SDMMC_SetClock(host->instance, false);
            SDMMC_SetCmdDataError(cardData, irqFlags);
            return kStatus_SDMMC_Failed;
        }

        if (irqFlags & SDHC_DMA_ERR_INT)
        {
            cardData->errors |= CARD_DATA_ERR_DMA;
            host->currentData = 0;
            SDMMC_SetClock(host->instance, false);
            SDMMC_SetCmdDataError(cardData, irqFlags);
            return kStatus_SDMMC_Failed;
        }
    } while (!(irqFlags & SDHC_DATA_COMPLETE_INT));

    SDHC_ClearIntFlags(base, mask);
    return kStatus_SDMMC_NoError;
}

/*FUNCTION****************************************************************
 *
 * Function Name: SDMMC_TransferData
 * Description: transfer data using different mode according to the flags
 * of host controller
 *
 *END*********************************************************************/
/*static sdmmc_status_t SDMMC_TransferData(host_t * host, uint32_t timeoutInMs)
{
    host_request_t * req = host->currentReq;
    if (req->flags & HOST_REQ_FLAGS_USE_DMA)
    {
        return SDMMC_TransferDataDma(host, timeoutInMs);
    }
    else
    {
        return SDMMC_TransferDataPio(host, timeoutInMs);
    }
}*/

#else /* FSL_CARD_DRIVER_USING_IRQ */
/*FUNCTION****************************************************************
 *
 * Function Name: SDMMC_ClearSetInt
 * Description: Clear then set corresponding interrupt mask
 *
 *END*********************************************************************/
static void SDMMC_ClearSetInt(host_t * host, uint32_t clear, uint32_t set)
{
    assert(host);
    SDHC_Type * base = &SDHC[host->instance];

    SDHC_SetIntState(base, false, clear);
    SDHC_SetIntSignal(base, false, clear);

    SDHC_SetIntState(base, true, set);
    SDHC_SetIntSignal(base, true, set);
}
/*FUNCTION****************************************************************
 *
 * Function Name: SDMMC_DataIrq
 * Description: handle data related irqs
 *
 *END*********************************************************************/
static void SDMMC_DataIrq(host_t * host, uint32_t irq)
{
    card_data_t *cardData = host->currentData;
    assert(irq & SDHC_DATA_ALL_INT);
    assert(cardData);
    assert(cardData->buffer);

    if (irq & (SDHC_DATA_ERR_INT | SDHC_DMA_ERR_INT))
    {
        SDMMC_SetCmdDataError(cardData, irq);
        OSA_SemaPost(cardData->complete);
        return;
    }

    if (irq & SDHC_BUF_READ_READY_INT)
    {
        SDMMC_PioReadBlock(host);
    }
    else if (irq & SDHC_BUF_WRITE_READY_INT)
    {
        SDMMC_PioWriteBlock(host);
    }
    else if (irq & SDHC_DATA_COMPLETE_INT)
    {
        OSA_SemaPost(cardData->complete);
    }
    else if (irq & SDHC_DMA_INT)
    {
        if (host->transferMode != kSdmmcHostTransModeSdma)
        {
            return;
        }
    }
}

/*FUNCTION****************************************************************
 *
 * Function Name: SDMMC_CmdIrq
 * Description: handle command related irqs
 *
 *END*********************************************************************/
static void SDMMC_CmdIrq(host_t * host, uint32_t irq)
{
    uint32_t i;
    card_cmd_t *cardCmd = host->currentCmd;
    SDHC_Type * base = &SDHC[host->instance];
    assert(cardCmd);
    assert(irq & SDHC_CMD_ALL_INT);

    if (irq & SDHC_CMD_ERR_INT)
    {
        SDMMC_SetCmdError(cardCmd, irq);
    }
    else if (irq & SDHC_CMD_COMPLETE_INT)
    {
        if (cardCmd->respType != kSdmmcRespTypeNone)
        {
            cardCmd->response[0] = SDHC_GetCardResponse(base, 0);
            /*if (cardCmd->respType != kSdmmcRespTypeR2)
            {
                if ((cardCmd->respType == kSdhcRespTypeR1) || (cardCmd->respType == kSdhcRespTypeR1b))
                {
                    req->cardErrStatus = SDMMC_R1_ERROR_BITS(req->response[0]);
                }
            }
            else*/
            if (cardCmd->respType == kSdmmcRespTypeR2)
            {
                cardCmd->response[1] = SDHC_GetCardResponse(base, 1);
                cardCmd->response[2] = SDHC_GetCardResponse(base, 2);
                cardCmd->response[3] = SDHC_GetCardResponse(base, 3);
                i = 4;
                do {
                    cardCmd->response[i - 1] <<= 8;
                    if (i > 1)
                    {
                        cardCmd->response[i - 1] |=
                            ((cardCmd->response[i-2] & 0xFF000000U) >> 24);
                    }
                } while(i--);
            }
        }
    }
    //if ((!req->data) || (req->cardErrStatus))

    //{
        OSA_SemaPost(cardCmd->complete);
    //}
}

/*FUNCTION****************************************************************
 *
 * Function Name: SDMMC_CardDetectIrq
 * Description: Card detection interrupt handler
 *
 *END*********************************************************************/
static void SDMMC_CardDetectIrq(host_t * host, uint32_t irq)
{
    assert(irq & SDHC_CD_ALL_INT);
    assert(host->cardDetectCallback);

    if ((irq & SDHC_CD_ALL_INT) == SDHC_CARD_INSERTION_INT)
    {
        if (host->cardDetectCallback)
        {
            host->cardDetectCallback(host->instance, true);
        }
        SDMMC_ClearSetInt(host, SDHC_CARD_INSERTION_INT, SDHC_CARD_REMOVAL_INT);
    }
    else
    {
        if (host->cardDetectCallback)
        {
            host->cardDetectCallback(host->instance, false);
        }
        SDMMC_ClearSetInt(host, SDHC_CARD_REMOVAL_INT, SDHC_CARD_INSERTION_INT);
    }
}

/*FUNCTION****************************************************************
 *
 * Function Name: SDMMC_BlockGapIrq
 * Description: Block gap interrupt handler
 *
 *END*********************************************************************/
static void SDMMC_BlockGapIrq(host_t * host)
{    assert(host->blockGapCallback);
    host->blockGapCallback(host->instance);
}

/*FUNCTION****************************************************************
 *
 * Function Name: SDMMC_CardIntIrq
 * Description: Card interrupt handler
 *
 *END*********************************************************************/
static void SDMMC_CardIntIrq(host_t * host)
{
    assert(host->cardIntCallback);
    host->cardIntCallback(host->instance);
}
/*FUNCTION****************************************************************
 *
 * Function Name: SDMMC_IrqHandler
 * Description: IRQ handler
 *
 *END*********************************************************************/
void SDMMC_IrqHandler(host_t * host)
{
    volatile uint32_t irq;
    volatile uint32_t cardInt = 0;
    SDHC_Type *base = &SDHC[host->instance];
    irq = SDHC_GetIntFlags(base);
    assert(host);
    if (!irq)
    {
        return;
    }

    if (irq & SDHC_CD_ALL_INT)
    {
        SDMMC_CardDetectIrq(host, (irq & SDHC_CD_ALL_INT));
    }
    if (irq & SDHC_CMD_ALL_INT)
    {
        SDMMC_CmdIrq(host, (irq & SDHC_CMD_ALL_INT));
    }
    if (irq & SDHC_DATA_ALL_INT)
    {
        SDMMC_DataIrq(host, (irq & SDHC_DATA_ALL_INT));
    }
    if (irq & SDHC_CARD_INT)
    {
        cardInt = 1;
    }
    if (irq & SDHC_BLOCK_GAP_EVENT_INT)
    {
        SDMMC_BlockGapIrq(host);
    }

    SDHC_ClearIntFlags(base, irq);

    if (cardInt)
    {
        SDMMC_CardIntIrq(host);
    }
    return;
}
#endif

/*!
* @brief Issues the request on a specific host controller and returns immediately.
*
* This function sents the command to the card on a specific SDHC.
* The command is sent and host will not wait the command response from the card.
* Command response and read/write data operation will be done in ISR instead of
* in this function.
*
* @param base SDHC base address
* @param host the host state inforamtion
* @return kStatus_SDMMC_NoError on success
*/
/*sdmmc_status_t SDMMC_IssueRequestBlocking(host_t * host, uint32_t timeoutInMs)
{
    sdmmc_status_t ret;
    host_request_t req;
    SDHC_Type* base = &SDHC[host->instance];
    assert(host);
    ret = kStatus_SDMMC_NoError;
    req->error = 0;

    // Poll busy signal line to wait until last time sdhc send operation complete 
    while(!(SDHC_GetPresentState(base) & SDHC_DATA0_LINE_LEVEL)){}
    // SDHC minimum data length is 32-bit DATA-PORT or ADMA1 4 bytes align 
    if ((req->data) && (req->data->blockSize % 4))
    {
        return kStatus_SDMMC_BlockSizeNotSupportError;
    }
    if ((req->data) && (host->transferMode != kSdhcTransModePio))
    {
        if (kStatus_SDMMC_NoError == SDMMC_PrepareDmaData(host))
        {
            req->flags |= kSdmmcHostTransModePio;
        }
    }

#if defined FSL_CARD_DRIVER_USING_IRQ
    osa_status_t status;
#if defined FSL_CARD_DRIVER_USING_DYNALLOC
    semaphore_t *complete = (semaphore_t *)OSA_MemAllocZero(sizeof(semaphore_t));
    if (kStatus_OSA_Success != OSA_SemaCreate(complete, 0))
    {
        return kStatus_SDMMC_Failed;
    }
    assert(!req->complete);         // it should not be asigned outside of this routine 
    req->complete = complete;
#else
    semaphore_t complete = {0};
    if (kStatus_OSA_Success != OSA_SemaCreate(&complete, 0))
    {
        return kStatus_SDMMC_Failed;
    }
    req->complete = &complete;
#endif
#endif

    SDMMC_SetClock(host, true);

    if (host->currentReq)
    {
        req->error |= HOST_REQ_ERR_HOST_BUSY;
        SDMMC_SetClock(host->instance, false);
#if defined FSL_CARD_DRIVER_USING_IRQ
        OSA_SemaDestroy(req->complete);
#if defined FSL_CARD_DRIVER_USING_DYNALLOC
        OSA_MemFree(req->complete);
#endif
        req->complete = NULL;
#endif
        return kStatus_SDMMC_HostIsBusyError;
    }

    host->currentReq = req;

    if (kStatus_SDMMC_NoError != SDMMC_SendCommand(host))
    {
        host->currentReq = 0;
        SDMMC_SetClock(host->instance, false);
        req->error |= HOST_REQ_ERR_SEND_CMD;
#if defined FSL_CARD_DRIVER_USING_IRQ
        OSA_SemaDestroy(req->complete);
#if defined FSL_CARD_DRIVER_USING_DYNALLOC
        OSA_MemFree(req->complete);
#endif
        req->complete = NULL;
#endif
        return kStatus_SDMMC_Failed;
    }

#if defined FSL_CARD_DRIVER_USING_IRQ
    do
    {
        if (!timeoutInMs)
        {
            status = OSA_SemaWait(req->complete, OSA_WAIT_FOREVER);
        }
        else
        {
            status = OSA_SemaWait(req->complete, timeoutInMs);
        }
    } while (status == kStatus_OSA_Idle);

    if (status != kStatus_OSA_Success)
    {
        req->error |= HOST_REQ_ERR_TIMEOUT;
    }

    OSA_SemaDestroy(req->complete);
#if defined FSL_CARD_DRIVER_USING_DYNALLOC
    OSA_MemFree(req->complete);
#endif
    req->complete = NULL;
#else //  FSL_CARD_DRIVER_USING_IRQ 
    uint32_t mask = 0, irqFlags = 0, i;
    mask = SDHC_CMD_COMPLETE_INT | SDHC_CMD_ERR_INT;
    if (kStatus_SDMMC_NoError != SDMMC_WaitInt(host, mask, &irqFlags, timeoutInMs))
    {
        host->currentReq = 0;
        SDMMC_SetClock(host->instance, false);
        SDMMC_SetCmdError(req, irqFlags);
        return kStatus_SDMMC_Failed;
    }

    if (irqFlags != SDHC_CMD_COMPLETE_INT)
    {
        SDHC_ClearIntFlags(base, mask);
        host->currentReq = 0;
        SDMMC_SetClock(host->instance, false);
        SDMMC_SetCmdError(req, irqFlags);
        return kStatus_SDMMC_Failed;
    }

    SDHC_ClearIntFlags(base, SDHC_CMD_COMPLETE_INT);
    if (host->respType != kSdmmcRespTypeNone)
    {
        req->response[0] = SDHC_GetCardResponse(base, 0);
        if (host->respType != kSdmmcRespTypeR2)
        {
            if ((req->respType == kSdhcRespTypeR1) ||
                    (req->respType == kSdhcRespTypeR1b))
            {
                req->cardErrStatus = SDMMC_R1_ERROR_BITS(req->response[0]);
            }
        }
        else
        {
            req->response[1] = SDHC_GetCardResponse(base, 1);
            req->response[2] = SDHC_GetCardResponse(base, 2);
            req->response[3] = SDHC_GetCardResponse(base, 3);
            i = 4;
            //R3-R2-R1-R0[lowest 8 bit is invalid bit] has the same format as spec R2 format after removed internal CRC7 and end bit. 
            do {
                req->response[i - 1] <<= 8;
                if (i > 1)
                {
                    req->response[i - 1] |= ((req->response[i-2] & 0xFF000000U) >> 24);
                }
            } while(i--);
        }
    }

    if ((!req->cardErrStatus) && (req->data))
    {
        ret = SDMMC_TransferData(host, timeoutInMs);
    }
#endif // ! FSL_CARD_DRIVER_USING_IRQ 

    if (req->cardErrStatus)
    {
        ret = kStatus_SDMMC_RequestCardStatusError;
    }

    if (req->error)
    {
        ret = kStatus_SDMMC_RequestFailed;
    }

    host->currentReq = 0;
    SDMMC_SetClock(host->instance, false);
    return ret;
}*/

sdmmc_status_t SDMMC_SendCmdBlocking(host_t *host, uint32_t timeoutInMs)
{
    sdmmc_status_t ret;
    SDHC_Type* base = &SDHC[host->instance];
    card_cmd_t *cardCmd = host->currentCmd;
    card_data_t *cardData = host->currentData;;
    assert(host);
    assert(cardCmd);

    ret = kStatus_SDMMC_NoError;
    cardCmd->errors = 0;

    SDMMC_SetClock(host->instance, true);
    
    /* Poll busy signal line to wait until last time sdhc send operation complete.
    Maily used in the multiple blocks read/write operation. Wait the busy signal  
    be changed after polling or get the data complete interrupt flag. */
    while(!(SDHC_GetPresentState(base) & SDHC_DATA0_LINE_LEVEL)){}
    
    /*if (host->currentCmd || host->currentData)
    {
        return kStatus_SDMMC_HostIsBusyError;
    }

    host->currentCmd = cardCmd;
    host->currentData = cardData;*/

    /* SDHC minimum data length is 32-bit DATA-PORT or ADMA1 4 bytes align */
    if ((cardData) && (cardData->blockSize % 4))
    {
        return kStatus_SDMMC_BlockSizeHostNotSupport;
    }

    if ((cardData) && (host->transferMode != kSdmmcHostTransModePio))
    {
        if (kStatus_SDMMC_NoError == SDMMC_PrepareDmaData(host))
        {
            cardData->flags |= CARD_DATA_FLAGS_USE_DMA;
        }
    }

#if defined FSL_CARD_DRIVER_USING_IRQ
    osa_status_t status;
/* Allocates semaphore for command. */
#if defined FSL_CARD_DRIVER_USING_DYNALLOC
    semaphore_t *cmdComplete = (semaphore_t *)OSA_MemAllocZero(sizeof(semaphore_t));
    if (kStatus_OSA_Success != OSA_SemaCreate(cmdComplete, 0))
    {
        return kStatus_SDMMC_Failed;
    }
    assert(!cardCmd->complete);         /* it should not be asigned outside of this routine */
    cardCmd->complete = cmdComplete;
#else
    //semaphore_t cmdComplete = {0};
    if (kStatus_OSA_Success != OSA_SemaCreate(&g_cmdComplete, 0))
    {
        return kStatus_SDMMC_Failed;
    }
    cardCmd->complete = &g_cmdComplete;
#endif
    if (cardData)
    {
        /* Allocates semaphore for command data if command has data. */
    #if defined FSL_CARD_DRIVER_USING_DYNALLOC
        semaphore_t *dataComplete = (semaphore_t *)OSA_MemAllocZero(sizeof(semaphore_t));
        if (kStatus_OSA_Success != OSA_SemaCreate(dataComplete, 0))
        {
            return kStatus_SDMMC_Failed;
        }
        assert(!cardData->complete);         /* it should not be asigned outside of this routine */
        cardData->complete = dataComplete;
    #else
        //semaphore_t dataComplete = {0};
        if (kStatus_OSA_Success != OSA_SemaCreate(&g_dataComplete, 0))
        {
            return kStatus_SDMMC_Failed;
        }
        cardData->complete = &g_dataComplete;
    #endif
    }

#endif

    //SDMMC_SetClock(host->instance, true);

    /*if (host->currentReq)
    {
        req->error |= HOST_REQ_ERR_HOST_BUSY;
        SDMMC_SetClock(host->instance, false);
#if defined FSL_CARD_DRIVER_USING_IRQ
        OSA_SemaDestroy(req->complete);
#if defined FSL_CARD_DRIVER_USING_DYNALLOC
        OSA_MemFree(req->complete);
#endif
        req->complete = NULL;
#endif
        return kStatus_SDMMC_HostIsBusyError;
    }*/

    if (kStatus_SDMMC_NoError != SDMMC_SendCommand(host))
    {
        //host->currentReq = 0;
        SDMMC_SetClock(host->instance, false);
        cardCmd->errors |= CARD_CMD_ERR_SEND_CMD;
#if defined FSL_CARD_DRIVER_USING_IRQ
        OSA_SemaDestroy(cardCmd->complete);
        OSA_SemaDestroy(cardData->complete);
#if defined FSL_CARD_DRIVER_USING_DYNALLOC
        OSA_MemFree(cardCmd->complete);
        OSA_MemFree(cardData->complete);
#endif
        cardCmd->complete = NULL;
        cardData->complete = NULL;
#endif
        return kStatus_SDMMC_Failed;
    }

#if defined FSL_CARD_DRIVER_USING_IRQ
    do
    {
        if (!timeoutInMs)
        {
            status = OSA_SemaWait(cardCmd->complete, OSA_WAIT_FOREVER);
        }
        else
        {
            status = OSA_SemaWait(cardCmd->complete, timeoutInMs);
        }
    } while (status == kStatus_OSA_Idle);

    if (status != kStatus_OSA_Success)
    {
        cardCmd->errors |= CARD_CMD_ERR_CMD_TIMEOUT;
        /* command data will not be received. */
        OSA_SemaDestroy(cardData->complete);
#if defined FSL_CARD_DRIVER_USING_DYNALLOC
        OSA_MemFree(cardData->complete);
#endif
        cardData->complete = NULL;
    }

    OSA_SemaDestroy(cardCmd->complete);
#if defined FSL_CARD_DRIVER_USING_DYNALLOC
    OSA_MemFree(cardCmd->complete);
#endif
    cardCmd->complete = NULL;
#else /* FSL_CARD_DRIVER_USING_IRQ */
    uint32_t mask = 0, irqFlags = 0, i;
    mask = SDHC_CMD_COMPLETE_INT | SDHC_CMD_ERR_INT;
    if (kStatus_SDMMC_NoError != SDMMC_WaitInt(host, mask, &irqFlags, timeoutInMs))
    {
        SDHC_ClearIntFlags(base, mask);
        host->currentCmd = 0;
        host->currentData = 0;
        SDMMC_SetClock(host->instance, false);
        SDMMC_SetCmdError(cardCmd, irqFlags);
        return kStatus_SDMMC_Failed;
    }
    if (irqFlags != SDHC_CMD_COMPLETE_INT)
    {
        SDHC_ClearIntFlags(base, mask);
        host->currentCmd = 0;
        host->currentData = 0;
        SDMMC_SetClock(host->instance, false);
        SDMMC_SetCmdError(cardCmd, irqFlags);
        return kStatus_SDMMC_Failed;
    }
    SDHC_ClearIntFlags(base, SDHC_CMD_COMPLETE_INT);

    if (cardCmd->respType != kSdmmcRespTypeNone)
    {
        cardCmd->response[0] = SDHC_GetCardResponse(base, 0);
        /*if (cardCmd->respType != kSdmmcRespTypeR2)
        {
            if ((req->respType == kSdhcRespTypeR1) ||
                    (req->respType == kSdhcRespTypeR1b))
            {
                req->cardErrStatus = SDMMC_R1_ERROR_BITS(req->response[0]);
            }
        }*/
        if (cardCmd->respType == kSdmmcRespTypeR2)
        {
            cardCmd->response[1] = SDHC_GetCardResponse(base, 1);
            cardCmd->response[2] = SDHC_GetCardResponse(base, 2);
            cardCmd->response[3] = SDHC_GetCardResponse(base, 3);
            i = 4;
            /* R3-R2-R1-R0[lowest 8 bit is invalid bit] has the same format as spec R2 format after removed internal CRC7 and end bit. */
            do {
                cardCmd->response[i - 1] <<= 8;
                if (i > 1)
                {
                    cardCmd->response[i - 1] |= ((cardCmd->response[i-2] & 0xFF000000U) >> 24);
                }
            } while(i--);
        }
    }
    /*if ((!req->cardErrStatus) && (req->data))
    {
        ret = SDMMC_TransferData(host, timeoutInMs);
    }*/
#endif /* ! FSL_CARD_DRIVER_USING_IRQ */
    /*if (req->cardErrStatus)
    {
        ret = kStatus_SDMMC_RequestCardStatusError;
    }*/
    if (cardCmd->errors)
    {
        ret = kStatus_SDMMC_SendCardCmdFailed;
    }

    host->currentCmd = 0;
    if (!cardData)
    {
        SDMMC_SetClock(host->instance, false);
    }
    return ret;
}
sdmmc_status_t SDMMC_CheckR1Response(card_cmd_t *cardCmd)
{
   // if ((cardCmd->respType == kSdhcRespTypeR1) || (cardCmd->respType == kSdhcRespTypeR1b))
    //{
        if (SDMMC_R1_ERROR_BITS(cardCmd->response[0]))
        {
            return kStatus_SDMMC_CardStatusError;
        }
    //}
    return kStatus_SDMMC_NoError;
}
sdmmc_status_t SDMMC_WaitDataTransferComplete(host_t *host, uint32_t timeoutInMs)
{
    card_data_t *cardData = host->currentData;
    osa_status_t status;
    assert(host);
    assert(cardData);
    sdmmc_status_t ret = kStatus_SDMMC_NoError;
    
#if defined FSL_CARD_DRIVER_USING_IRQ
    assert(cardData->complete);
    do
    {
        if (!timeoutInMs)
        {
            status = OSA_SemaWait(cardData->complete, OSA_WAIT_FOREVER);
        }
        else
        {
            status = OSA_SemaWait(cardData->complete, timeoutInMs);
        }
    } while (status == kStatus_OSA_Idle);

    if (status != kStatus_OSA_Success)
    {
        cardData->errors |= CARD_DATA_ERR_DATA_TIMEOUT;
    }

    OSA_SemaDestroy(cardData->complete);
#if defined FSL_CARD_DRIVER_USING_DYNALLOC
    OSA_MemFree(cardData->complete);
#endif
    cardData->complete = NULL;
#else /* FSL_CARD_DRIVER_USING_IRQ */
    if (cardData->flags & CARD_DATA_FLAGS_USE_DMA)
    {
        SDMMC_TransferDataDma(host, timeoutInMs);
    }
    else
    {
        SDMMC_TransferDataPio(host, timeoutInMs);
    }
#endif

    if (cardData->errors)
    {
        ret = kStatus_SDMMC_TransferCardDataFailed;
    }

    host->currentData = 0;
    SDMMC_SetClock(host->instance, false);
    return ret;
}

/*FUNCTION****************************************************************
 *
 * Function Name: SDMMC_DetectCard
 * Description: check whether the card is present on specified host
 *      controller.
 *
 *END*********************************************************************/
sdmmc_status_t SDMMC_DetectCard(host_t *host)
{
    assert(host->instance < SDHC_INSTANCE_COUNT);
    SDHC_Type *base = &SDHC[host->instance];
    if (host->cardDetectMode == kSdmmcHostCardDetectGpio)
    {
        return kStatus_SDMMC_CardDetectNotSupportYet;
    }

    SDMMC_SetClock(host->instance, true);
    if (!(SDHC_GetPresentState(base) & CARD_CMD_ERR_SEND_CMD))
    {
        host->flags &= (uint32_t)(~HOST_FLAGS_CARD_PRESENTED);
        SDMMC_SetClock(host->instance, false);
        return kStatus_SDMMC_NoCardInsertedError;
    }
    host->flags |= HOST_FLAGS_CARD_PRESENTED;
    SDHC_SetCardActive(base, 100);
    SDMMC_SetClock(host->instance, false);
    return kStatus_SDMMC_NoError;
}
sdmmc_status_t SDMMC_ConfigClock(host_t *host, uint32_t destClock)
{
    assert(host);
    uint32_t baseClock;
    baseClock = CLOCK_SYS_GetSdhcFreq(host->instance);
    sdhc_sd_clock_config_t sdClockConfig;
    sdClockConfig.sdClockEnable = true;
    sdClockConfig.baseClockFreq = baseClock;
    sdClockConfig.sdClockFreq = destClock;
    if (destClock > baseClock)
    {
        return kStatus_SDMMC_SetCardBusClockFailed;
    }
    SDMMC_SetClock(host->instance, true);
    SDHC_SetSdClock(&SDHC[host->instance], &sdClockConfig);
    SDMMC_SetClock(host->instance, false);
    return kStatus_SDMMC_NoError;
}
sdmmc_status_t SDMMC_SetHostBusWidth(host_t *host, sdhc_dtw_t busWidth)
{
    SDMMC_SetClock(host->instance, true);
    SDHC_SetDataTransferWidth(&SDHC[host->instance], busWidth);
    SDMMC_SetClock(host->instance, false);
    return kStatus_SDMMC_NoError;
}