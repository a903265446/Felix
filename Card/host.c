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
 #include <assert.h>
 #include "card.h"


/* Following part is the common function between SD card and MMC card */

#if !defined FSL_CARD_DRIVER_USING_DYNALLOC
host_request_t g_currentReq;
#define FSL_HOST_ADMA_TABLE_MAX_ENTRY       16
static uint32_t g_AdmaTableAddress[SDHC_INSTANCE_COUNT][FSL_HOST_ADMA_TABLE_MAX_ENTRY >> 1];
sdmmc_status_t SDMMC_SetStaticAllocMemory(host_t * host)
{
    host->admaTableAddress = g_AdmaTableAddress;
    SDHC_SetAdmaAddress(base, (uint32_t)host->admaTableAddress);
}
#endif

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

/*FUNCTION****************************************************************
 *
 * Function Name: SDMMC_PrepareDmaData
 * Description: Prepare data for transferring
 *
 *END*********************************************************************/
static sdmmc_status_t SDMMC_PrepareDmaData(host_t * host) 
{
    sdmmc_status_t ret;
    uint32_t totalSize, entries;
    volatile sdhc_host_t * host;
    host_request_t * req;
#if defined FSL_CARD_DRIVER_USING_DYNALLOC
    sdhc_adma1_descriptor_t * adma1TableAddress = NULL;
    sdhc_adma2_descriptor_t * adma2TableAddress = NULL;
#endif
    SDHC_Type* base = SDHC[host->hostInstance];

    assert(req);
    assert(req->data);
    assert(req->data->buffer);
    assert(req->data->blockCount);
    assert(req->data->blockSize);

    ret = kStatus_SDMMC_NoError;
    req = host->currentReq;

    if ((host->mode != kSdhcTransModeAdma2)
#if defined BSP_HOST_ENABLE_ADMA1
            && (host->mode != kSdhcTransModeAdma1)
#endif
            )
    {
        return ret;
    }

    totalSize = (req->data->blockSize * req->data->blockCount);

#if defined FSL_CARD_DRIVER_USING_DYNALLOC
    
    if ((host->mode == kSdhcTransModeAdma2))
    {
        /* ADMA2 */
        entries = SDHC_GetAdmaEntryNumber(base, totalSize);
        if (entries > host->admaTableMaxEntries)
        {
            /* Larger table is needed */
            if (host->admaTableAddress)
            {
                OSA_MemFree(host->admaTableAddress);
                host->admaTableAddress = NULL;
                host->admaTableMaxEntries = 0;
            }
            adma2TableAddress = (sdhc_adma2_descriptor_t *)OSA_MemAllocZero(entries * sizeof(sdhc_adma2_descriptor_t));
        }
        if ((adma2TableAddress == NULL) && (host->admaTableAddress == NULL))
        {
            host->admaTableMaxEntries = 0;
            /* Failed to alloc memory for ADMA descriptor table */
            return kStatus_SDMMC_DmaAddressError;
        }
        if (host->admaTableAddress == NULL)
        {
            /* Update ADMA table address */
            host->admaTableAddress = (uint32_t *)adma2TableAddress;
            /* Update ADMA table capacity */
            host->admaTableMaxEntries = entries;
        }
    }
    else
    {
        /* ADMA1 */
        entries = SDHC_GetAdmaEntryNumber(base, totalSize);
        if (entries > host->admaTableMaxEntries)
        {
            /* Larger table is needed */
            if (host->admaTableAddress)
            {
                OSA_MemFree(host->admaTableAddress);
                host->admaTableAddress = NULL;
                host->admaTableMaxEntries = 0;
            }
            adma1TableAddress = (sdhc_adma1_descriptor_t *)OSA_MemAllocZero(entries * sizeof(sdhc_adma1_descriptor_t));
            if ((adma1TableAddress == NULL) && (host->admaTableAddress == NULL))
            {
                host->admaTableMaxEntries = 0;
                /* Failed to alloc memory for ADMA descriptor table */
                return kStatus_SDMMC_DmaAddressError;
            }
        }
        if (host->admaTableAddress == NULL)
        {
            /* Update ADMA table address */
            host->admaTableAddress = (uint32_t *)adma1TableAddress;
            /* Update ADMA table capacity */
            host->admaTableMaxEntries = entries;
        }
    }
    
    SDHC_SetAdmaAddress(base, (uint32_t)host->admaTableAddress);
#else
    if (entries > FSL_HOST_ADMA_TABLE_MAX_ENTRY)
    {
        return kStatus_SDMMC_Failed;
    }   
#endif    
    SDHC_SetAdmaTable(base, req->data->buffer, totalSize);
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
    sdhc_card_cmd_config_t cmdConfig;
    sdmmc_status_t ret = kStatus_SDMMC_NoError;
    host_request_t * req;
    SDHC_Type * base = SDHC[host->hostInstance];

    req = host->currentReq;
    if (req->data)
    {
        flags |= SDHC_DATA_PRESENT;

        SDHC_SetIntState(base, false, (SDHC_DMA_ERR_INT | SDHC_DMA_INT | SDHC_BUF_READ_READY_INT | SDHC_BUF_WRITE_READY_INT));
        SDHC_SetIntSignal(base, false, (SDHC_DMA_ERR_INT | SDHC_DMA_INT | SDHC_BUF_READ_READY_INT | SDHC_BUF_WRITE_READY_INT));

        if (req->flags & HOST_REQ_FLAGS_USE_DMA)
        {
            flags |= SDHC_ENABLE_DMA;
            SDHC_SetIntState(base, true, (SDHC_DMA_ERR_INT | SDHC_DMA_INT));
#if defined FSL_CARD_DIVER_USING_IRQ
            SDHC_SetIntSignal(base, true, (SDHC_DMA_ERR_INT | SDHC_DMA_INT));
#endif
        }
        else
        {
            SDHC_SetIntState(base, true, (SDHC_BUF_READ_READY_INT | SDHC_BUF_WRITE_READY_INT));
#if defined FSL_CARD_DIVER_USING_IRQ
            SDHC_SetIntSignal(base, true, (SDHC_BUF_READ_READY_INT | SDHC_BUF_WRITE_READY_INT));
#endif
        }

        if (req->flags & HOST_REQ_FLAGS_DATA_READ)
        {
            flags |= SDHC_ENABLE_DATA_READ;
        }
    }
    /* Defines the flag corresponding to each response type which will be set in each transfer. */
    switch (host->respType)
    {
        case kSdemmcRespTypeNone:
            break;
        case kSdemmcRespTypeR1:
            flags |= (SDHC_RESP_LEN_48 | SDHC_ENABLE_CRC_CHECK | SDHC_ENABLE_INDEX_CHECK);    /* Response 1 */
            break;
        case kSdemmcRespTypeR1b:
            flags |= (SDHC_RESP_LEN_48_BC | SDHC_ENABLE_CRC_CHECK | SDHC_ENABLE_INDEX_CHECK);  /* Response 1 with busy */
            break;
        case kSdemmcRespTypeR2:
            flags |= (SDHC_RESP_LEN_136 | SDHC_ENABLE_CRC_CHECK);     /* Response 2 */
            break;
        case kSdemmcRespTypeR3:
            flags |= (SDHC_RESP_LEN_48);
            break;
        case kSdemmcRespTypeR4:
            flags |= (SDHC_RESP_LEN_48);
            break;
        case kSdemmcRespTypeR5:
            flags |= (SDHC_RESP_LEN_48 | SDHC_ENABLE_CRC_CHECK);/* Response 5 */
            break;
        case kSdemmcRespTypeR5b:
            flags |= (SDHC_RESP_LEN_48_BC | SDHC_ENABLE_CRC_CHECK | SDHC_ENABLE_INDEX_CHECK);  /* Response 5 with busy */
            break;
        case kSdemmcRespTypeR6:
            flags |= (SDHC_RESP_LEN_48 | SDHC_ENABLE_CRC_CHECK | SDHC_ENABLE_INDEX_CHECK);    /* Response 6 */
            break;
        case kSdemmcRespTypeR7:
            flags |= (SDHC_RESP_LEN_48 | SDHC_ENABLE_CRC_CHECK | SDHC_ENABLE_INDEX_CHECK);     /* Response 7 */
            break;
        default:
            break;
    }

    while((SDHC_GetPresentState(base) & SDHC_CMD_INHIBIT)) {}

    if(req->flags & HOST_REQ_FLAGS_STOP_TRANS)
    {
        flags |= SDHC_CMD_TYPE_ABORT;
    }
    else if ((req->data) || (host->respType == kSdemmcRespTypeR1b) || (host->respType == kSdemmcRespTypeR5b))
    {
        while((SDHC_GetPresentState(base) & SDHC_DAT_INHIBIT)) {}
    }

    if (req->data)
    {
        if (req->data->blockCount > 1)
        {
            flags |= SDHC_MULTIPLE_BLOCK;
            flags |= SDHC_ENABLE_BLOCK_COUNT;
#ifdef FSL_CARD_DIVER_ENABLE_HOST_AUTOCMD12
            /* Enable Auto CMD12 */
            flags |= SDHC_ENABLE_AUTO_CMD12;
#endif
        }
        if (req->data->blockCount > SDHC_MAX_BLOCK_COUNT)
        {
            cmdConfig.dataBlockSize  = req->data->blockSize;
            cmdConfig.dataBlockCount = SDHC_MAX_BLOCK_COUNT;
            flags &= ~SDHC_ENABLE_BLOCK_COUNT;
        }
        else
        {
            cmdConfig.dataBlockSize  = req->data->blockSize;
            cmdConfig.dataBlockCount = req->data->blockCount;
        }
    }
    else
    {
        cmdConfig.dataBlockSize  = 0;
        cmdConfig.dataBlockCount = 0;
    }

    cmdConfig.argument  = req->argument;
    cmdConfig.cmdIndex = req->cmdIndex;
    cmdConfig.cmdFlags = flags;
    SDHC_SetCardCommand(base, &cmdConfig);
    return ret;
}

/*FUNCTION****************************************************************
 *
 * Function Name: SDMMC_SetReqeustError
 * Description: Set error flags for a given request according to irq flags
 *
 *END*********************************************************************/
static void SDMMC_SetReqeustError(host_request_t *req, uint32_t irqFlags)
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
    assert(host);
    host_request_t * req;
    SDHC_Type * base = SDHC[host->hostInstance];
    req = host->currentReq;
    blockCount = req->data->blockCount;
    while ((SDHC_GetPresentState(base) & kSdhcHalIsBuffReadEnabled))
    {
        blockSize = req->data->blockSize;
        while (blockSize)
        {
            req->data->buffer[req->data->bytesTransferred >> 2] = SDHC_GetData(base);
            req->data->bytesTransferred += 4;
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
    host_request_t * req;
    SDHC_Type * base = SDHC[host->hostInstance];
    req = host->currentReq;
    blockCount = req->data->blockCount;
    while (SDHC_GetCurState(base, kSdhcHalIsBuffWriteEnabled))
    {
        blockSize = req->data->blockSize;
        while (blockSize)
        {
            SDHC_SetData(base, req->data->buffer[req->data->bytesTransferred >> 2]);
            req->data->bytesTransferred += 4;

            blockSize -= 4;
        }
        blockCount--;
        if (!blockCount)
        {
            break;
        }
    }
}

#if ! defined FSL_CARD_DIVER_USING_IRQ
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
    SDHC_Type * base = SDHC[host->hostInstance];
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

    host_request_t * req;
    SDHC_Type* base = SDHC[host->hostInstance];
    req = host->currentReq;
    mask = SDHC_DATA_COMPLETE_INT | SDHC_DATA_ERR_INT;
    if ((req->flags & HOST_REQ_FLAGS_DATA_READ))
    {
        opMask = SDHC_BUF_READ_READY_INT;
    }
    else
    {
        opMask = SDHC_BUF_WRITE_READY_INT;
    }
    for (i = 0; i < req->data->blockCount; i++)
    {
        status = SDMMC_WaitInt(host, mask | opMask, &irqFlags, timeoutInMs);
        if (status != kStatus_SDMMC_NoError)
        {
            req->error |= HOST_REQ_ERR_TIMEOUT;
            host->currentReq = 0;
            SDMMC_SetClock(instance, false);
            SDMMC_SetReqeustError(req, irqFlags);
            return kStatus_SDMMC_Failed;
        }
        if (irqFlags & SDHC_DATA_ERR_INT)
        {
            SDHC_ClearIntFlags(base, mask);
            host->currentReq = 0;
            SDMMC_SetClock(instance, false);
            SDMMC_SetReqeustError(req, irqFlags);
            return kStatus_SDMMC_Failed;
        }
        if (irqFlags & opMask)
        {
            if ((req->flags & HOST_REQ_FLAGS_DATA_READ))
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
            req->error |= HOST_REQ_ERR_TIMEOUT;
            host->currentReq = 0;
            SDMMC_SetClock(instance, false);
            SDMMC_SetReqeustError(req, irqFlags);
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

    if (host->mode == kSdhcTransModeSdma)
    {
        return kStatus_SDMMC_NotSupportYet;
    }

    mask = SDHC_DATA_COMPLETE_INT | SDHC_DMA_ERR_INT;
    do
    {
        status = SDMMC_WaitInt(host, mask, &irqFlags, timeoutInMs);
        if (status != kStatus_SDMMC_NoError)
        {
            req->error |= HOST_REQ_ERR_TIMEOUT;
            host->currentReq = 0;
            SDMMC_SetClock(instance, false);
            SDMMC_SetReqeustError(req, irqFlags);
            return kStatus_SDMMC_Failed;
        }

        if (irqFlags & SDHC_DMA_ERR_INT)
        {
            req->error |= HOST_REQ_ERR_DMA;
            host->currentReq = 0;
            SDMMC_SetClock(instance, false);
            SDMMC_SetRequestError(req, irqFlags);
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
static sdmmc_status_t SDMMC_TransferData(host_t * host, uint32_t timeoutInMs)
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
}

#else /* FSL_CARD_DIVER_USING_IRQ */
/*FUNCTION****************************************************************
 *
 * Function Name: SDMMC_ClearSetInt
 * Description: Clear then set corresponding interrupt mask
 *
 *END*********************************************************************/
static void SDMMC_ClearSetInt(host_t * host, uint32_t clear, uint32_t set)
{
    assert(host);
    SDHC_Type * base = SDHC[host->hostInstance];

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
    host_request_t *req;
    assert(irq & SDHC_DATA_ALL_INT);

    assert(host);
    req = host->currentReq;
    assert(req);
    assert(req->data);
    assert(req->data->buffer);

    if (irq & (SDHC_DATA_ERR_INT | SDHC_DMA_ERR_INT))
    {
        SDMMC_SetReqeustError(req, irq);
        OSA_SemaPost(req->complete);
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
        OSA_SemaPost(req->complete);
    }
    else if (irq & SDHC_DMA_INT)
    {
        if (host->transferMode != kSdhcTransModeSdma)
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
static void SDMMC_CmdIrq(host_t * host)
{
    host_request_t *req;
    uint32_t i;
    SDHC_Type * base = SDHC[host->hostInstance];
    assert(host);
    assert(irq & SDHC_CMD_ALL_INT);
    req = host->currentReq;
    if (irq & SDHC_CMD_ERR_INT)
    {
        SDMMC_SetReqeustError(req, irq);
    }
    else if (irq & SDHC_CMD_COMPLETE_INT)
    {
        if (host->respType != kSdemmcRespTypeNone)
        {
            req->response[0] = SDHC_GetCardResponse(base, 0);
            if (host->respType != kSdemmcRespTypeR2)
            {
                if ((req->respType == kSdhcRespTypeR1) || (req->respType == kSdhcRespTypeR1b))
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
                do {
                    req->response[i - 1] <<= 8;
                    if (i > 1)
                    {
                        req->response[i - 1] |=
                            ((req->response[i-2] & 0xFF000000U) >> 24);
                    }
                } while(i--);
            }
        }
    }
    if ((!req->data) || (req->cardErrStatus))
    {
        OSA_SemaPost(req->complete);
    }
}

/*FUNCTION****************************************************************
 *
 * Function Name: SDMMC_CardDetectIrq
 * Description: Card detection interrupt handler
 *
 *END*********************************************************************/
static void SDMMC_CardDetectIrq(host_t * host)
{
    assert(irq & SDHC_CD_ALL_INT);
    assert(host->cardDetectCallback);

    if ((irq & SDHC_CD_ALL_INT) == SDHC_CARD_INSERTION_INT)
    {
        if (host->cardDetectCallback)
        {
            host->cardDetectCallback(host->hostInstance, true);
        }
        SDMMC_ClearSetInt(host, SDHC_CARD_INSERTION_INT, SDHC_CARD_REMOVAL_INT);
    }
    else
    {
        if (host->cardDetectCallback)
        {
            host->cardDetectCallback(host->hostInstance, false);
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
    assert(host->blockGapCallback);
    host->blockGapCallback(host->hostInstance);
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
    host->cardIntCallback(host->hostInstance);
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
    irq = SDHC_GetIntFlags(base);

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
static sdmmc_status_t SDMMC_SetClock(host_t * host, bool enable)
{
    return kStatus_SDMMC_NoError;
}
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
sdmmc_status_t SDMMC_IssueRequestBlocking(host_t * host, uint32_t timeoutInMs)
{
    sdmmc_status_t ret;
    host_request_t req;
    SDHC_Type* base = SDHC[host->hostInstance];
    assert(host);
    ret = kStatus_SDMMC_NoError;
    req->error = 0;

    /* Poll busy signal line to wait until last time sdhc send operation complete */
    while(!(SDHC_GetPresentState(base) & SDHC_DATA0_LINE_LEVEL)){}
    /* SDHC minimum data length is 32-bit DATA-PORT or ADMA1 4 bytes align */
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

#if defined FSL_CARD_DIVER_USING_IRQ
    osa_status_t status;
#if defined FSL_CARD_DRIVER_USING_DYNALLOC
    semaphore_t *complete = (semaphore_t *)OSA_MemAllocZero(sizeof(semaphore_t));
    if (kStatus_OSA_Success != OSA_SemaCreate(complete, 0))
    {
        return kStatus_SDMMC_Failed;
    }
    assert(!req->complete);         /* it should not be asigned outside of this routine */
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
        SDMMC_SetClock(instance, false);
#if defined FSL_CARD_DIVER_USING_IRQ
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
        SDMMC_SetClock(instance, false);
        req->error |= HOST_REQ_ERR_SEND_CMD;
#if defined FSL_CARD_DIVER_USING_IRQ
        OSA_SemaDestroy(req->complete);
#if defined FSL_CARD_DRIVER_USING_DYNALLOC
        OSA_MemFree(req->complete);
#endif
        req->complete = NULL;
#endif
        return kStatus_SDMMC_Failed;
    }

#if defined FSL_CARD_DIVER_USING_IRQ
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
#else /* FSL_CARD_DIVER_USING_IRQ */
    uint32_t mask = 0, irqFlags = 0, i;
    mask = SDHC_CMD_COMPLETE_INT | SDHC_CMD_ERR_INT;
    if (kStatus_SDMMC_NoError != SDMMC_WaitInt(host, mask, &irqFlags, timeoutInMs))
    {
        host->currentReq = 0;
        SDMMC_SetClock(instance, false);
        SDMMC_SetReqeustError(req, irqFlags);
        return kStatus_SDMMC_Failed;
    }

    if (irqFlags != SDHC_CMD_COMPLETE_INT)
    {
        SDHC_ClearIntFlags(base, mask);
        host->currentReq = 0;
        SDMMC_SetClock(instance, false);
        SDMMC_SetReqeustError(req, irqFlags);
        return kStatus_SDMMC_Failed;
    }

    SDHC_ClearIntFlags(base, SDHC_CMD_COMPLETE_INT);
    if (host->respType != kSdemmcRespTypeNone)
    {
        req->response[0] = SDHC_GetCardResponse(base, 0);
        if (host->respType != kSdemmcRespTypeR2)
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
            /* R3-R2-R1-R0[lowest 8 bit is invalid bit] has the same format as spec R2 format after removed internal CRC7 and end bit. */
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
#endif /* ! FSL_CARD_DIVER_USING_IRQ */

    if (card->cardErrStatus)
    {
        ret = kStatus_SDMMC_RequestCardStatusError;
    }

    if (req->error)
    {
        ret = kStatus_SDMMC_RequestFailed;
    }

    host->currentReq = 0;
    SDMMC_SetClock(instance, false);
    return ret;
}


