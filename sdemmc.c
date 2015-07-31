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

/*FUNCTION****************************************************************
 *
 * Function Name: SDEMMC_AllSendCid
 * Description: send all_send_cid command
 *
 *END*********************************************************************/
static sdemmc_status_t SDEMMC_AllSendCid(sdhc_card_t *card);
/*FUNCTION****************************************************************
 *
 * Function Name: SDEMMC_SendCsd
 * Description: get csd from card
 *
 *END*********************************************************************/
static sdemmc_status_t SDEMMC_SendCsd(sdhc_card_t *card);

/* Above part is the declaration of the composite function based on SD and MMC card. */

/* Following part is the common function between SD card and MMC card */

/*FUNCTION****************************************************************
 *
 * Function Name: SDEMMC_DelayMsec
 * Description: blocking delay msecond
 *
 *END*********************************************************************/
static void SDEMMC_DelayMsec(uint32_t msec)
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
 * Function Name: SDEMMC_SelectCard
 * Description: select or deselect card
 *
 *END*********************************************************************/
static sdemmc_status_t SDEMMC_SelectCard(sdhc_card_t *card, bool isSelected)
{
    sdhc_request_t *req = 0;
#if ! defined BSP_FSL_SDHC_USING_DYNALLOC
    sdhc_request_t request = {0};
    req = &request;
#endif
    assert(card);

#if defined BSP_FSL_SDHC_USING_DYNALLOC
    req = (sdhc_request_t *)OSA_MemAllocZero(sizeof(sdhc_request_t));
    if (req == NULL)
    {
        return kStatus_SDEMMC_OutOfMemory;
    }
#endif
    req->cmdIndex = kSelectCard;
    if (isSelected)
    {
        req->argument = card->rca << 16;
        req->respType = kSdhcRespTypeR1;
    }
    else
    {
        req->argument = 0;
        req->respType = kSdhcRespTypeNone;
    }
    if (kStatus_SDEMMC_NoError !=
            SDHC_DRV_IssueRequestBlocking(card->hostInstance,
                                          req,
                                          FSL_SDCARD_REQUEST_TIMEOUT))
    {
#if defined BSP_FSL_SDHC_USING_DYNALLOC
        OSA_MemFree(req);
#endif
        req = NULL;
        return kStatus_SDEMMC_RequestFailed;
    }
#if defined BSP_FSL_SDHC_USING_DYNALLOC
    OSA_MemFree(req);
#endif
    req = NULL;
    /* Wait until card to transfer state */
    return kStatus_SDEMMC_NoError;
}

/*FUNCTION****************************************************************
 *
 * Function Name: SDEMMC_GoIdle
 * Description: reset all cards to idle state
 *
 *END*********************************************************************/
static sdemmc_status_t SDEMMC_GoIdle(sdhc_card_t *card)
{
    sdhc_request_t *req = 0;
    sdemmc_status_t err;
#if ! defined BSP_FSL_SDHC_USING_DYNALLOC
    sdhc_request_t request = {0};
    req = &request;
#endif
    assert(card);

#if defined BSP_FSL_SDHC_USING_DYNALLOC
    req = (sdhc_request_t *)OSA_MemAllocZero(sizeof(sdhc_request_t));
    if (req == NULL)
    {
        return kStatus_SDEMMC_OutOfMemory;
    }
#endif
    req->cmdIndex = kGoIdleState;
    err = SDHC_DRV_IssueRequestBlocking(card->hostInstance,
                                        req,
                                        FSL_SDCARD_REQUEST_TIMEOUT);
#if defined BSP_FSL_SDHC_USING_DYNALLOC
    OSA_MemFree(req);
#endif
    req = NULL;
    return err;
}

/*FUNCTION****************************************************************
 *
 * Function Name: SDEMMC_SendStatus
 * Description:  send the sd status
 *
 *END*********************************************************************/
static sdemmc_status_t SDEMMC_SendStatus(sdhc_card_t *card)
{
    sdhc_request_t *req = 0;
    sdemmc_status_t err = kStatus_SDEMMC_NoError;
    uint32_t timeout = 1000;
#if ! defined BSP_FSL_SDHC_USING_DYNALLOC
    sdhc_request_t request = {0};
    req = &request;
#endif
    assert(card);

#if defined BSP_FSL_SDHC_USING_DYNALLOC
    req = (sdhc_request_t *)OSA_MemAllocZero(sizeof(sdhc_request_t));
    if (req == NULL)
    {
        return kStatus_SDEMMC_OutOfMemory;
    }
#endif
    req->cmdIndex = kSendStatus;
    req->argument = card->rca << 16;
    req->respType = kSdhcRespTypeR1;
    do
    {
        if (kStatus_SDEMMC_NoError !=
                SDHC_DRV_IssueRequestBlocking(card->hostInstance,
                                              req,
                                              FSL_SDCARD_REQUEST_TIMEOUT))
        {
#if defined BSP_FSL_SDHC_USING_DYNALLOC
            OSA_MemFree(req);
#endif
            req = NULL;
            return kStatus_SDEMMC_RequestFailed;
        }
        if ((req->response[0] & SDMMC_R1_READY_FOR_DATA)
             && (SDMMC_R1_CURRENT_STATE(req->response[0]) != SDMMC_R1_STATE_PRG))
        {
            break;
        }

        SDEMMC_DelayMsec(1);
    } while(timeout--);

    if (!timeout)
    {
#if defined BSP_FSL_SDHC_USING_DYNALLOC
        OSA_MemFree(req);
#endif
        req = NULL;
        return kStatus_SDEMMC_TimeoutError;
    }

#if defined BSP_FSL_SDHC_USING_DYNALLOC
    OSA_MemFree(req);
#endif
    req = NULL;
    return err;
}

#if ! defined BSP_FSL_SDHC_ENABLE_AUTOCMD12
/*FUNCTION****************************************************************
 *
 * Function Name: SDEMMC_StopTransmission
 * Description:  Send stop transmission command to card to stop ongoing
 * data transferring.
 *
 *END*********************************************************************/
static sdemmc_status_t SDEMMC_StopTransmission(sdhc_card_t *card)
{
    sdhc_request_t *req = 0;
    sdemmc_status_t err = kStatus_SDEMMC_NoError;
#if ! defined BSP_FSL_SDHC_USING_DYNALLOC
    sdhc_request_t request = {0};
    req = &request;
#endif
    assert(card);

#if defined BSP_FSL_SDHC_USING_DYNALLOC
    req = (sdhc_request_t *)OSA_MemAllocZero(sizeof(sdhc_request_t));
    if (req == NULL)
    {
        return kStatus_SDEMMC_OutOfMemory;
    }
#endif

    req->cmdIndex = kStopTransmission;
    req->argument = 0;
    req->respType = kSdhcRespTypeR1b;
    req->data = 0;
    if (kStatus_SDEMMC_NoError !=
            SDHC_DRV_IssueRequestBlocking(card->hostInstance,
                                          req,
                                          FSL_SDCARD_REQUEST_TIMEOUT))
    {
#if defined BSP_FSL_SDHC_USING_DYNALLOC
        OSA_MemFree(req);
#endif
        req = NULL;
        return kStatus_SDEMMC_RequestFailed;
    }

#if defined BSP_FSL_SDHC_USING_DYNALLOC
    OSA_MemFree(req);
#endif
    req = NULL;
    return err;
}
#endif

/*FUNCTION****************************************************************
 *
 * Function Name: SDEMMC_CheckReadOnly
 * Description: Check if the card is ready only
 *
 *END*********************************************************************/
bool SDEMMC_CheckReadOnly(sdhc_card_t *card)
{
    assert(card);
    if(kCardTypeSd == card->cardType)
    {
        
    }
    switch(card->cardType)
    {
    case kCardTypeSd:
      {
          return ((card->csd.sdCsd.flags & SDCARD_CSD_PERM_WRITE_PROTECT) ||
            (card->csd.sdCsd.flags & SDCARD_CSD_TMP_WRITE_PROTECT));         
      }
    case kCardTypeMmc:
      {
          return ((card->csd.mmcCsd.flags & SDCARD_CSD_PERM_WRITE_PROTECT) ||
            (card->csd.mmcCsd.flags & SDCARD_CSD_TMP_WRITE_PROTECT)); 
      }
    default:
      {
          return false;/* Default as non-read only */
      }
    }
    
}

/*FUNCTION****************************************************************
 *
 * Function Name: SDEMMC_Shutdown
 * Description: destory initialized card and shutdown the corresponding
 * host controller
 *
 *END*********************************************************************/
void SDEMMC_Shutdown(sdhc_card_t *card)
{
    assert(card);
    SDEMMC_SelectCard(card, false);
}

/*FUNCTION****************************************************************
 *
 * Function Name: SDEMMC_SendApplicationCmd
 * Description: send application command to card
 *
 *END*********************************************************************/
static sdemmc_status_t SDEMMC_SendApplicationCmd(sdhc_card_t *card)
{
    sdhc_request_t *req = 0;
    sdemmc_status_t ret = kStatus_SDEMMC_NoError;
#if ! defined BSP_FSL_SDHC_USING_DYNALLOC
    sdhc_request_t request = {0};
    req = &request;
#endif
    assert(card);

#if defined BSP_FSL_SDHC_USING_DYNALLOC
    req = (sdhc_request_t *)OSA_MemAllocZero(sizeof(sdhc_request_t));
    if (req == NULL)
    {
        return kStatus_SDEMMC_OutOfMemory;
    }
#endif
    req->cmdIndex = kAppCmd;
    req->argument = 0;
    if (card->cardType != kCardTypeUnknown)
    {
        req->argument = card->rca << 16;
    }
    req->respType = kSdhcRespTypeR1;

    if (kStatus_SDEMMC_NoError !=
            SDHC_DRV_IssueRequestBlocking(card->hostInstance,
                                          req,
                                          FSL_SDCARD_REQUEST_TIMEOUT))
    {
        if (req->error & FSL_SDHC_REQ_ERR_CMD_TIMEOUT)
        {
            ret = kStatus_SDEMMC_TimeoutError;
        }
        else
        {
            ret = kStatus_SDEMMC_RequestFailed;
        }
#if defined BSP_FSL_SDHC_USING_DYNALLOC
        OSA_MemFree(req);
#endif
        req = NULL;
        return ret;
    }

    if (!(req->response[0] & SDMMC_R1_APP_CMD))
    {
#if defined BSP_FSL_SDHC_USING_DYNALLOC
        OSA_MemFree(req);
#endif
        req = NULL;
        return kStatus_SDEMMC_CardNotSupport;
    }
#if defined BSP_FSL_SDHC_USING_DYNALLOC
    OSA_MemFree(req);
#endif
    req = NULL;
    return kStatus_SDEMMC_NoError;
}

/*FUNCTION****************************************************************
 *
 * Function Name: SDEMMC_SetBlockSize
 * Description:  Set the block length in bytes for SDSC cards. For SDHC cards,
 * it does not affect memory read or write commands, always 512 bytes fixed
 * block length is used.
 *
 *END*********************************************************************/
static sdemmc_status_t SDEMMC_SetBlockSize(sdhc_card_t *card, uint32_t blockSize)
{
    sdhc_request_t *req = 0;
#if ! defined BSP_FSL_SDHC_USING_DYNALLOC
    sdhc_request_t request = {0};
    req = &request;
#endif
    assert(card);

#if defined BSP_FSL_SDHC_USING_DYNALLOC
    req = (sdhc_request_t *)OSA_MemAllocZero(sizeof(sdhc_request_t));
    if (req == NULL)
    {
        return kStatus_SDEMMC_OutOfMemory;
    }
#endif
    req->cmdIndex = kSetBlockLen;
    req->argument = blockSize;
    req->respType = kSdhcRespTypeR1;

    if (kStatus_SDEMMC_NoError !=
            SDHC_DRV_IssueRequestBlocking(card->hostInstance,
                                          req,
                                          FSL_SDCARD_REQUEST_TIMEOUT))
    {
#if defined BSP_FSL_SDHC_USING_DYNALLOC
        OSA_MemFree(req);
#endif
        req = NULL;
        return kStatus_SDEMMC_RequestFailed;
    }

#if defined BSP_FSL_SDHC_USING_DYNALLOC
    OSA_MemFree(req);
#endif
    req = NULL;
    return kStatus_SDEMMC_NoError;
}

/*FUNCTION****************************************************************
 *
 * Function Name: SDEMMC_SetBlkCnt
 * Description:  Send the set-block-count command.
 *
 *END*********************************************************************/
static sdemmc_status_t SDEMMC_SetBlkCnt(sdhc_card_t* card, uint32_t blockCount)
{
    sdhc_request_t *req = 0;
    uint32_t index;
    
#if ! defined BSP_FSL_SDHC_USING_DYNALLOC
    sdhc_request_t request = {0};
    req = &request;
#endif
    assert(card);
    
#if defined BSP_FSL_SDHC_USING_DYNALLOC
    req = (sdhc_request_t *)OSA_MemAllocZero(sizeof(sdhc_request_t));
    if (req == NULL)
    {
        return kStatus_SDEMMC_OutOfMemory;
    }
#endif
    req->cmdIndex = kSetBlockCount;
    req->argument = blockCount;
    req->respType = kSdhcRespTypeR1;

    if(kStatus_SDEMMC_NoError != SDHC_DRV_IssueRequestBlocking(card->host->instance,
                                        req,
                                        FSL_MMC_REQUEST_TIMEOUT))    
    {   
    #if defined BSP_FSL_SDHC_USING_DYNALLOC
        OSA_MemFree(req);
    #endif        
        req = NULL;
        return kStatus_SDEMMC_PreDefBlkCntFailed;
    }
#if defined BSP_FSL_SDHC_USING_DYNALLOC
    OSA_MemFree(req);
#endif
    req = NULL;
    return kStatus_SDEMMC_NoError;   
}



 