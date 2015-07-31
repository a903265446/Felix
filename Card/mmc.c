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
 * Function Name: SDEMMC_CheckReadOnly
 * Description: Check if the card is ready only
 *
 *END*********************************************************************/
bool EMMC_CheckReadOnly(sd_t *card)
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
 * Function Name: SDEMMC_SelectCard
 * Description: select or deselect card
 *
 *END*********************************************************************/
static emmc_status_t EMMC_SelectCard(sd_t *card, bool isSelected)
{
    host_request_t *req = 0;
#if ! defined BSP_HOST_USING_DYNALLOC
    host_request_t request = {0};
    req = &request;
#endif
    assert(card);

#if defined BSP_HOST_USING_DYNALLOC
    req = (host_request_t *)OSA_MemAllocZero(sizeof(host_request_t));
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
    host->currentReq = req;
    if (kStatus_SDEMMC_NoError != SDEMMC_IssueRequestBlocking(host))
    {
#if defined BSP_HOST_USING_DYNALLOC
        OSA_MemFree(req);
#endif
        req = NULL;
        return kStatus_SDEMMC_RequestFailed;
    }
#if defined BSP_HOST_USING_DYNALLOC
    OSA_MemFree(req);
#endif
    req = NULL;
    /* Wait until card to transfer state */
    return kStatus_SDEMMC_NoError;
}

/*FUNCTION****************************************************************
 *
 * Function Name: SDEMMC_SendStatus
 * Description:  send the sd status
 *
 *END*********************************************************************/
static sdemmc_status_t EMMC_SendStatus(host_t * host)
{
    host_request_t *req = 0;
    sdemmc_status_t err = kStatus_SDEMMC_NoError;
    uint32_t timeout = 1000;
#if ! defined BSP_HOST_USING_DYNALLOC
    host_request_t request = {0};
    req = &request;
#endif

#if defined BSP_HOST_USING_DYNALLOC
    req = (host_request_t *)OSA_MemAllocZero(sizeof(host_request_t));
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
        if (kStatus_SDEMMC_NoError != SDEMMC_IssueRequestBlocking(host))
        {
#if defined BSP_HOST_USING_DYNALLOC
            OSA_MemFree(req);
#endif
            req = NULL;
            return kStatus_SDEMMC_RequestFailed;
        }
        if ((req->response[0] & SDEMMC_R1_READY_FOR_DATA)
             && (SDEMMC_R1_CURRENT_STATE(req->response[0]) != SDEMMC_R1_STATE_PRG))
        {
            break;
        }

        SDEMMC_DelayMsec(1);
    } while(timeout--);

    if (!timeout)
    {
#if defined BSP_HOST_USING_DYNALLOC
        OSA_MemFree(req);
#endif
        req = NULL;
        return kStatus_SDEMMC_TimeoutError;
    }

#if defined BSP_HOST_USING_DYNALLOC
    OSA_MemFree(req);
#endif
    req = NULL;
    return err;
}

/*FUNCTION****************************************************************
 *
 * Function Name: SDEMMC_Shutdown
 * Description: destory initialized card and shutdown the corresponding
 * host controller
 *
 *END*********************************************************************/
void EMMC_Shutdown(sd_t *card)
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
static sdemmc_status_t EMMC_SendApplicationCmd(sd_t *card)
{
    host_request_t *req = 0;
    sdemmc_status_t ret = kStatus_SDEMMC_NoError;
#if ! defined BSP_HOST_USING_DYNALLOC
    host_request_t request = {0};
    req = &request;
#endif
    assert(card);

#if defined BSP_HOST_USING_DYNALLOC
    req = (host_request_t *)OSA_MemAllocZero(sizeof(host_request_t));
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
            SDEMMC_IssueRequestBlocking(card->hostInstance,
                                          req,
                                          FSL_SDCARD_REQUEST_TIMEOUT))
    {
        if (req->error & HOST_REQ_ERR_CMD_TIMEOUT)
        {
            ret = kStatus_SDEMMC_TimeoutError;
        }
        else
        {
            ret = kStatus_SDEMMC_RequestFailed;
        }
#if defined BSP_HOST_USING_DYNALLOC
        OSA_MemFree(req);
#endif
        req = NULL;
        return ret;
    }

    if (!(req->response[0] & SDEMMC_R1_APP_CMD))
    {
#if defined BSP_HOST_USING_DYNALLOC
        OSA_MemFree(req);
#endif
        req = NULL;
        return kStatus_SDEMMC_CardNotSupport;
    }
#if defined BSP_HOST_USING_DYNALLOC
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
static sdemmc_status_t EMMC_SetBlkCnt(sd_t* card, uint32_t blockCount)
{
    host_request_t *req = 0;
    uint32_t index;
    
#if ! defined BSP_HOST_USING_DYNALLOC
    host_request_t request = {0};
    req = &request;
#endif
    assert(card);
    
#if defined BSP_HOST_USING_DYNALLOC
    req = (host_request_t *)OSA_MemAllocZero(sizeof(host_request_t));
    if (req == NULL)
    {
        return kStatus_SDEMMC_OutOfMemory;
    }
#endif
    req->cmdIndex = kSetBlockCount;
    req->argument = blockCount;
    req->respType = kSdhcRespTypeR1;

    if(kStatus_SDEMMC_NoError != SDEMMC_IssueRequestBlocking(card->host->instance,
                                        req,
                                        FSL_MMC_REQUEST_TIMEOUT))    
    {   
    #if defined BSP_HOST_USING_DYNALLOC
        OSA_MemFree(req);
    #endif        
        req = NULL;
        return kStatus_SDEMMC_PreDefBlkCntFailed;
    }
#if defined BSP_HOST_USING_DYNALLOC
    OSA_MemFree(req);
#endif
    req = NULL;
    return kStatus_SDEMMC_NoError;   
}

/*FUNCTION****************************************************************
 *
 * Function Name: SDEMMC_GoIdle
 * Description: reset all cards to idle state
 *
 *END*********************************************************************/
static sdemmc_status_t EMMC_GoIdle(host_t* host)
{
    host_request_t *req = 0;
    sdemmc_status_t err;
#if ! defined BSP_HOST_USING_DYNALLOC
    host_request_t request = {0};
    req = &request;
#endif

#if defined BSP_HOST_USING_DYNALLOC
    req = (host_request_t *)OSA_MemAllocZero(sizeof(host_request_t));
    if (req == NULL)
    {
        return kStatus_SDEMMC_OutOfMemory;
    }
#endif
    req->cmdIndex = kGoIdleState;
    host->currentReq = req;
    err = SDEMMC_IssueRequestBlocking(host, CARD_REQUEST_TIMEOUT);
#if defined BSP_HOST_USING_DYNALLOC
    OSA_MemFree(req);
#endif
    req = NULL;
    return err;
}

#if ! defined BSP_HOST_ENABLE_AUTOCMD12
/*FUNCTION****************************************************************
 *
 * Function Name: SDEMMC_StopTransmission
 * Description:  Send stop transmission command to card to stop ongoing
 * data transferring.
 *
 *END*********************************************************************/
static sdemmc_status_t EMMC_StopTransmission(host_t * host)
{
    host_request_t *req = 0;
    sdemmc_status_t err = kStatus_SDEMMC_NoError;
#if ! defined BSP_HOST_USING_DYNALLOC
    host_request_t request = {0};
    req = &request;
#endif

#if defined BSP_HOST_USING_DYNALLOC
    req = (host_request_t *)OSA_MemAllocZero(sizeof(host_request_t));
    if (req == NULL)
    {
        return kStatus_SDEMMC_OutOfMemory;
    }
#endif

    req->cmdIndex = kStopTransmission;
    req->argument = 0;
    req->respType = kSdhcRespTypeR1b;
    req->data = 0;
    host->currentReq = req;
    if (kStatus_SDEMMC_NoError != SDEMMC_IssueRequestBlocking(host, CARD_REQUEST_TIMEOUT))
    {
#if defined BSP_HOST_USING_DYNALLOC
        OSA_MemFree(req);
#endif
        req = NULL;
        return kStatus_SDEMMC_RequestFailed;
    }

#if defined BSP_HOST_USING_DYNALLOC
    OSA_MemFree(req);
#endif
    req = NULL;
    return err;
}
#endif

/*FUNCTION****************************************************************
 *
 * Function Name: SDEMMC_SetBlockSize
 * Description:  Set the block length in bytes for SDSC cards. For SDHC cards,
 * it does not affect memory read or write commands, always 512 bytes fixed
 * block length is used.
 *
 *END*********************************************************************/
static sdemmc_status_t EMMC_SetBlockSize(host_t * host, uint32_t blockSize)
{
    host_request_t *req = 0;
#if ! defined BSP_HOST_USING_DYNALLOC
    host_request_t request = {0};
    req = &request;
#endif

#if defined BSP_HOST_USING_DYNALLOC
    req = (host_request_t *)OSA_MemAllocZero(sizeof(host_request_t));
    if (req == NULL)
    {
        return kStatus_SDEMMC_OutOfMemory;
    }
#endif
    req->cmdIndex = kSetBlockLen;
    req->argument = blockSize;
    req->respType = kSdhcRespTypeR1;
    host->currentReq = req;
    if (kStatus_SDEMMC_NoError != SDEMMC_IssueRequestBlocking(host, CARD_REQUEST_TIMEOUT))
    {
#if defined BSP_HOST_USING_DYNALLOC
        OSA_MemFree(req);
#endif
        req = NULL;
        return kStatus_SDEMMC_RequestFailed;
    }

#if defined BSP_HOST_USING_DYNALLOC
    OSA_MemFree(req);
#endif
    req = NULL;
    return kStatus_SDEMMC_NoError;
}

/*FUNCTION****************************************************************
 *
 * Function Name: EMMC_VolValid
 * Description: Validate the card voltage range if equals to host intended voltage range
 *
 *END*********************************************************************/
static sdhc_status_t EMMC_VolValid(sdhc_card_t *card, bool hostIntendedVolRange)
{
    sdhc_request_t *req = 0;
    sdhc_status_t err;
    uint8_t v170To195;
    uint16_t v270To360;
    uint8_t accMode;
    
    assert(card);
    /* Save host intended voltage range */
    card->hostIntVol       = kVol270to360;
#if ! defined BSP_FSL_SDHC_USING_DYNALLOC
    sdhc_request_t request = {0};
    req = &request;
#endif
    err = kStatus_SDHC_NoError;
#if defined BSP_FSL_SDHC_USING_DYNALLOC
    req = (sdhc_request_t *)OSA_MemAllocZero(sizeof(sdhc_request_t));
    if (req == NULL)
    {
        return kStatus_SDHC_OutOfMemory;
    }
#endif
    
    /* Send CMD1, with the intended voltage range in the argument 
    (either 0x00FF8000 or 0x00000080) */
    req->cmdIndex = kSendOpCond;
    if(hostIntendedVolRange == kVol170to195)
    {
        card->ocr.mmcOcr = 0;
        card->ocr.mmcOcr |=  (kVol170to195<<MMC_OCR_V170TO195_POS);
        req->argument = card->ocr.mmcOcr;
    }
    else
    {
        card->ocr.mmcOcr = 0;
        card->ocr.mmcOcr |= (kVol270to360<<MMC_OCR_V270TO360_POS);
        req->argument = card->ocr.mmcOcr;
    }
    req->respType = kSdhcRespTypeR3;
    
    do
    {
        err = SDHC_DRV_IssueRequestBlocking(card->host->instance,
                                        req,
                                        FSL_MMC_REQUEST_TIMEOUT);
        if (kStatus_SDHC_NoError == err)
        {
            if(!((req->response[0] & MMC_OCR_BUSY_MASK) >> MMC_OCR_BUSY_POS))/* Return busy state */
            {
                req = NULL;
                continue;
            }
            else
            {
                /* Get the voltage range and access mode in the OCR register */
                card->ocr.mmcOcr = req->response[0];
                /* Save raw OCR register content */
                //card->ocr        = req->response[0];
                break;
            }
        }
        else
        {
            break;
        }
        
    }while(1);
    if (kStatus_SDHC_NoError == err)
    {
    #if defined BSP_FSL_SDHC_USING_DYNALLOC
        OSA_MemFree(req);
    #endif        
        req = NULL;
        return kStatus_SDHC_NoError;
    }
#if defined BSP_FSL_SDHC_USING_DYNALLOC
    OSA_MemFree(req);
#endif
    req = NULL;
    return err;
}

/*FUNCTION****************************************************************
 *
 * Function Name: EMMC_SetRelAddr
 * Description: Set the relative address of the card.
 *
 *END*********************************************************************/
static sdhc_status_t EMMC_SetRelAddr(sdhc_card_t *card)
{
    sdhc_request_t *req = 0;
    sdhc_status_t err;

#if ! defined BSP_FSL_SDHC_USING_DYNALLOC
    sdhc_request_t request = {0};
    req = &request;
#endif
    assert(card);

#if defined BSP_FSL_SDHC_USING_DYNALLOC
    req = (sdhc_request_t *)OSA_MemAllocZero(sizeof(sdhc_request_t));
    if (req == NULL)
    {
        return kStatus_SDHC_OutOfMemory;
    }
#endif
    /* Send CMD3 with a chosen RCA, with value greater than 1 */
    req->cmdIndex = kMmcSetRelativeAddr;
    req->argument = (MMC_DEFAULT_RSA<<16);
    req->respType = kSdhcRespTypeR1;
    err = SDHC_DRV_IssueRequestBlocking(card->host->instance,
                                        req,
                                        FSL_MMC_REQUEST_TIMEOUT);
    
    if (kStatus_SDHC_NoError == err)
    {
        card->rca = MMC_DEFAULT_RSA;
    #if defined BSP_FSL_SDHC_USING_DYNALLOC
        OSA_MemFree(req);
    #endif        
        req = NULL;
        return kStatus_SDHC_NoError;
    }
#if defined BSP_FSL_SDHC_USING_DYNALLOC
    OSA_MemFree(req);
#endif
    req = NULL;
    return err;
}

/*FUNCTION****************************************************************
 *
 * Function Name: EMMC_CalDevSizeAsBlks
 * Description: Calculate the size of the card.
 *
 *END*********************************************************************/
static void EMMC_CalDevSizeAsBlks(sdhc_card_t* card)
{
    uint32_t c_size,c_size_mult, mult, read_bl_len, block_len;
    uint32_t blkSizeDef = FSL_SDHC_CARD_DEFAULT_BLOCK_SIZE;/* Default block size after power on */    
    assert(card);
    
    c_size                   = card->csd.mmcCsd.c_size;
    /*  For higher than 2GB of density of card the maximum possible value should
    be set to this register (0xFFF) */
    if(c_size != 0xFFF)
    {
        c_size_mult              = card->csd.mmcCsd.c_size_mult;
        mult                     = (2 << (c_size_mult + 2 - 1));
        read_bl_len              = card->csd.mmcCsd.readBlkLen;
        block_len                = (2 << (read_bl_len - 1));
        
        card->blockCount         = (((c_size + 1)*mult)/blkSizeDef);        
    }
    else /* For higher than 2GB of density of card, the device size represented 
      by sector-count field in the EXT_CSD */
    {
        card->blockCount         = card->ext1.mmcExtCsd.sectorCount;          
        card->caps              |= SDMMC_CARD_CAPS_HIGHCAPACITY;
    }
    card->blockSize              = FSL_SDHC_CARD_DEFAULT_BLOCK_SIZE;
}

/*FUNCTION****************************************************************
 *
 * Function Name: EMMC_DecodeCsd
 * Description: Decode the CSD register content.
 *
 *END*********************************************************************/
static void EMMC_DecodeCsd(uint32_t *rawCsd, sdhc_card_t *card)
{
    mmccard_csd_t *csd;
    assert(rawCsd);
    assert(card);
    csd = &(card->csd.mmcCsd);
    csd->csdStructVer = (uint8_t)((rawCsd[3] & 0xC0000000U) >> 30);
    csd->sysSpecVer   = (uint8_t)((rawCsd[3] & 0x30000000U) >> 28);
    if(4 == csd->sysSpecVer)
    {
         card->caps |= SDMMC_CARD_CAPS_HIGHSPEED;         
    }
    csd->taac = (uint8_t)((rawCsd[3] & 0xFF0000) >> 16);
    csd->nsac = (uint8_t)((rawCsd[3] & 0xFF00) >> 8);
    csd->tranSpeed = (uint8_t)(rawCsd[3] & 0xFF);
    csd->ccc = (uint16_t)((rawCsd[2] & 0xFFF00000U) >> 20);
    /* Max block length which can be read/write one time */
    csd->readBlkLen = (uint8_t)((rawCsd[2] & 0xF0000) >> 16);
    if (rawCsd[2] & 0x8000)
    {
        csd->flags |= MMCCARD_CSD_READ_BL_PARTIAL;
    }
    if (rawCsd[2] & 0x4000)
    {
        csd->flags |= MMCCARD_CSD_WRITE_BLK_MISALIGN;
    }
    if (rawCsd[2] & 0x2000)
    {
        csd->flags |= MMCCARD_CSD_READ_BLK_MISALIGN;
    }
    if (rawCsd[2] & 0x1000)
    {
        csd->flags |= MMCCARD_CSD_DSR_IMP;
    }
    csd->c_size = (uint16_t)(((rawCsd[2]&0x300)<<2) + ((rawCsd[2]&0xFF) << 2) 
                               + (rawCsd[1]&0xC0000000)>>30 );
    csd->vdd_r_cur_min = (uint8_t)((rawCsd[1]&0x38000000)>>27);
    csd->vdd_r_cur_max = (uint8_t)((rawCsd[1]&0x07000000)>>24);
    csd->vdd_w_cur_min = (uint8_t)((rawCsd[1]&0x00E00000)>>21);
    csd->vdd_w_cur_max = (uint8_t)((rawCsd[1]&0x001C0000)>>18);
    csd->c_size_mult   = (uint8_t)((rawCsd[1]&0x00038000)>>15);
    csd->eraseGrpSize  = (uint8_t)((rawCsd[1]&0x00007C00)>>10);
    csd->eraseGrpSizeMult = (uint8_t)((rawCsd[1]&0x000003E0)>>5);
    csd->wpGrpSize     = (uint8_t)(rawCsd[1]&0x0000001F);
    if(rawCsd[0] & 0x80000000)
    {
        csd->flags |= MMCCARD_CSD_WP_GRP_ENABLED;
    }
    csd->defaultEcc    = (uint8_t)((rawCsd[0] & 0x60000000) >> 29);
    csd->writeSpeedFactor = (uint8_t)((rawCsd[0] & 0x1C000000) >> 26); 
    csd->maxWriteBlkLen   = (uint8_t)((rawCsd[0] & 0x03C00000) >> 22);
    if(rawCsd[0] & 0x00200000)
    {
        csd->flags |= MMCCARD_CSD_WRITE_BL_PARTIAL;
    }
    if(rawCsd[0] & 0x00010000)
    {
        csd->flags |= MMCCARD_CSD_CONTENT_PROT_APP;
    }
    if(rawCsd[0] & 0x00008000)
    {
        csd->flags |= MMCCARD_CSD_FILE_FORMAT_GROUP;
    }
    if(rawCsd[0] & 0x00004000)
    {
        csd->flags |= MMCCARD_CSD_COPY;
    }
    if(rawCsd[0] & 0x00002000)
    {
        csd->flags |= MMCCARD_CSD_PERM_WRITE_PROTECT;
    }
    if(rawCsd[0] & 0x00001000)
    {
        csd->flags |= MMCCARD_CSD_TMP_WRITE_PROTECT;
    }
    csd->fileFormat = (uint8_t)((rawCsd[0] & 0x00000C00) >> 10);
    csd->eccCode    = (uint8_t)((rawCsd[0] & 0x00000300) >> 8);
    /* Calculate the device size */
    EMMC_CalDevSizeAsBlks(card); 
}

/*!
 * @brief The divide value used to avoid fload point calculation
 */
#define MULT_DIV_IN_TRAN_SPEED 10;

/*FUNCTION****************************************************************
 *
 * Function Name: EMMC_SetMaxTranSpeed
 * Description: Set the card to max transfer speed
 *
 *END*********************************************************************/
static sdhc_status_t EMMC_SetMaxNonHsSpeed(sdhc_card_t* card)
{
    uint32_t freqUnitInCsd;
    uint32_t mult10InCsd;
    uint32_t maxFreq; 
    /* g_freqUnitInTranSpeed and g_mult10InTranSpeed are used to calculate the max speed 
      in 1 bit mode of card.
      To MMC card: For cards supporting version 4.0, 4.1, and 4.2 of the specification,
      the value shall be 20MHz (0x2A). For cards supporting version 4.3, the value 
      shall be 26 MHz (0x32). In High speed mode, the max frequence is decided by 
      CARD_TYPE in EXT_CSD. 
      To SD card: Note that for current SD Memory Cards, this field shall be always 
      0_0110_010b (032h) which is equal to 25 MHz - the mandatory maximum operating 
      frequency of SD Memory Card. In High-Speed mode, this field shall be always 
      0_1011_010b (05Ah) which is equal to 50 MHz, and when the timing mode returns 
      to the default by CMD6 or CMD0 command, its value will be 032h. */
    /* Frequence unit defined in TRAN-SPEED field in CSD */
    uint32_t g_freqUnitInTranSpeed[] = {100000, 1000000, 10000000, 100000000};
    /* The multiplying value defined in TRAN-SPEED field in CSD */
    uint32_t g_mult10InTranSpeed[]   = {0, 10, 12, 13, 15, 20, 26, 30, 35, 40, 45,\
                               52, 55, 60, 70, 80};

    assert(card);
    
    freqUnitInCsd       = g_freqUnitInTranSpeed[RD_MMC_CSD_TRAN_SPEED_FREQ_UNIT(card->csd.mmcCsd)];
    mult10InCsd         = g_mult10InTranSpeed[RD_MMC_CSD_TRAN_SPPED_MULT(card->csd.mmcCsd)];
    maxFreq             = (freqUnitInCsd*mult10InCsd)/MULT_DIV_IN_TRAN_SPEED;
    if (SDHC_DRV_ConfigClock(card->host->instance, maxFreq))
    {
        return kStatus_SDHC_SetClockFailed;
    }
    return kStatus_SDHC_NoError;
}

/*FUNCTION****************************************************************
 *
 * Function Name: EMMC_CalLegacyErsUnitSize
 * Description: Calculate legacy erase unit size
 *
 *END*********************************************************************/
static void EMMC_CalLegacyErsUnitSize(sdhc_card_t* card)
{
    uint32_t erase_grp_size, erase_grp_mult;
    assert(card);
    
    erase_grp_size                    = card->csd.mmcCsd.eraseGrpSize;
    erase_grp_mult                    = card->csd.mmcCsd.eraseGrpSizeMult;
    card->mmcErsGrpSizeAsBlk          = ((erase_grp_size + 1)*(erase_grp_mult + 1));
    card->mmcErsGrpCnt                = (((card->blockCount)/(card->mmcErsGrpSizeAsBlk)) - 1);     
}

/*FUNCTION****************************************************************
 *
 * Function Name: EMMC_CalHCErsUnitSize
 * Description: Calculate high capacity erase unit size
 *
 *END*********************************************************************/
static void EMMC_CalHCErsUnitSize(sdhc_card_t* card)
{
    assert(card);
    if((0 == card->ext1.mmcExtCsd.HC_ERASE_GRP_SIZE) | (0 == card->ext1.mmcExtCsd.ERASE_TIMEOUT_MULT) )
    {
        EMMC_CalLegacyErsUnitSize(card);
        return;
    }
    card->mmcErsGrpSizeAsBlk          = card->ext1.mmcExtCsd.HC_ERASE_GRP_SIZE;
    card->mmcErsGrpCnt                = (((card->blockCount)/(card->mmcErsGrpSizeAsBlk)) - 1);   
}

/*FUNCTION****************************************************************
 *
 * Function Name: EMMC_CalErsUnitSize
 * Description: Calculate erase unit size of the card
 *
 *END*********************************************************************/
static void EMMC_CalErsUnitSize(sdhc_card_t* card)
{
    assert(card);
    /* If the master enables bit ¡°0¡± in the extended CSD register byte [175], 
    the slave uses high capacity value for the erase operation */
    if((card->ext1.mmcExtCsd.ERASE_GROUP_DEF)&0x01)
    {
        EMMC_CalLegacyErsUnitSize(card);
    }
    else
    {
        EMMC_CalHCErsUnitSize(card);
    }
}

/*FUNCTION****************************************************************
 *
 * Function Name: EMMC_FillExtCsdByte
 * Description: Fill the specific field of the card according to the index and 
 * value inputted by the user.
 *
 *END*********************************************************************/
static sdhc_status_t EMMC_FillExtCsdByte(sdhc_card_t* card, mmc_extcsd_op_t* op)
{
    uint32_t param = 0;
    sdhc_request_t *req = 0;
    sdhc_status_t err = kStatus_SDHC_NoError;
    sdhc_data_t data = {0};    
    
    assert(card);
    assert(op);
    
    param |= (op->cmdSet << MMC_SWITCH_PARAM_CMD_SET_POS);
    param |= (op->value << MMC_SWITCH_PARAM_VALUE_POS);
    param |= (op->indexOfByte << MMC_SWITCH_PARAM_INDEX_OF_BYTE_POS);
    param |= (op->accMode << MMC_SWITCH_PARAM_ACCESS_MODE_POS);
#if ! defined BSP_FSL_SDHC_USING_DYNALLOC
    sdhc_request_t request = {0};
    req = &request;
#endif
    
#if defined BSP_FSL_SDHC_USING_DYNALLOC
    req = (sdhc_request_t *)OSA_MemAllocZero(sizeof(sdhc_request_t));
    if (req == NULL)
    {
        return kStatus_SDHC_OutOfMemory;
    }
#endif
    req->cmdIndex = kMmcSwitch;
    req->argument = param;
    req->respType = kSdhcRespTypeR1b;/* Send switch command to set the pointed byte*/
    if(kStatus_SDHC_NoError != SDHC_DRV_IssueRequestBlocking(card->host->instance,
                                        req,
                                        FSL_MMC_REQUEST_TIMEOUT))    
    {   
    #if defined BSP_FSL_SDHC_USING_DYNALLOC
        OSA_MemFree(req);
    #endif        
        req = NULL;
        return kStatus_SDHC_SendMMCSwitchCmdError;
    }
#if defined BSP_FSL_SDHC_USING_DYNALLOC
    OSA_MemFree(req);
#endif
    req = NULL;
    /* wait for the card to be out of BUSY */
    if(kStatus_SDHC_NoError != SDEMMC_SendStatus(card))
    {
        return kStatus_SDHC_SendCardStatusFailed;
    } 
    return kStatus_SDHC_NoError;   
}


/*FUNCTION****************************************************************
 *
 * Function Name: EMMC_EnableHCErsUnitSize
 * Description: Enable the high capacity erase feature of the card.
 *
 *END*********************************************************************/
static void EMMC_EnableHCErsUnitSize(sdhc_card_t* card)
{
    /* Enable the high capacity erase unit */
    mmc_extcsd_op_t op;
    assert(card);
    op.accMode      = kSetBits;
    op.indexOfByte  = ERASE_GRP_DEF_INDEX;
    op.value        = 0x01;/* The high capacity erase unit size enablement bit is bit 0*/
    EMMC_FillExtCsdByte(card, &op);
}

/*FUNCTION****************************************************************
 *
 * Function Name: EMMC_DecodeExtCsd
 * Description: Decode the EXT_CSD register
 *
 *END*********************************************************************/
static sdhc_status_t EMMC_DecodeExtCsd(uint32_t* rawExtCsd, sdhc_card_t* card)
{
    mmccard_extcsd_t* extCsd;
    assert(rawExtCsd);
    assert(card);
    
    extCsd                        = &(card->ext1.mmcExtCsd);
    extCsd->supportedCmdSet       = (rawExtCsd[1]&0x000000FF);
    extCsd->bootInfo              = (rawExtCsd[70]&0x000000FF);
    if(kSupAltBoot == extCsd->bootInfo)
    {/* Card support alternate boot*/
        card->caps               |= MMC_CARD_SUP_ALT_BOOT;
    }
    extCsd->bootSizeMult          = ((rawExtCsd[71]&0x00FF0000)>>16);
    /* Get boot partition size*/
    card->bootPartSizeAsByte      = (128*1024*extCsd->bootSizeMult);
      
    extCsd->accessSize            = ((rawExtCsd[71]&0x0000FF00)>>8);
    extCsd->HC_ERASE_GRP_SIZE     = ((rawExtCsd[71]&0x000000FF));
    extCsd->ERASE_TIMEOUT_MULT    = ((rawExtCsd[72]&0xFF000000)>>24);
    extCsd->reliableWrSecCnt      = ((rawExtCsd[72]&0x00FF0000)>>16);
    extCsd->hc_wp_grp_size        = ((rawExtCsd[72]&0x0000FF00)>>8);
    extCsd->sleepCurrentVCC       = (rawExtCsd[72]&0x000000FF);
    extCsd->sleepCurrentVCCQ      = ((rawExtCsd[73]&0xFF000000)>>24);
    extCsd->slpAwkTimeout         = ((rawExtCsd[73]&0x0000FF00)>>8);
    extCsd->sectorCount           = rawExtCsd[74];
    extCsd->MIN_PERF_W_8_52       = ((rawExtCsd[75]&0x00FF0000)>>16);
    extCsd->MIN_PERF_R_8_52       = ((rawExtCsd[75]&0x0000FF00)>>8);
    extCsd->MIN_PERF_W_8_26_4_52  = (rawExtCsd[75]&0x000000FF);
    extCsd->MIN_PERF_R_8_26_4_52  = ((rawExtCsd[76]&0xFF000000)>>24);
    extCsd->MIN_PERF_W_4_26       = ((rawExtCsd[76]&0x00FF0000)>>16);
    extCsd->MIN_PERF_R_4_26       = ((rawExtCsd[76]&0x0000FF00)>>8);
    extCsd->PWR_CL_26_360         = ((rawExtCsd[77]&0xFF000000)>>24);
    extCsd->PWR_CL_52_360         = ((rawExtCsd[77]&0x00FF0000)>>16);
    extCsd->PWR_CL_26_195         = ((rawExtCsd[77]&0x0000FF00)>>8);
    extCsd->PWR_CL_52_195         = (rawExtCsd[77]&0x000000FF);
    extCsd->cardType              = (rawExtCsd[78]&0x000000FF);
    extCsd->csdStrucVer           = ((rawExtCsd[79]&0x00FF0000)>>16);
    extCsd->extcsdVer             = (rawExtCsd[79]&0x000000FF);
    /* If ext_csd version is V4.3, high capacity erase feature can only be enabled
    in MMC V4.3 */
    if(extCsd->extcsdVer == kextcsdVer13)
    {
        /* Enable the high capacity erase unit */
        EMMC_EnableHCErsUnitSize(card);
    }
    extCsd->cmdSet                = ((rawExtCsd[80]&0xFF000000)>>24);  
    extCsd->cmdSetRev             = ((rawExtCsd[80]&0x0000FF00)>>8);
    extCsd->powerCls              = ((rawExtCsd[81]&0xFF000000)>>24);
    extCsd->HS_TIMING             = ((rawExtCsd[81]&0x0000FF00)>>8);
    extCsd->busWidth              = ((rawExtCsd[82]&0xFF000000)>>24);
    extCsd->erasedMemCnt          = ((rawExtCsd[82]&0x0000FF00)>>8);
    extCsd->bootConfig            = ((rawExtCsd[83]&0xFF000000)>>24);
    extCsd->bootBusWidth          = ((rawExtCsd[83]&0x0000FF00)>>8);
    extCsd->ERASE_GROUP_DEF       = ((rawExtCsd[84]&0xFF000000)>>24);
    /* Calculate the erase unit size */
    EMMC_CalErsUnitSize(card);
    return kStatus_SDHC_NoError;
} 

/*FUNCTION****************************************************************
 *
 * Function Name: EMMC_SendExtCsd
 * Description: Get the content of the EXT_CSD register
 *
 *END*********************************************************************/
static sdhc_status_t EMMC_SendExtCsd(sdhc_card_t* card)
{
    sdhc_request_t *req = 0;
    sdhc_status_t err = kStatus_SDHC_NoError;
    sdhc_data_t data = {0};
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
        return kStatus_SDHC_OutOfMemory;
    }
#endif
    req->cmdIndex = kMmcSendExtCsd;
    req->argument = 0;
    req->respType = kSdhcRespTypeR1;
    req->flags    = FSL_SDHC_REQ_FLAGS_DATA_READ;
    req->data     = &data;
    data.blockCount = 1;
    data.blockSize  = MMC_EXTCSD_LEN_AS_BYTE;
    data.buffer     = card->rawExt1.rawExtcsd;
    data.req        = req;
    err = SDHC_DRV_IssueRequestBlocking(card->host->instance,
                                        req,
                                        FSL_MMC_REQUEST_TIMEOUT);
    
    if (kStatus_SDHC_NoError == err)
    {        
        /*The response is from bit 127:8 in R2, corisponding to req->response[3]
        :req->response[0][31:8]*/
        for(index=0; index<MMC_EXTCSD_LEN_AS_WORD; index++)
        {
            card->rawExt1.rawExtcsd[index] = swap_be32(card->rawExt1.rawExtcsd[index]);
        }
        EMMC_DecodeExtCsd(card->rawExt1.rawExtcsd, card);
    #if defined BSP_FSL_SDHC_USING_DYNALLOC
        OSA_MemFree(req);
    #endif        
        req = NULL;
        return kStatus_SDHC_NoError;
    }
#if defined BSP_FSL_SDHC_USING_DYNALLOC
    OSA_MemFree(req);
#endif
    req = NULL;
    return err;
}

/*FUNCTION****************************************************************
 *
 * Function Name: EMMC_GetHighSpeedFreq
 * Description: Get the frequence when card in high speed mode
 *
 *END*********************************************************************/
static sdhc_status_t EMMC_GetHighSpeedFreq(sdhc_card_t* card)
{
    /* This field defines the type of the card. he only currently valid values \
    for this field are 0x01 and 0x03. */
    if(kHSAt26MHZ == RD_CARDTYPE_HSFREQ_26MHZ(card->ext1.mmcExtCsd))
    {
        card->caps  |= MMC_CARD_SUP_HS_26MHZ;
    }
    else if(kHSAt52MHZ == RD_CARDTYPE_HSFREQ_52MHZ(card->ext1.mmcExtCsd))
    {
        card->caps  |= MMC_CARD_SUP_HS_52MHZ;
    }
    else
    {
        return kStatus_SDHC_SwitchHighSpeedFailed;
    }
    return kStatus_SDHC_NoError;
}

/*FUNCTION****************************************************************
 *
 * Function Name: EMMC_GetPowCls
 * Description: Get the power class of the card at specific bus width and 
 * specific voltage
 *
 *END*********************************************************************/
static sdhc_status_t EMMC_GetPowCls(sdhc_card_t* card, uint8_t* getPowCls,
                                           mmc_bus_width_t busWidth)
{  
    uint8_t mask;  
    assert(card);
    if(busWidth <= kMmcBusWidth1b)
    {
        return kStatus_SDHC_GetMMCPowClsError;
    }    
    if(kMmcBusWidth4b == busWidth)
    {
        mask = MMC_EXTCSD_PWRCLFFVV_4BUS_MASK;/* the mask of 4 bit bus width's power class*/
    }
    else
    {
        mask = MMC_EXTCSD_PWRCLFFVV_8BUS_MASK; /* the mask of 8 bit bus width's power class*/  
    }
    if( DOES_MMC_CARD_SUP_HS_52MHZ(card) && (kVol170to195 == card->hostIntVol) )
    {
        *getPowCls = ((card->ext1.mmcExtCsd.PWR_CL_52_195)&mask);
    }
    else if(DOES_MMC_CARD_SUP_HS_52MHZ(card) && (kVol270to360 == card->hostIntVol))
    {
        *getPowCls = ((card->ext1.mmcExtCsd.PWR_CL_52_360)&mask);
    }
    else if(DOES_MMC_CARD_SUP_HS_26MHZ(card) && (kVol170to195 == card->hostIntVol))
    {
        /* 8 bit at 26MHZ/195V*/
        *getPowCls = ((card->ext1.mmcExtCsd.PWR_CL_26_195)&mask);
    }
    else if(DOES_MMC_CARD_SUP_HS_26MHZ(card) && (kVol270to360 == card->hostIntVol))
    {
        /* 8 bit at 26MHZ/360V*/
        *getPowCls = ((card->ext1.mmcExtCsd.PWR_CL_26_360)&mask);
    }
    else
    {
        //have some error
        return kStatus_SDHC_GetMMCPowClsError;
    }
    return kStatus_SDHC_NoError;
}

/*FUNCTION****************************************************************
 *
 * Function Name: EMMC_SendTestPattern
 * Description: Send test pattern to get the functional pin in the MMC bus
 *
 *END*********************************************************************/
static sdhc_status_t EMMC_SendTestPattern(sdhc_card_t* card, 
                                                 uint32_t blockSize, uint32_t* buffer)
{
    sdhc_request_t *req = 0;
    sdhc_data_t data = {0};
#if ! defined BSP_FSL_SDHC_USING_DYNALLOC
    sdhc_request_t request = {0};
    req = &request;
#endif
    assert(card);
    assert(blockSize <= FSL_SDHC_CARD_DEFAULT_BLOCK_SIZE);
    assert(buffer);
    
#if defined BSP_FSL_SDHC_USING_DYNALLOC
    req = (sdhc_request_t *)OSA_MemAllocZero(sizeof(sdhc_request_t));
    if (req == NULL)
    {
        return kStatus_SDHC_OutOfMemory;
    }
#endif
    req->cmdIndex = kSendTuningBlock;
    req->argument = 0;
    req->respType = kSdhcRespTypeR1;
    req->data     = &data;
    data.blockCount = 1;
    data.blockSize  = blockSize;
    data.buffer     = buffer;
    data.req        = req;
    if(kStatus_SDHC_NoError != SDHC_DRV_IssueRequestBlocking(card->host->instance,
                                        req,
                                        FSL_MMC_REQUEST_TIMEOUT))    
    {   
    #if defined BSP_FSL_SDHC_USING_DYNALLOC
        OSA_MemFree(req);
    #endif        
        req = NULL;
        return kStatus_SDHC_BusTestProcFailed;
    }
#if defined BSP_FSL_SDHC_USING_DYNALLOC
    OSA_MemFree(req);
#endif
    req = NULL;
    return kStatus_SDHC_NoError;
}

/*FUNCTION****************************************************************
 *
 * Function Name: EMMC_GetTestPattern
 * Description: Get test pattern reversed by the card after the send-test-pattern
 * command
 *
 *END*********************************************************************/
static sdhc_status_t EMMC_GetTestPattern(sdhc_card_t* card, uint32_t blockSize, uint32_t* buffer)
{
    sdhc_request_t *req = 0;
    sdhc_data_t data = {0};
#if ! defined BSP_FSL_SDHC_USING_DYNALLOC
    sdhc_request_t request = {0};
    req = &request;
#endif
    assert(card);
    assert(blockSize <= FSL_SDHC_CARD_DEFAULT_BLOCK_SIZE);
    assert(buffer);
    
#if defined BSP_FSL_SDHC_USING_DYNALLOC
    req = (sdhc_request_t *)OSA_MemAllocZero(sizeof(sdhc_request_t));
    if (req == NULL)
    {
        return kStatus_SDHC_OutOfMemory;
    }
#endif
    req->cmdIndex = kMmcBusTestRead;
    req->respType = kSdhcRespTypeR1;
    req->flags    = FSL_SDHC_REQ_FLAGS_DATA_READ;
    req->data     = &data;
    data.blockCount = 1;
    data.blockSize  = blockSize;
    data.buffer     = buffer;
    data.req        = req;
    if(kStatus_SDHC_NoError != SDHC_DRV_IssueRequestBlocking(card->host->instance,
                                        req,
                                        FSL_MMC_REQUEST_TIMEOUT))    
    {   
    #if defined BSP_FSL_SDHC_USING_DYNALLOC
        OSA_MemFree(req);
    #endif        
        req = NULL;
        return kStatus_SDHC_BusTestProcFailed;
    }
#if defined BSP_FSL_SDHC_USING_DYNALLOC
    OSA_MemFree(req);
#endif
    req = NULL;
    return kStatus_SDHC_NoError;   
}

/*FUNCTION****************************************************************
 *
 * Function Name: EMMC_BusTestProc
 * Description: Bus test procedure to get the functional pin in the bus
 *
 *END*********************************************************************/
static sdhc_status_t EMMC_BusTestProc(sdhc_card_t* card, mmc_bus_width_t busWidth)
{
    uint32_t blockSize;
    uint32_t sendPattern[kMmcBusWidth8b];
    uint32_t recvPattern[kMmcBusWidth8b];
    bool testRes = false; /* Test procedure not passed*/
    assert(card);
    assert(busWidth <= kMmcBusWidth8b);
    
    /* For 8 data lines the data block would be (MSB to LSB): 0x0000_0000_0000_AA55,
    For 4 data lines the data block would be (MSB to LSB): 0x0000_005A, 
    For only 1 data line the data block would be: 0x80*/
    if(kMmcBusWidth8b == busWidth)
    {
        blockSize  = kMmcBusWidth8b;
        sendPattern[0]  = MMC_TEST_PATTERN_8BIT_BUS;
        sendPattern[1]  = 0;        
    }
    else if(kMmcBusWidth4b == busWidth)
    {
        blockSize  = kMmcBusWidth4b;
        sendPattern[0]  = MMC_TEST_PATTERN_4BIT_BUS;
    }
    else
    {
        blockSize  = kMmcBusWidth1b;
        sendPattern[0]  = MMC_TEST_PATTERN_1BIT_BUS;
    }
    if(kStatus_SDHC_NoError != EMMC_SendTestPattern(card, blockSize, sendPattern))
    {
        return kStatus_SDHC_BusTestProcFailed;
    }
    if(kStatus_SDHC_NoError != EMMC_GetTestPattern(card, blockSize, recvPattern))
    {
        return kStatus_SDHC_BusTestProcFailed;
    }
    /* XOR the send pattern and recv pattern */
    if(kMmcBusWidth8b == busWidth)
    {
        if( (sendPattern[0]^recvPattern[0]) == MMC_PATTERN_XOR_RES_8BIT_BUS )
        {
            testRes = true; /* Test procedure passed */
        }
    }
    else if(kMmcBusWidth4b == busWidth)
    {
        if( (sendPattern[0]^recvPattern[0]) == MMC_PATTERN_XOR_RES_4BIT_BUS )
        {
            testRes = true; /* Test procedure passed */
        }
    }
    else if(kMmcBusWidth1b == busWidth)
    {
        if( (sendPattern[0]^recvPattern[0]) == MMC_PATTERN_XOR_RES_1BIT_BUS )
        {
            testRes = true; /* Test procedure passed */
        }
    }
    if(!testRes)
    {
        return kStatus_SDHC_BusTestProcFailed;
    }
    return kStatus_SDHC_NoError;    
}


/*FUNCTION****************************************************************
 *
 * Function Name: EMMC_SetBusWidth
 * Description: Set the bus width.
 *
 *END*********************************************************************/
static sdhc_status_t EMMC_SetBusWidth(sdhc_card_t* card, 
                                             mmc_bus_width_t busWidth)
{    
    uint8_t powClsAt8bit;
    mmc_extcsd_op_t extOp;
    assert(card);
    if(kStatus_SDHC_NoError != 
       EMMC_GetPowCls(card, &powClsAt8bit, busWidth))
    {
        return kStatus_SDHC_GetMMCPowClsError;
    } 
    /* Set power class of pointed bus width */
    extOp.accMode     = kWriteBits;
    extOp.indexOfByte = POWER_CLASS_INDEX;
    extOp.value       = powClsAt8bit;
    if(kStatus_SDHC_NoError != EMMC_FillExtCsdByte(card, &extOp))
    {
        return kStatus_SDHC_SetPowClsError;
    }          
    /* Set bus width of pointed bus width */
    extOp.accMode     = kWriteBits;
    extOp.indexOfByte = BUS_WIDTH_INDEX;
    extOp.value       = busWidth;      
    if(kStatus_SDHC_NoError != EMMC_FillExtCsdByte(card, &extOp))
    {
        return kStatus_SDHC_SetBusWidthFailed;
    }    
    return kStatus_SDHC_NoError;    
}

/*FUNCTION****************************************************************
 *
 * Function Name: EMMC_SwitchHighSpeed
 * Description: Switch the card to high speed mode
 *
 *END*********************************************************************/
static sdhc_status_t EMMC_SwitchHighSpeed(sdhc_card_t* card)
{    
    mmc_extcsd_op_t extOp;    
    assert(card);
   
    /* If host support high speed mode, then switch to high speed. */
    if (DOES_HOST_SUPPORT_HIGHSPEED(card->host))
    {       
        /* Switch to high speed timing */
        extOp.accMode     = kWriteBits;
        extOp.indexOfByte = HS_TIMING_INDEX;
        extOp.value       = kMmcHSTiming;
        if(kStatus_SDHC_NoError != EMMC_FillExtCsdByte(card, &extOp))
        {
            return kStatus_SDHC_SwitchHighSpeedFailed;
        }   
        if(kStatus_SDHC_NoError != EMMC_GetHighSpeedFreq(card))
        {
            return kStatus_SDHC_SwitchHighSpeedFailed;
        }
        /* Switch to corresponding frequence in high speed mode  */
        if(DOES_MMC_CARD_SUP_HS_52MHZ(card))
        {
            if (kStatus_SDHC_NoError !=
                    SDHC_DRV_ConfigClock(card->host->instance, MMC_CLK_52MHZ))
            {
                return kStatus_SDHC_SetClockFailed;
            }
        }
        else if(DOES_MMC_CARD_SUP_HS_26MHZ(card))
        {
             if (kStatus_SDHC_NoError !=
                    SDHC_DRV_ConfigClock(card->host->instance, MMC_CLK_26MHZ))
            {
                return kStatus_SDHC_SetClockFailed;
            } 
        }
        else
        {
            return kStatus_SDHC_SetClockFailed;
        }
    }
    return kStatus_SDHC_NoError;    
}

/*FUNCTION****************************************************************
 *
 * Function Name: EMMC_DecodeCid
 * Description: decode cid register
 *
 *END*********************************************************************/
static void EMMC_DecodeCid(uint32_t *rawCid, sdhc_card_t *card)
{
    mmccard_cid_t *cid;
    assert(rawCid);
    assert(card);
    cid = &(card->cid.mmcCid);

    cid->mid           = (uint8_t)((rawCid[3] & 0xFF000000) >> 24);

    cid->oid           = (uint16_t)((rawCid[3] & 0xFFFF00) >> 8);

    cid->pnm[0] = (uint8_t)((rawCid[3] & 0xFF));
    cid->pnm[1] = (uint8_t)((rawCid[2] & 0xFF000000U) >> 24);
    cid->pnm[2] = (uint8_t)((rawCid[2] & 0xFF0000) >> 16);
    cid->pnm[3] = (uint8_t)((rawCid[2] & 0xFF00) >> 8);
    cid->pnm[4] = (uint8_t)((rawCid[2] & 0xFF));

    cid->prv = (uint8_t)((rawCid[1] & 0xFF000000U) >> 24);

    cid->psn = (uint32_t)((rawCid[1] & 0xFFFFFF) << 8);
    cid->psn |= (uint32_t)((rawCid[0] & 0xFF000000U) >> 24);

    cid->mdt = (uint16_t)((rawCid[0] & 0xFFF00) >> 8);
}

/*FUNCTION****************************************************************
 *
 * Function Name: EMMC_AutoSetWidth
 * Description: Set the bus width automatically
 *
 *END*********************************************************************/
static sdhc_status_t EMMC_AutoSetWidth(sdhc_card_t *card)
{
    uint8_t i;
    mmc_bus_width_t maxBusWidth, busWidth[] = {kMmcBusWidth1b, kMmcBusWidth4b, kMmcBusWidth8b};
    assert(card);
    
    /* Get max width data bus */
    for(i = 0; i < MMC_BUSWIDTH_TYPE_NUM; i++)
    {
        if(kStatus_SDHC_NoError == EMMC_BusTestProc(card, busWidth[i]))
        {
            maxBusWidth = busWidth[i];
            break;
        } 
    }
    if(i == MMC_BUSWIDTH_TYPE_NUM)/* Board haven't functional pin. */
    {
        return kStatus_SDHC_SetBusWidthFailed;
    }
     /* From the EXT_CSD the host can learn the power class of the card,
and choose to work with a wider data bus */
    if(kStatus_SDHC_NoError != EMMC_SetBusWidth(card, maxBusWidth))
    {
        return kStatus_SDHC_SetBusWidthFailed;
    }
    if((kMmcBusWidth4b == maxBusWidth)&&DOES_HOST_SUPPORT_4BITS(card->host))
    {
        if (kStatus_SDHC_NoError != SDHC_DRV_SetBusWidth(card->hostInstance, kSdhcBusWidth4Bit))
        {
            return kStatus_SDHC_SetBusWidthFailed;
        }
    }               
    else if((kMmcBusWidth8b == maxBusWidth)&&DOES_HOST_SUPPORT_8BITS(card->host))
    {
        if (kStatus_SDHC_NoError != SDHC_DRV_SetBusWidth(card->hostInstance, kSdhcBusWidth8Bit))
        {
            return kStatus_SDHC_SetBusWidthFailed;
        }
    }
    return kStatus_SDHC_NoError;
}


/*FUNCTION****************************************************************
 *
 * Function Name: EMMC_Init
 * Description: Initialize the MMCCARD
 *
 *END*********************************************************************/
static sdhc_status_t EMMC_Init(sdhc_card_t *card)
{
    sdhc_status_t bootRes;
    assert(card);
    
    /* Set clock to 400KHz, or less */
    if (SDHC_DRV_ConfigClock(card->host->instance, SDMMC_CLK_400KHZ))
    {
        return kStatus_SDHC_SetClockFailed;
    }
    
    /* Send CMD0 to reset the bus */
    if(kStatus_SDHC_NoError != SDEMMC_GoIdle(card))
    {
        return kStatus_SDHC_SetCardToIdle;
    }
    /* Apply power to the bus, communication voltage range (2.7-3.6V) */
    /* Validate the voltage range */
    if(kStatus_SDHC_NoError != EMMC_VolValid(card, kVol270to360))
    {
        return kStatus_SDHC_SendAppOpCondFailed;
    }
    /* Get card CID */
    if(kStatus_SDHC_NoError != SDEMMC_AllSendCid(card))
    {
        return kStatus_SDHC_AllSendCidFailed;
    }
    /* Set the card address */
    if(kStatus_SDHC_NoError != EMMC_SetRelAddr(card))
    {
        return kStatus_SDHC_SendRcaFailed;
    }
    /* Get the CSD register content */
    if(kStatus_SDHC_NoError != SD_SendCsd(card))
    {
        return kStatus_SDHC_SendCsdFailed;
    }
    /* If necessary, adjust the host parameters according to the information in the CSD */
    if(card->csd.mmcCsd.sysSpecVer == 4)
    {
        card->caps |= SDMMC_CARD_CAPS_HIGHSPEED;
        card->caps |= MMC_CARD_SUPPORT_EXT_CSD;
    }
    else
    {
        /* Card is old MMC card */
        return kStatus_SDHC_OldMmcCard;
    }
    /* Send CMD7 with the card¡¯s RCA to place the card in tran state */
    /* Puts current selected card in trans state */
    if(kStatus_SDHC_NoError != SDEMMC_SelectCard(card, true))
    {
        return kStatus_SDHC_SelectCardFailed;
    }
    /* Get EXT_CSD register content */
    if(kStatus_SDHC_NoError != EMMC_SendExtCsd(card))
    {
        return kStatus_SDHC_SendExtCsdFailed;
    }    
    if(kStatus_SDHC_NoError != EMMC_AutoSetWidth(card))/* Sets card data width and block size  */
    {
        return kStatus_SDHC_SetBusWidthFailed;
    }
    /* Switch to high speed mode */  
    if(kStatus_SDHC_NoError != EMMC_SwitchHighSpeed(card))
    {
        return kStatus_SDHC_SwitchHighSpeedFailed;
    }
    if (SDEMMC_SetBlockSize(card, FSL_SDHC_CARD_DEFAULT_BLOCK_SIZE))
    {
        return kStatus_SDHC_SetCardBlockSizeFailed;
    }
    /* Set default access non-boot partition */
    card->curAccPart = kAccBootPartNot;
    card type = mmc
    return kStatus_SDHC_NoError;
}


/*FUNCTION****************************************************************
 *
 * Function Name: EMMC_Erase
 * Description:  Erase the MMC card content
 *
 *END*********************************************************************/
static sdhc_status_t EMMC_Erase(sdhc_card_t* card, uint32_t ersGrpStart, 
                                uint32_t grpCount)
{
    uint32_t s, e;
    sdhc_request_t *req = 0;
    assert(card);
    assert(grpCount);    
    /*Assert not to pass the device capacity*/
    assert(ersGrpStart < (ersGrpStart + grpCount));
    assert((ersGrpStart + grpCount - 1) <= (card->mmcErsGrpCnt));  
    /* Assert the start address must be group address */
    if((ersGrpStart+1)%(card->mmcErsGrpSizeAsBlk*FSL_SDHC_CARD_DEFAULT_BLOCK_SIZE) != 0)
    {
        return kStatus_SDHC_AddrNotGrp;
    }
    switch(card->curAccPart)
    {
    case kAccBootPartNot:
      {
          if((ersGrpStart < (ersGrpStart + grpCount)) \
            || ((ersGrpStart + grpCount - 1) <= (card->mmcErsGrpCnt)))
          {
              return kStatus_SDHC_AddrNotGrp;
          }
          break;
      }
    case kAccBootPart1:
    case kAccBootPart2:
      {
          /* Check if the erase request data size is more than boot partition size */
          if(((ersGrpStart + grpCount - 1)*(card->mmcErsGrpSizeAsBlk))*FSL_SDHC_CARD_DEFAULT_BLOCK_SIZE\
                     > (card->bootPartSizeAsByte))
          {
              return kStatus_SDHC_InvalidIORange;
          }
          break;
      }
    }
    
#if ! defined BSP_FSL_SDHC_USING_DYNALLOC
    sdhc_request_t request = {0};
    req = &request;
#endif

#if defined BSP_FSL_SDHC_USING_DYNALLOC
    req = (sdhc_request_t *)OSA_MemAllocZero(sizeof(sdhc_request_t));
    if (req == NULL)
    {
        return kStatus_SDHC_OutOfMemory;
    }
#endif
    /* Calculate the start group number and end group number */
    s = ersGrpStart;
    e = s + grpCount - 1;
    if(DOES_CARD_SUPPORT_HIGHCAPACITY(card))
    {
        /* The implementation of a higher than 2GB of density of memory will not
      be backwards compatible with the lower densities.First of all the address 
      argument for higher than 2GB of density of memory is changed to be sector 
      address (512B sectors) instead of byte address */
        s = (s * (card->mmcErsGrpSizeAsBlk));
        e = (e * (card->mmcErsGrpSizeAsBlk));
    }
    else
    {
        /* The address unit is byte when card capacity is lower than 2GB*/
        s = (s * (card->mmcErsGrpSizeAsBlk)*FSL_SDHC_CARD_DEFAULT_BLOCK_SIZE);
        e = (e * (card->mmcErsGrpSizeAsBlk)*FSL_SDHC_CARD_DEFAULT_BLOCK_SIZE);
    }
    /* Set the start erase group address */         
    req->cmdIndex = kMmcEraseGroupStart;
    req->argument = s;
    req->respType = kSdhcRespTypeR1;
    if (kStatus_SDHC_NoError !=
            SDHC_DRV_IssueRequestBlocking(card->host->instance,
                                          req,
                                          FSL_MMC_REQUEST_TIMEOUT))
    {
#if defined BSP_FSL_SDHC_USING_DYNALLOC
        OSA_MemFree(req);
#endif
        req = NULL;
        return kStatus_SDHC_RequestFailed;
    }
    /* Set the end erase group address */
    req->cmdIndex = kMmcEraseGroupEnd;
    req->argument = e;
    if (kStatus_SDHC_NoError !=
            SDHC_DRV_IssueRequestBlocking(card->host->instance,
                                          req,
                                          FSL_MMC_REQUEST_TIMEOUT))
    {
#if defined BSP_FSL_SDHC_USING_DYNALLOC
        OSA_MemFree(req);
#endif
        req = NULL;
        return kStatus_SDHC_RequestFailed;
    }
    /* Start the erase process */
    req->cmdIndex = kErase;
    req->argument = 0;
    req->respType = kSdhcRespTypeR1b;
    if (kStatus_SDHC_NoError !=
            SDHC_DRV_IssueRequestBlocking(card->host->instance,
                                          req,
                                          FSL_MMC_REQUEST_TIMEOUT))
    {
#if defined BSP_FSL_SDHC_USING_DYNALLOC
        OSA_MemFree(req);
#endif
        req = NULL;
        return kStatus_SDHC_RequestFailed;
    }

#if defined BSP_FSL_SDHC_USING_DYNALLOC
    OSA_MemFree(req);
#endif
    req = NULL;
    /* Wait the write and program process complete in the card */
    if (kStatus_SDHC_NoError != SDEMMC_SendStatus(card))
    {
        return kStatus_SDHC_SendCardStatusFailed;
    }
    return kStatus_SDHC_NoError;
}

/*FUNCTION****************************************************************
 *
 * Function Name: EMMC_SelPart
 * Description:  Select the partition used to read/write data after initilized 
 *
 *END*********************************************************************/
sdhc_status_t EMMC_SelPart(sdhc_card_t* card, mmc_acc_part_t partNum)
{
    mmc_extcsd_op_t extOp;
    assert(card);
    assert(kCardTypeMmc == card->cardType);
    uint8_t bootConfig;
    bootConfig  = card->ext1.mmcExtCsd.bootConfig;
    bootConfig |= (partNum << BOOT_PARTITION_ACCESS_POS);
    extOp.accMode     = kWriteBits;
    extOp.indexOfByte = BOOT_CONFIG_INDEX;
    extOp.value       = bootConfig;      
    if(kStatus_SDHC_NoError != EMMC_FillExtCsdByte(card, &extOp))
    {
        return kStatus_SDHC_ConfigBootFailed;
    }
    card->ext1.mmcExtCsd.bootConfig = bootConfig;   
    /* Save current access partition number */
    card->curAccPart = partNum;
    return kStatus_SDHC_NoError;
}

/*FUNCTION****************************************************************
 *
 * Function Name: SDEMMC_ConfigBootPart
 * Description:  Configure boot activity after power on 
 *
 *END*********************************************************************/
sdhc_status_t EMMC_ConfPowOnBoot(sdhc_card_t* card, const mmcboot_user_config_t* bootConfig)
{
    uint8_t bootParam;
    mmc_extcsd_op_t extOp;
    assert(card);
    assert(kCardTypeMmc == card->cardType);
    assert(bootConfig);
    if(kCardTypeMmc != card->cardType)
    {
        return kStatus_SDHC_CardTypeError;
    }
    if(kextcsdVer13 != card->ext1.mmcExtCsd.extcsdVer)/* Only V4.3 support fast boot */
    {
        return kStatus_SDHC_NotSupportYet;
    }
    /* Nothing to set */
    if(!(bootConfig->setFlags))
    {
        return kStatus_SDHC_InvalidParameter;
    }
    /* Set the BOOT_CONFIG field of EXT_CSD */
    bootParam = card->ext1.mmcExtCsd.bootConfig;
    if( (bootConfig->setFlags) & BOOT_PARTITION_ENABLE_SET_FLAG)
    {        
        bootParam |= ((bootConfig->configMask & BOOT_PARTITION_ENABLE_MASK) >> BOOT_PARTITION_ENABLE_POS);
    }
    if(bootConfig->setFlags & BOOT_ACK_SET_FLAG)
    {
        bootParam |= ((bootConfig->configMask & BOOT_ACK_MASK) >> BOOT_ACK_POS);
    }      
    extOp.accMode     = kWriteBits;
    extOp.indexOfByte = BOOT_CONFIG_INDEX;
    extOp.value       = bootParam;      
    if(kStatus_SDHC_NoError != EMMC_FillExtCsdByte(card, &extOp))
    {
        return kStatus_SDHC_ConfigBootFailed;
    }    
    card->ext1.mmcExtCsd.bootConfig = bootParam;
    /*Set BOOT_BUS_WIDTH in EXT_CSD */
    bootParam = card->ext1.mmcExtCsd.bootBusWidth;
    if(bootConfig->setFlags & BOOT_BUS_WIDTH_SET_FLAG)
    {
        bootParam |= ((bootConfig->configMask & BOOT_BUS_WIDTH_MASK) >> BOOT_BUS_WIDTH_POS);
    }
    if(bootConfig->setFlags & RESET_BOOT_BUS_WIDTH_SET_FLAG)
    {
        bootParam |= ((bootConfig->configMask & RESET_BOOT_BUS_WIDTH_MASK) >> RESET_BOOT_BUS_WIDTH_POS);
    }
    extOp.accMode     = kWriteBits;
    extOp.indexOfByte = BOOT_BUS_WIDTH_INDEX;
    extOp.value       = bootParam;      
    if(kStatus_SDHC_NoError != EMMC_FillExtCsdByte(card, &extOp))
    {
        return kStatus_SDHC_ConfigBootFailed;
    }     
    card->ext1.mmcExtCsd.bootBusWidth = bootParam;
    return kStatus_SDHC_NoError;
}



/*FUNCTION****************************************************************
 *
 * Function Name: EMMC_Read
 * Description: read data from specific MMC card
 *
 *END*********************************************************************/
static sdhc_status_t EMMC_Read(sdhc_card_t *card,
                                     uint8_t *buffer,
                                     uint32_t startBlock,
                                     uint32_t blockSize,
                                     uint32_t blockCount)
{
    sdhc_request_t *req = 0;
    sdhc_data_t data = {0};
#if ! defined BSP_FSL_SDHC_USING_DYNALLOC
    sdhc_request_t request = {0};
    req = &request;
#endif

    assert(card);
    assert(buffer);
    assert(blockCount);
    assert(blockSize);
    assert(blockSize == FSL_SDHC_CARD_DEFAULT_BLOCK_SIZE);
    /* Check address range */
    switch(card->curAccPart)
    {
    case kAccBootPartNot:
      {
         if ((IS_HIGHCAPACITY_CARD(card) && (blockSize != 512))
             || (blockSize > card->blockSize)
             || (blockSize > card->host->maxBlockSize)
             || (blockSize % 4))
         {
             return kStatus_SDHC_BlockSizeNotSupportError;
         }
         break;
      }
    case kAccBootPart1:
    case kAccBootPart2:
      {
         /* Boot part1 and part2 have the same partition size */
         if(((startBlock+blockCount)*FSL_SDHC_CARD_DEFAULT_BLOCK_SIZE) \
                                 > card->bootPartSizeAsByte)
         {
             return kStatus_SDHC_BlockSizeNotSupportError;
         }
         break;
      }
      
    }
    
#if defined BSP_FSL_SDHC_USING_DYNALLOC
    req = (sdhc_request_t *)OSA_MemAllocZero(sizeof(sdhc_request_t));
    if (req == NULL)
    {
        return kStatus_SDHC_OutOfMemory;
    }
#endif

    data.blockSize = blockSize;
    data.blockCount = blockCount;
    data.buffer = (uint32_t *)buffer;

    req->data = &data;
    req->cmdIndex = kReadMultipleBlock;
    if (data.blockCount == 1)
    {
        req->cmdIndex = kReadSingleBlock;
    }
    else
    {
#if ((! defined BSP_FSL_SDHC_ENABLE_AUTOCMD12) && (defined MMC_SUP_PRE_DEF_BLK_CNT))
         /* If enabled the pre-define count read/write featue of the card, 
         need to set block count firstly */
#if defined MMC_SUP_PRE_DEF_BLK_CNT
         SDEMMC_SetBlkCnt(card, blockCount);
#endif

#endif   
    }

    req->argument = startBlock;
    if (!IS_HIGHCAPACITY_CARD(card))
    {
        req->argument *= data.blockSize;
    }
    req->flags = FSL_SDHC_REQ_FLAGS_DATA_READ;
    req->respType = kSdhcRespTypeR1;

    data.req = req;
    if (kStatus_SDHC_NoError !=
            SDHC_DRV_IssueRequestBlocking(card->hostInstance,
                                          req,
                                          FSL_SDCARD_REQUEST_TIMEOUT))
    {
#if defined BSP_FSL_SDHC_USING_DYNALLOC
        OSA_MemFree(req);
#endif
        req = NULL;
        return kStatus_SDHC_RequestFailed;
    }

#if ((! defined BSP_FSL_SDHC_ENABLE_AUTOCMD12) && (! defined MMC_SUP_PRE_DEF_BLK_CNT))
    if (data.blockCount > 1)
    {
         if (kStatus_SDHC_NoError != SDEMMC_StopTransmission(card))
         {
#if defined BSP_FSL_SDHC_USING_DYNALLOC
             OSA_MemFree(req);
#endif
             req = NULL;
             return kStatus_SDHC_StopTransmissionFailed;
          }
    }
#endif
#if defined BSP_FSL_SDHC_USING_DYNALLOC
    OSA_MemFree(req);
#endif
    req = NULL;
    return kStatus_SDHC_NoError;
}

/*FUNCTION****************************************************************
 *
 * Function Name: EMMC_Write
 * Description: write data from specific MMC card
 *
 *END*********************************************************************/
static sdhc_status_t EMMC_Write(sdhc_card_t *card,
                                      uint8_t *buffer,
                                      uint32_t startBlock,
                                      uint32_t blockSize,
                                      uint32_t blockCount)
{
    sdhc_request_t *req = 0;
    sdhc_data_t data = {0};
#if ! defined BSP_FSL_SDHC_USING_DYNALLOC
    sdhc_request_t request = {0};
    req = &request;
#endif
    assert(card);
    assert(buffer);
    assert(blockCount);
    assert(blockSize);
    assert(blockSize == FSL_SDHC_CARD_DEFAULT_BLOCK_SIZE);
    /* Check address range */
    switch(card->curAccPart)
    {
    case kAccBootPartNot:
      {
         if ((IS_HIGHCAPACITY_CARD(card) && (blockSize != 512))
             || (blockSize > card->blockSize)
             || (blockSize > card->host->maxBlockSize)
             || (blockSize % 4))
         {
             return kStatus_SDHC_BlockSizeNotSupportError;
         }
         break;
      }
    case kAccBootPart1:
    case kAccBootPart2:
      {
         /* Boot part1 and part2 have the same partition size */
         if(((startBlock+blockCount)*FSL_SDHC_CARD_DEFAULT_BLOCK_SIZE) \
                                 > card->bootPartSizeAsByte)
         {
             return kStatus_SDHC_BlockSizeNotSupportError;
         }
         break;
      }
      
    }
#if defined BSP_FSL_SDHC_USING_DYNALLOC
    req = (sdhc_request_t *)OSA_MemAllocZero(sizeof(sdhc_request_t));
    if (req == NULL)
    {
        return kStatus_SDHC_OutOfMemory;
    }
#endif

    data.blockSize = blockSize;
    data.blockCount = blockCount;
    data.buffer = (uint32_t *)buffer;

    req->data = &data;
    req->cmdIndex = kWriteMultipleBlock;
    if (data.blockCount == 1)
    {
        req->cmdIndex = kWriteBlock;
    }
    else
    {
#if ((! defined BSP_FSL_SDHC_ENABLE_AUTOCMD12) && (defined MMC_SUP_PRE_DEF_BLK_CNT))
        /* If enabled the pre-define count read/write featue of the card, 
        need to set block count firstly */
#if defined MMC_SUP_PRE_DEF_BLK_CNT
        SDEMMC_SetBlkCnt(card, blockCount);
#endif
#endif

    }
    req->argument = startBlock;
    if (!IS_HIGHCAPACITY_CARD(card))
    {
        req->argument *= data.blockSize;
    }
    req->respType = kSdhcRespTypeR1;
    data.req = req;
    if (kStatus_SDHC_NoError !=
            SDHC_DRV_IssueRequestBlocking(card->hostInstance,
                                          req,
                                          FSL_SDCARD_REQUEST_TIMEOUT))
    {
#if defined BSP_FSL_SDHC_USING_DYNALLOC
        OSA_MemFree(req);
#endif
        req = NULL;
        return kStatus_SDHC_RequestFailed;
    }

    if (data.blockCount > 1)
    {
#if ((! defined BSP_FSL_SDHC_ENABLE_AUTOCMD12) && (!defined MMC_SUP_PRE_DEF_BLK_CNT))             
        if (kStatus_SDHC_NoError != SDEMMC_StopTransmission(card))
        {
#if defined BSP_FSL_SDHC_USING_DYNALLOC
            OSA_MemFree(req);
#endif
            req = NULL;
            return kStatus_SDHC_StopTransmissionFailed;
        }

#endif
        if (kStatus_SDHC_NoError != SDEMMC_SendStatus(card))
        {
#if defined BSP_FSL_SDHC_USING_DYNALLOC
            OSA_MemFree(req);
#endif
            req = NULL;
            return kStatus_SDHC_SendCardStatusFailed;
        }
    }
#if defined BSP_FSL_SDHC_USING_DYNALLOC
    OSA_MemFree(req);
#endif
    req = NULL;
    return kStatus_SDHC_NoError;
}
             
/*FUNCTION****************************************************************
 *
 * Function Name: EMMC_AllSendCid
 * Description: send all_send_cid command
 *
 *END*********************************************************************/
static sdhc_status_t EMMC_AllSendCid(sdhc_card_t *card)
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
        return kStatus_SDHC_OutOfMemory;
    }
#endif
    req->cmdIndex = kAllSendCid;
    req->argument = 0;
    req->respType = kSdhcRespTypeR2;
    if (kStatus_SDHC_NoError ==
            SDHC_DRV_IssueRequestBlocking(card->hostInstance,
                                          req,
                                          FSL_SDCARD_REQUEST_TIMEOUT))
    {
        memcpy(card->rawCid, req->response, sizeof(card->rawCid));
        EMMC_DecodeCid(req->response, card);    
        
#if defined BSP_FSL_SDHC_USING_DYNALLOC
        OSA_MemFree(req);
#endif
        req = NULL;
        return kStatus_SDHC_NoError;
    }
#if defined BSP_FSL_SDHC_USING_DYNALLOC
    OSA_MemFree(req);
#endif
    req = NULL;
    return kStatus_SDHC_RequestFailed;
}

/*FUNCTION****************************************************************
 *
 * Function Name: SDCARD_DRV_SendCsd
 * Description: get csd from card
 *
 *END*********************************************************************/
static sdmmc_status_t SD_SendCsd(sd_t * card)
{
    host_request_t *req = 0;
#if ! defined FSL_CARD_DRIVER_USING_DYNALLOC
    host_request_t request = {0};
    req = &request;
#endif
    assert(card);

#if defined FSL_CARD_DRIVER_USING_DYNALLOC
    req = (host_request_t *)OSA_MemAllocZero(sizeof(host_request_t));
    if (req == NULL)
    {
        return kStatus_SDMMC_OutOfMemory;
    }
#endif
    req->cmdIndex = kSendCsd;
    req->argument = card->rca << 16;
    req->respType = kSdhcRespTypeR2;
    card->host->currentReq = req;
    if (kStatus_SDMMC_NoError ==
            SDMMC_IssueRequestBlocking(card, FSL_CARD_REQUEST_TIMEOUT))
    {
        memcpy(card->rawCsd, req->response, sizeof(card->rawCsd));
        /*The response is from bit 127:8 in R2, corrisponding to req->response[3]
        :req->response[0][31:8]*/
        switch(card->cardType)
        {
        case kCardTypeSd:
          {
              SDCARD_DRV_SdDecodeCsd(req->response, card);
              break;
          }
        case kCardTypeMmc:
          {
              SDCARD_DRV_MmcDecodeCsd(req->response, card);
              break;
          }
        default:
          {
              break;
          }
        }
        
#if defined FSL_CARD_DRIVER_USING_DYNALLOC
        OSA_MemFree(req);
#endif
        req = NULL;
        return kStatus_SDMMC_NoError;
    }
#if defined FSL_CARD_DRIVER_USING_DYNALLOC
    OSA_MemFree(req);
#endif
    req = NULL;
    return kStatus_SDMMC_RequestFailed;
}

/*FUNCTION****************************************************************
 *
 * Function Name: SDCARD_DRV_Init
 * Description: initialize card on the given host controller
 *
 *END*********************************************************************/
sdhc_status_t SDCARD_DRV_Init(sdhc_host_t *host, sdhc_card_t *card)
{
    sdhc_status_t err = kStatus_SDHC_NoError;
    uint32_t acmd41Arg;

    assert(card);
    assert(host);

    card->cardType = kCardTypeUnknown;
    card->host = host;
    card->hostInstance = host->instance;

    if (SDHC_DRV_ConfigClock(card->hostInstance, SDMMC_CLK_400KHZ))
    {
        return kStatus_SDHC_SetClockFailed;
    }

    err = SDCARD_DRV_GoIdle(card);
    if (err)
    {
        return kStatus_SDHC_SetCardToIdle;
    }
    acmd41Arg = card->host->ocrSupported;

    err = SDCARD_DRV_SdSendIfCond(card);
    if (err == kStatus_SDHC_NoError)
    {
        /* SDHC or SDXC card */
        acmd41Arg |= SD_OCR_HCS;
        card->caps |= SDMMC_CARD_CAPS_SDHC;
    }
    else
    {
        /* SDSC card */
        err = SDCARD_DRV_GoIdle(card);
        if (err)
        {
            return kStatus_SDHC_SetCardToIdle;
        }
    }

    err = SDCARD_DRV_SdAppSendOpCond(card, acmd41Arg);
    if (kStatus_SDHC_TimeoutError == err)
    {
        /* MMC card */
        //return kStatus_SDHC_NotSupportYet;
        
        return SDCARD_DRV_MmcInit(card);
    }
    else if (err)
    {
        return kStatus_SDHC_SendAppOpCondFailed;
    }

    return SDCARD_DRV_SdInit(card);
}
             
/*FUNCTION****************************************************************
 *
 * Function Name: SDCARD_DRV_ReadBlocks
 * Description: read blocks from card with default block size from SD or MMC card
 *
 *END*********************************************************************/
sdhc_status_t SDCARD_DRV_ReadBlocks(sdhc_card_t *card,
                                    uint8_t *buffer,
                                    uint32_t startBlock,
                                    uint32_t blockCount)
{
    uint32_t blkCnt, blkLeft, blkDone;
    sdhc_status_t err = kStatus_SDHC_NoError;

    assert(card);
    assert(buffer);
    assert(blockCount);

    blkLeft = blockCount;
    blkDone = 0;

    if ((blockCount + startBlock) > card->blockCount)
    {
        return kStatus_SDHC_InvalidIORange;
    }

    while(blkLeft)
    {
        if (blkLeft > card->host->maxBlockCount)
        {
            blkLeft = blkLeft - card->host->maxBlockCount;
            blkCnt = card->host->maxBlockCount;
        }
        else
        {
            blkCnt = blkLeft;
            blkLeft = 0;
        }
        switch(card->cardType)
        {
        case kCardTypeSd:
            {
                err = SDCARD_DRV_SdRead(card, buffer, startBlock,FSL_SDHC_CARD_DEFAULT_BLOCK_SIZE,  blockCount);
                break;
            }
        case kCardTypeMmc:
            {
                err = SDCARD_DRV_MmcRead(card, buffer, startBlock, FSL_SDHC_CARD_DEFAULT_BLOCK_SIZE, blockCount);
                break;
            }
        default:
            {
                err = kStatus_SDHC_CardTypeError;
                break;
            }
        }  
        if (err != kStatus_SDHC_NoError)
        {
            return err;
        }
        blkDone += blkCnt;
    }

    return err;
}

/*FUNCTION****************************************************************
 *
 * Function Name: SDCARD_DRV_WriteBlocks
 * Description: write blocks to card with default block size to SD/MMC card
 *
 *END*********************************************************************/
sdhc_status_t SDCARD_DRV_WriteBlocks(sdhc_card_t *card,
                                     uint8_t *buffer,
                                     uint32_t startBlock,
                                     uint32_t blockCount)
{
    uint32_t blkCnt, blkLeft, blkDone;
    sdhc_status_t err = kStatus_SDHC_NoError;

    assert(card);
    assert(buffer);
    assert(blockCount);

    blkLeft = blockCount;
    blkDone = 0;

    if ((blockCount + startBlock) > card->blockCount)
    {
        return kStatus_SDHC_InvalidIORange;
    }

    while(blkLeft)
    {
        if (blkLeft > card->host->maxBlockCount)
        {
            blkLeft = blkLeft - card->host->maxBlockCount;
            blkCnt = card->host->maxBlockCount;
        }
        else
        {
            blkCnt = blkLeft;
            blkLeft = 0;
        }

        switch(card->cardType)
        {
        case kCardTypeSd:
            {
                err = SDCARD_DRV_SdWrite(card, buffer, startBlock, FSL_SDHC_CARD_DEFAULT_BLOCK_SIZE, blockCount);
                break;
            }
        case kCardTypeMmc:
            {
                err = SDCARD_DRV_MmcWrite(card, buffer, startBlock, FSL_SDHC_CARD_DEFAULT_BLOCK_SIZE, blockCount);
                break;
            }
        default:
            {
                err = kStatus_SDHC_CardTypeError;
                break;
            }
        } 
        if (err != kStatus_SDHC_NoError)
        {
            return err;
        }
        blkDone += blkCnt;
    }

    return err;
}
             
/*FUNCTION****************************************************************
 *
 * Function Name: SDCARD_DRV_EraseBlocks
 * Description: erase block range from SD/MMC card with default block size
 *
 *END*********************************************************************/
sdhc_status_t SDCARD_DRV_EraseBlocks(sdhc_card_t *card,
                                     uint32_t startBlock,
                                     uint32_t blockCount)
{
    uint32_t blkDone = 0, blkLeft, blkCnt;
    sdhc_status_t err = kStatus_SDHC_NoError;
    assert(card);
    assert(blockCount);


    blkLeft = blockCount;
    while(blkLeft)
    {
        /* Devide total blocks to multiple sectorSize(max blocks can be erase one time)*/
        if (blkLeft > (card->csd.sdCsd.sectorSize + 1))
        {
            blkCnt = card->csd.sdCsd.sectorSize + 1;
            blkLeft = blkLeft - blkCnt;
        }
        else
        {
            blkCnt = blkLeft;
            blkLeft = 0;
        }
        switch(card->cardType)
        {
        case kCardTypeSd:
            {
                err = SDCARD_DRV_SdErase(card, startBlock, blockCount);
                break;
            }
        case kCardTypeMmc:
            {
                err = SDCARD_DRV_MmcErase(card, startBlock, blockCount);
                break;
            }
        default:
            {
                err = kStatus_SDHC_CardTypeError;
                break;
            }
        }  
        if (kStatus_SDHC_NoError != err)
        {
            return kStatus_SDHC_CardEraseBlocksFailed;
        }

        if (kStatus_SDHC_NoError != SDCARD_DRV_SendStatus(card))
        {
            return kStatus_SDHC_SendCardStatusFailed;
        }
        blkDone += blkCnt;
    }
    return kStatus_SDHC_NoError;
}

