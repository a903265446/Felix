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


#include <string.h>
#include "card.h"

/*FUNCTION****************************************************************
 *
 * Function Name: SDMMC_CheckReadOnly
 * Description: Check if the card is ready only
 *
 *END*********************************************************************/
bool SD_CheckReadOnly(sd_t *card)
{
    assert(card);
    return ((card->csd.flags & SD_CSD_PERM_WRITE_PROTECT) ||
            (card->csd.flags & SD_CSD_TMP_WRITE_PROTECT));   
    //return false;/* Default as non-read only */    
}

/*FUNCTION****************************************************************
 *
 * Function Name: SDMMC_SelectCard
 * Description: select or deselect card
 *
 *END*********************************************************************/
static sdmmc_status_t SD_SelectCard(sd_t *card, bool isSelected)
{
    host_t* host = card->host;
    card_cmd_t *cardCmd = 0;
#if ! defined FSL_CARD_DRIVER_USING_DYNALLOC
    card_cmd_t command = {0};
    cardCmd = &command;
#endif
    assert(card);

#if defined FSL_CARD_DRIVER_USING_DYNALLOC
    cardCmd = (card_cmd_t *)OSA_MemAllocZero(sizeof(card_cmd_t));
    if (cardCmd == NULL)
    {
        return kStatus_SDMMC_OutOfMemory;
    }
#endif
    cardCmd->cmdIndex = kSdmmcSelectCard;
    if (isSelected)
    {
        cardCmd->argument = card->rca << 16;
        cardCmd->respType = kSdmmcRespTypeR1;
    }
    else
    {
        cardCmd->argument = 0;
        cardCmd->respType = kSdmmcRespTypeNone;
    }
    host->currentCmd = cardCmd;
    host->currentData = NULL;
    if ((kStatus_SDMMC_NoError != SDMMC_SendCmdBlocking(host, FSL_CARD_COMMAND_TIMEOUT))
        || (kStatus_SDMMC_NoError != SDMMC_CheckR1Response(cardCmd)))
    {
#if defined FSL_CARD_DRIVER_USING_DYNALLOC
        OSA_MemFree(cardCmd);
#endif
        cardCmd = NULL;
        return kStatus_SDMMC_SendCardCmdFailed;
    }

#if defined FSL_CARD_DRIVER_USING_DYNALLOC
    OSA_MemFree(cardCmd);
#endif
    cardCmd = NULL;
    /* Wait until card to transfer state */
    return kStatus_SDMMC_NoError;
}

/*FUNCTION****************************************************************
 *
 * Function Name: SDMMC_SendStatus
 * Description:  send the sd status
 *
 *END*********************************************************************/
static sdmmc_status_t SD_SendStatus(sd_t *card)
{
    host_t * host = card->host;
    card_cmd_t *cardCmd = 0;
    sdmmc_status_t err = kStatus_SDMMC_NoError;
    uint32_t timeout = 1000;
#if ! defined FSL_CARD_DRIVER_USING_DYNALLOC
    card_cmd_t command = {0};
    cardCmd = &command;
#endif

#if defined FSL_CARD_DRIVER_USING_DYNALLOC
    cardCmd = (card_cmd_t *)OSA_MemAllocZero(sizeof(card_cmd_t));
    if (cardCmd == NULL)
    {
        return kStatus_SDMMC_OutOfMemory;
    }
#endif
    cardCmd->cmdIndex = kSdmmcSendStatus;
    cardCmd->argument = card->rca << 16;
    cardCmd->respType = kSdmmcRespTypeR1;
    host->currentCmd = cardCmd;
    host->currentData = 0;
    do
    {
        if ((kStatus_SDMMC_NoError != SDMMC_SendCmdBlocking(host, FSL_CARD_COMMAND_TIMEOUT))
            || (kStatus_SDMMC_NoError != SDMMC_CheckR1Response(cardCmd)))
        {
#if defined FSL_CARD_DRIVER_USING_DYNALLOC
            OSA_MemFree(cardCmd);
#endif
            cardCmd = NULL;
            return kStatus_SDMMC_SendCardCmdFailed;
        }
        if ((cardCmd->response[0] & SDMMC_R1_READY_FOR_DATA)
             && (SDMMC_R1_CURRENT_STATE(cardCmd->response[0]) != SDMMC_R1_STATE_PRG))
        {
            break;
        }

        SDMMC_DelayMsec(1);
    } while(timeout--);

    if (!timeout)
    {
#if defined FSL_CARD_DRIVER_USING_DYNALLOC
        OSA_MemFree(cardCmd);
#endif
        cardCmd = NULL;
        return kStatus_SDMMC_TimeoutError;
    }

#if defined FSL_CARD_DRIVER_USING_DYNALLOC
    OSA_MemFree(cardCmd);
#endif
    cardCmd = NULL;
    return err;
}

/*FUNCTION****************************************************************
 *
 * Function Name: SDMMC_Shutdown
 * Description: destory initialized card and shutdown the corresponding
 * host controller
 *
 *END*********************************************************************/
void SD_Shutdown(sd_t *card)
{
    SD_SelectCard(card, false);
}


/*FUNCTION****************************************************************
 *
 * Function Name: SDMMC_SendApplicationCmd
 * Description: send application command to card
 *
 *END*********************************************************************/
static sdmmc_status_t SD_SendApplicationCmd(sd_t *card)
{
    assert(card);
    host_t *host = card->host;
    assert(host);
    card_cmd_t *cardCmd = 0;
    sdmmc_status_t ret = kStatus_SDMMC_NoError;
#if ! defined FSL_CARD_DRIVER_USING_DYNALLOC
    card_cmd_t command = {0};
    cardCmd = &command;
#endif
#if defined FSL_CARD_DRIVER_USING_DYNALLOC
    cardCmd = (card_cmd_t *)OSA_MemAllocZero(sizeof(card_cmd_t));
    if (cardCmd == NULL)
    {
        return kStatus_SDMMC_OutOfMemory;
    }
#endif

    cardCmd->cmdIndex = kSdmmcAppCmd;
    cardCmd->argument = 0;
    //if (card->cardType != kCardTypeUnknown)
    //{
        cardCmd->argument = card->rca << 16;
   // }
    cardCmd->respType = kSdmmcRespTypeR1;
    host->currentCmd = cardCmd;
    host->currentData = 0;
    if ((kStatus_SDMMC_NoError != SDMMC_SendCmdBlocking(host, FSL_CARD_COMMAND_TIMEOUT))
            || (kStatus_SDMMC_NoError != SDMMC_CheckR1Response(cardCmd)))
    {
        if (cardCmd->errors & CARD_CMD_ERR_CMD_TIMEOUT)
        {
            ret = kStatus_SDMMC_TimeoutError;
        }
        else
        {
            ret = kStatus_SDMMC_SendCardCmdFailed;
        }
#if defined FSL_CARD_DRIVER_USING_DYNALLOC
        OSA_MemFree(cardCmd);
#endif
        cardCmd = NULL;
        return ret;
    }

    if (!(cardCmd->response[0] & SDMMC_R1_APP_CMD))
    {
#if defined FSL_CARD_DRIVER_USING_DYNALLOC
        OSA_MemFree(cardCmd);
#endif
        cardCmd = NULL;
        return kStatus_SDMMC_CardNotSupport;
    }
#if defined FSL_CARD_DRIVER_USING_DYNALLOC
    OSA_MemFree(cardCmd);
#endif
    cardCmd = NULL;
    return kStatus_SDMMC_NoError;
}

/*FUNCTION****************************************************************
 *
 * Function Name: SDMMC_GoIdle
 * Description: reset all cards to idle state
 *
 *END*********************************************************************/
static sdmmc_status_t SD_GoIdle(sd_t *card)
{
    assert(card);
    host_t *host = card->host;
    card_cmd_t *cardCmd = 0;
    sdmmc_status_t err;
#if ! defined FSL_CARD_DRIVER_USING_DYNALLOC
    card_cmd_t command = {0};
    cardCmd = &command;
#endif
#if defined FSL_CARD_DRIVER_USING_DYNALLOC
    cardCmd = (card_cmd_t *)OSA_MemAllocZero(sizeof(card_cmd_t));
    if (cardCmd == NULL)
    {
        return kStatus_SDMMC_OutOfMemory;
    }
#endif

    cardCmd->cmdIndex = kSdmmcGoIdleState;
    host->currentCmd = cardCmd;
    host->currentData = 0;
    err = SDMMC_SendCmdBlocking(host, FSL_CARD_COMMAND_TIMEOUT);
#if defined FSL_CARD_DRIVER_USING_DYNALLOC
    OSA_MemFree(cardCmd);
#endif
    cardCmd = NULL;
    return err;
}

#if ! defined BSP_HOST_ENABLE_AUTOCMD12
/*FUNCTION****************************************************************
 *
 * Function Name: SDMMC_StopTransmission
 * Description:  Send stop transmission command to card to stop ongoing
 * data transferring.
 *
 *END*********************************************************************/
static sdmmc_status_t SD_StopTransmission(sd_t* card)
{
    host_t *host = card->host;
    card_cmd_t *cardCmd = 0;
    sdmmc_status_t err = kStatus_SDMMC_NoError;
#if ! defined FSL_CARD_DRIVER_USING_DYNALLOC
    card_cmd_t command = {0};
    cardCmd = &command;
#endif
#if defined FSL_CARD_DRIVER_USING_DYNALLOC
    cardCmd = (card_cmd_t *)OSA_MemAllocZero(sizeof(card_cmd_t));
    if (cardCmd == NULL)
    {
        return kStatus_SDMMC_OutOfMemory;
    }
#endif

    cardCmd->cmdIndex = kSdmmcStopTransmission;
    cardCmd->flags |= CARD_CMD_FLAGS_STOP_TRANS;
    cardCmd->argument = 0;
    cardCmd->respType = kSdmmcRespTypeR1b;
    
    host->currentCmd = cardCmd;
    host->currentData = 0;
    if ((kStatus_SDMMC_NoError != SDMMC_SendCmdBlocking(host, FSL_CARD_COMMAND_TIMEOUT))
            || (kStatus_SDMMC_NoError != SDMMC_CheckR1Response(cardCmd)))
    {
#if defined FSL_CARD_DRIVER_USING_DYNALLOC
        OSA_MemFree(cardCmd);
#endif
        cardCmd = NULL;
        return kStatus_SDMMC_SendCardCmdFailed;
    }

#if defined FSL_CARD_DRIVER_USING_DYNALLOC
    OSA_MemFree(cardCmd);
#endif
    cardCmd = NULL;
    return err;
}
#endif

/*FUNCTION****************************************************************
 *
 * Function Name: SDMMC_SetBlockSize
 * Description:  Set the block length in bytes for SDSC cards. For SDHC cards,
 * it does not affect memory read or write commands, always 512 bytes fixed
 * block length is used.
 *
 *END*********************************************************************/
static sdmmc_status_t SD_SetBlockSize(sd_t *card, uint32_t blockSize)
{
    host_t *host = card->host;
    card_cmd_t *cardCmd = 0;
#if ! defined FSL_CARD_DRIVER_USING_DYNALLOC
    card_cmd_t command = {0};
    cardCmd = &command;
#endif

#if defined FSL_CARD_DRIVER_USING_DYNALLOC
    cardCmd = (card_cmd_t *)OSA_MemAllocZero(sizeof(card_cmd_t));
    if (cardCmd == NULL)
    {
        return kStatus_SDMMC_OutOfMemory;
    }
#endif
    cardCmd->cmdIndex = kSdmmcSetBlockLen;
    cardCmd->argument = blockSize;
    cardCmd->respType = kSdmmcRespTypeR1;
    host->currentCmd = cardCmd;
    host->currentData = 0;
    if ((kStatus_SDMMC_NoError != SDMMC_SendCmdBlocking(host, FSL_CARD_COMMAND_TIMEOUT))
            || (kStatus_SDMMC_NoError != SDMMC_CheckR1Response(cardCmd)))
    {
#if defined FSL_CARD_DRIVER_USING_DYNALLOC
        OSA_MemFree(cardCmd);
#endif
        cardCmd = NULL;
        return kStatus_SDMMC_SendCardCmdFailed;
    }

#if defined FSL_CARD_DRIVER_USING_DYNALLOC
    OSA_MemFree(cardCmd);
#endif
    cardCmd = NULL;
    return kStatus_SDMMC_NoError;
}

/*FUNCTION****************************************************************
 *
 * Function Name: SD_DecodeCsd
 * Description: decode csd register
 *
 *END*********************************************************************/
static void SD_DecodeCsd(uint32_t *rawCsd, sd_t *card)
{
    sd_csd_t *csd;
    assert(rawCsd);
    assert(card);
    csd = &(card->csd);
    csd->csdStructure = (uint8_t)((rawCsd[3] & 0xC0000000U) >> 30);
    csd->taac = (uint8_t)((rawCsd[3] & 0xFF0000) >> 16);
    csd->nsac = (uint8_t)((rawCsd[3] & 0xFF00) >> 8);
    csd->tranSpeed = (uint8_t)(rawCsd[3] & 0xFF);
    csd->ccc = (uint16_t)((rawCsd[2] & 0xFFF00000U) >> 20);
    csd->readBlkLen = (uint8_t)((rawCsd[2] & 0xF0000) >> 16);
    if (rawCsd[2] & 0x8000)
    {
        csd->flags |= SD_CSD_READ_BL_PARTIAL;
    }
    if (rawCsd[2] & 0x4000)
    {
        csd->flags |= SD_CSD_WRITE_BLK_MISALIGN;
    }
    if (rawCsd[2] & 0x2000)
    {
        csd->flags |= SD_CSD_READ_BLK_MISALIGN;
    }
    if (rawCsd[2] & 0x1000)
    {
        csd->flags |= SD_CSD_DSR_IMP;
    }
    if (csd->csdStructure == 0)
    {
        csd->cSize = (uint32_t)((rawCsd[2] & 0x3FF) << 2);
        csd->cSize |= (uint32_t)((rawCsd[1] & 0xC0000000U) >> 30);
        csd->vddRCurrMin = (uint8_t)((rawCsd[1] & 0x38000000) >> 27);
        csd->vddRCurrMax = (uint8_t)((rawCsd[1] & 0x7000000) >> 24);
        csd->vddWCurrMin = (uint8_t)((rawCsd[1] & 0xE00000) >> 20);
        csd->vddWCurrMax = (uint8_t)((rawCsd[1] & 0x1C0000) >> 18);
        csd->cSizeMult = (uint8_t)((rawCsd[1] & 0x38000) >> 15);
        card->blockCount = (csd->cSize + 1) <<  (csd->cSizeMult + 2);
        card->blockSize = (1 << (csd->readBlkLen));
        if (card->blockSize != FSL_CARD_DEFAULT_BLOCK_SIZE)
        {
            card->blockCount = card->blockCount * card->blockSize; 
            card->blockSize = FSL_CARD_DEFAULT_BLOCK_SIZE;
            card->blockCount = card->blockCount / card->blockSize;
        }
    }
    else if (csd->csdStructure == 1)
    {
        card->blockSize = FSL_CARD_DEFAULT_BLOCK_SIZE;
        csd->cSize = (uint32_t)((rawCsd[2] & 0x3F) << 16);
        csd->cSize |= (uint32_t)((rawCsd[1] & 0xFFFF0000U) >> 16);
        if (csd->cSize >= 0xFFFF)
        {
            card->caps |= SD_CARD_CAPS_SDXC;
        }
        card->blockCount = (csd->cSize + 1) * 1024;
    }

    if ((uint8_t)((rawCsd[1] & 0x4000) >> 14))
    {
        csd->flags |= SD_CSD_ERASE_BLK_ENABLED;
    }

    csd->sectorSize = (uint8_t)((rawCsd[1] & 0x3F80) >> 7);
    csd->wpGrpSize = (uint8_t)(rawCsd[1] & 0x7F);
    if ((uint8_t)(rawCsd[0] & 0x80000000U))
    {
        csd->flags |= SD_CSD_WP_GRP_ENABLED;
    }
    csd->r2wFactor = (uint8_t)((rawCsd[0] & 0x1C000000) >> 26);
    csd->writeBlkLen = (uint8_t)((rawCsd[0] & 0x3C00000) >> 22);
    if ((uint8_t)((rawCsd[0] & 0x200000) >> 21))
    {
        csd->flags |= SD_CSD_WRITE_BL_PARTIAL;
    }
    if ((uint8_t)((rawCsd[0] & 0x8000) >> 15))
    {
        csd->flags |= SD_CSD_FILE_FORMAT_GROUP;
    }
    if ((uint8_t)((rawCsd[0] & 0x4000) >> 14))
    {
        csd->flags |= SD_CSD_COPY;
    }
    if ((uint8_t)((rawCsd[0] & 0x2000) >> 13))
    {
        csd->flags |= SD_CSD_PERM_WRITE_PROTECT;
    }
    if ((uint8_t)((rawCsd[0] & 0x1000) >> 12))
    {
        csd->flags |= SD_CSD_TMP_WRITE_PROTECT;
    }
    csd->fileFormat = (uint8_t)((rawCsd[0] & 0xC00) >> 10);
}

/*FUNCTION****************************************************************
 *
 * Function Name: SD_DecodeCid
 * Description: decode cid register
 *
 *END*********************************************************************/
static void SD_DecodeCid(uint32_t *rawCid, sd_t *card)
{
    sd_cid_t *cid;
    assert(rawCid);
    assert(card);
    cid = &(card->cid);

    cid->mid = (uint8_t)((rawCid[3] & 0xFF000000) >> 24);

    cid->oid = (uint16_t)((rawCid[3] & 0xFFFF00) >> 8);

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
 * Function Name: SD_SendRca
 * Description: send rca command to card to get relative card address
 *
 *END*********************************************************************/
static sdmmc_status_t SD_SendRca(sd_t *card)
{
    host_t *host = card->host;
    card_cmd_t *cardCmd = 0;
#if ! defined FSL_CARD_DRIVER_USING_DYNALLOC
    card_cmd_t command = {0};
    cardCmd = &command;
#endif
    assert(card);
#if defined FSL_CARD_DRIVER_USING_DYNALLOC
    cardCmd = (card_cmd_t *)OSA_MemAllocZero(sizeof(card_cmd_t));
    if (cardCmd == NULL)
    {
        return kStatus_SDMMC_OutOfMemory;
    }
#endif

    cardCmd->cmdIndex = kSdSendRelativeAddr;
    cardCmd->argument = 0;
    cardCmd->respType = kSdmmcRespTypeR6;
    host->currentCmd = cardCmd;
    host->currentData = 0;
    if (kStatus_SDMMC_NoError == SDMMC_SendCmdBlocking(host, FSL_CARD_COMMAND_TIMEOUT))
    {
        card->rca = cardCmd->response[0] >> 16;
#if defined FSL_CARD_DRIVER_USING_DYNALLOC
        OSA_MemFree(cardCmd);
#endif
        cardCmd = NULL;
        return kStatus_SDMMC_NoError;
    }
    
#if defined FSL_CARD_DRIVER_USING_DYNALLOC
    OSA_MemFree(cardCmd);
#endif
    cardCmd = NULL;
    return kStatus_SDMMC_SendCardCmdFailed;
}

/*FUNCTION****************************************************************
 *
 * Function Name: SD_Switch
 * Description: send switch command to card
 *
 *END*********************************************************************/
static sdmmc_status_t SD_Switch(sd_t *card,
                       uint32_t mode,
                       uint32_t group,
                       uint32_t value,
                       uint32_t *resp)
{
    host_t *host = card->host;
    card_cmd_t *cardCmd = 0;
    card_data_t cardData = {0};
#if ! defined FSL_CARD_DRIVER_USING_DYNALLOC
    card_cmd_t command = {0};
    cardCmd = &command;
#endif
    assert(card);
#if defined FSL_CARD_DRIVER_USING_DYNALLOC
    cardCmd = (card_cmd_t *)OSA_MemAllocZero(sizeof(card_cmd_t));
    if (cardCmd == NULL)
    {
        return kStatus_SDMMC_OutOfMemory;
    }
#endif

    
    cardCmd->cmdIndex = kSdSwitchFunction;
    cardCmd->argument = mode << 31 | 0x00FFFFFF;
    cardCmd->argument &= ~((uint32_t)(0xF) << (group * 4));
    cardCmd->argument |= value << (group * 4);
    cardCmd->respType = kSdmmcRespTypeR1;
    

    cardData.blockSize = 64;
    cardData.blockCount = 1;
    cardData.buffer = resp;
    cardData.flags |= CARD_DATA_FLAGS_DATA_READ;

    if (kStatus_SDMMC_NoError != SD_SetBlockSize(card, cardData.blockSize))
    {
#if defined FSL_CARD_DRIVER_USING_DYNALLOC
        OSA_MemFree(cardCmd);
#endif
        cardCmd = NULL;
        return kStatus_SDMMC_SetCardBlockSizeFailed;
    }
    
    host->currentCmd = cardCmd;
    host->currentData = &cardData;
    if (kStatus_SDMMC_NoError != SDMMC_SendCmdBlocking(host, FSL_CARD_COMMAND_TIMEOUT)
        || (kStatus_SDMMC_NoError != SDMMC_CheckR1Response(cardCmd))
        || (kStatus_SDMMC_NoError != SDMMC_WaitDataTransferComplete(host, FSL_CARD_COMMAND_TIMEOUT)))
    {
#if defined FSL_CARD_DRIVER_USING_DYNALLOC
        OSA_MemFree(cardCmd);
#endif
        cardCmd = NULL;
        return kStatus_SDMMC_SendCardCmdFailed;
    }
    /*if (kStatus_SDMMC_NoError != SDMMC_WaitDataTransferComplete(host, FSL_CARD_COMMAND_TIMEOUT))
    //if (kStatus_SDMMC_NoError != SDMMC_WaitDataTransferComplete(host, 1000))
    {
#if defined FSL_CARD_DRIVER_USING_DYNALLOC
        OSA_MemFree(cardCmd);
#endif
        cardCmd = NULL;
        return kStatus_SDMMC_SendCardCmdFailed;
    }*/
#if defined FSL_CARD_DRIVER_USING_DYNALLOC
    OSA_MemFree(cardCmd);
#endif
    cardCmd = NULL;
    return kStatus_SDMMC_NoError;
}

/*FUNCTION****************************************************************
 *
 * Function Name: SD_DecodeScr
 * Description: decode scr register
 *
 *END*********************************************************************/
static void SD_DecodeScr(uint32_t *rawScr, sd_t *card)
{
    sd_scr_t *scr;
    assert(rawScr);
    assert(card);

    scr = &(card->scr);
    scr->scrStructure = (uint8_t)((rawScr[0] & 0xF0000000U) >> 28);
    scr->sdSpec = (uint8_t)((rawScr[0] & 0xF000000) >> 24);
    if ((uint8_t)((rawScr[0] & 0x800000) >> 23))
    {
        scr->flags |= SD_SCR_DATA_STAT_AFTER_ERASE;
    }
    scr->sdSecurity = (uint8_t)((rawScr[0] & 0x700000) >> 20);
    scr->sdBusWidths = (uint8_t)((rawScr[0] & 0xF0000) >> 16);
    if ((uint8_t)((rawScr[0] & 0x8000) >> 15))
    {
        scr->flags |= SD_SCR_SD_SPEC3;
    }
    scr->exSecurity = (uint8_t)((rawScr[0] & 0x7800) >> 10);
    scr->cmdSupport = (uint8_t)(rawScr[0] & 0x3);
    scr->reservedForMan = rawScr[1];

    switch(scr->sdSpec)
    {
        case 0:
            card->version = SD_SPEC_VERSION_1_0;
            break;
        case 1:
            card->version = SD_SPEC_VERSION_1_1;
            break;
        case 2:
            card->version = SD_SPEC_VERSION_2_0;
            if (card->scr.flags & SD_SCR_SD_SPEC3)
            {
                card->version = SD_SPEC_VERSION_3_0;
            }
            break;
        default:
            break;
    }
    if (card->scr.sdBusWidths & SD_SCR_BUS_WIDTHS_4BIT)
    {
        card->caps |= SD_CARD_CAPS_BUSWIDTH_4BITS;
    }
}

/*FUNCTION****************************************************************
 *
 * Function Name: SD_SendScr
 * Description: fetch scr register from card
 *
 *END*********************************************************************/
static sdmmc_status_t SD_SendScr(sd_t *card)
{
    host_t *host = card->host;
    card_cmd_t *cardCmd = 0;
    card_data_t cardData = {0};
    sdmmc_status_t err = kStatus_SDMMC_NoError;
    uint32_t rawScr[2] = {0};

#if ! defined FSL_CARD_DRIVER_USING_DYNALLOC
    card_cmd_t command = {0};
    cardCmd = &command;
#endif
    assert(card);
#if defined FSL_CARD_DRIVER_USING_DYNALLOC
    cardCmd = (card_cmd_t *)OSA_MemAllocZero(sizeof(card_cmd_t));
    if (cardCmd == NULL)
    {
        return kStatus_SDMMC_OutOfMemory;
    }
#endif

    err = SD_SendApplicationCmd(card);
    if (err)
    {
#if defined FSL_CARD_DRIVER_USING_DYNALLOC
        OSA_MemFree(cardCmd);
#endif
        cardCmd = NULL;
        return err;
    }

    
    
    cardCmd->cmdIndex = kSdAppSendScr;
    cardCmd->respType = kSdmmcRespTypeR1;
    cardCmd->argument = 0;
    host->currentCmd = cardCmd;

    cardData.blockSize = 8;
    cardData.blockCount = 1;
    cardData.buffer = rawScr;
    cardData.flags |= CARD_DATA_FLAGS_DATA_READ;
    host->currentData = &cardData;

    if ((kStatus_SDMMC_NoError != SDMMC_SendCmdBlocking(host, FSL_CARD_COMMAND_TIMEOUT))
        || (kStatus_SDMMC_NoError != SDMMC_CheckR1Response(cardCmd))
        || (kStatus_SDMMC_NoError != SDMMC_WaitDataTransferComplete(host, FSL_CARD_COMMAND_TIMEOUT)))
    {
#if defined FSL_CARD_DRIVER_USING_DYNALLOC
        OSA_MemFree(cardCmd);
#endif
        cardCmd = NULL;
        return kStatus_SDMMC_SendCardCmdFailed;
    }
    rawScr[0] = swap_be32(rawScr[0]);
    rawScr[1] = swap_be32(rawScr[1]);
    memcpy(card->rawScr, rawScr, sizeof(card->rawScr));
    SD_DecodeScr(rawScr, card);
#if defined FSL_CARD_DRIVER_USING_DYNALLOC
    OSA_MemFree(cardCmd);
#endif
    cardCmd = NULL;
    return kStatus_SDMMC_NoError;
}

/*FUNCTION****************************************************************
 *
 * Function Name: SD_SwitchHighspeed
 * Description: switch high speed mode of the specific card
 *
 *END*********************************************************************/
static sdmmc_status_t SD_SwitchHighspeed(sd_t *card)
{
    uint32_t response[16] = {0};
    sdmmc_status_t err = kStatus_SDMMC_NoError;
    assert(card);

    if ((card->version < SD_SPEC_VERSION_1_0)
         || (!(card->csd.ccc & SD_CCC_SWITCH)))
    {
        return kStatus_SDMMC_CardNotSupport;
    }

    err = SD_Switch(card, kSdSwitchCheck, 0, 1, response);
    if (err)
    {
        return err;
    }

    if ((!(swap_be32(response[3]) & 0x10000)) ||
        ((swap_be32(response[4]) & 0x0f000000) == 0x0F000000))
    {
        return kStatus_SDMMC_CardNotSupport;
    }

    err = SD_Switch(card, kSdSwitchSet, 0, 1, response);
    if (err)
    {
        return err;
    }

    if ((swap_be32(response[4]) & 0x0f000000) != 0x01000000)
    {
        err = kStatus_SDMMC_SwitchFunctionCmdFailed;
    }

    return err;

}

/*FUNCTION****************************************************************
 *
 * Function Name: SD_SetBusWidth
 * Description: set desired buswidth
 *
 *END*********************************************************************/
static uint32_t SD_SetBusWidth(sd_t *card, sd_buswidth_t busWidth)
{
    host_t *host = card->host;
    card_cmd_t *cardCmd = 0;
    sdmmc_status_t err = kStatus_SDMMC_NoError;
#if ! defined FSL_CARD_DRIVER_USING_DYNALLOC
    card_cmd_t command = {0};
    cardCmd = &command;
#endif
    assert(card);
#if defined FSL_CARD_DRIVER_USING_DYNALLOC
    cardCmd = (card_cmd_t *)OSA_MemAllocZero(sizeof(card_cmd_t));
    if (cardCmd == NULL)
    {
        return kStatus_SDMMC_OutOfMemory;
    }
#endif

    err = SD_SendApplicationCmd(card);
    if (err != kStatus_SDMMC_NoError)
    {
#if defined FSL_CARD_DRIVER_USING_DYNALLOC
        OSA_MemFree(cardCmd);
#endif
        cardCmd = NULL;
        return err;
    }

    cardCmd->cmdIndex = kSdAppSetBusWdith;
    cardCmd->respType = kSdmmcRespTypeR1;
    cardCmd->argument = busWidth;
    host->currentCmd = cardCmd;
    host->currentData = 0;

    if (kStatus_SDMMC_NoError != SDMMC_SendCmdBlocking(host, FSL_CARD_COMMAND_TIMEOUT)
        || (kStatus_SDMMC_NoError != SDMMC_CheckR1Response(cardCmd)))
    {
#if defined FSL_CARD_DRIVER_USING_DYNALLOC
        OSA_MemFree(cardCmd);
#endif
        cardCmd = NULL;
        return kStatus_SDMMC_SendCardCmdFailed;
    }

#if defined FSL_CARD_DRIVER_USING_DYNALLOC
    OSA_MemFree(cardCmd);
#endif
    cardCmd = NULL;
    return err;
}



/*FUNCTION****************************************************************
 *
 * Function Name: SD_SendCsd
 * Description: get csd from card
 *
 *END*********************************************************************/
static sdmmc_status_t SD_SendCsd(sd_t * card)
{
    host_t *host = card->host;
    card_cmd_t *cardCmd = 0;
#if ! defined FSL_CARD_DRIVER_USING_DYNALLOC
    card_cmd_t command = {0};
    cardCmd = &command;
#endif
    assert(card);
#if defined FSL_CARD_DRIVER_USING_DYNALLOC
    cardCmd = (card_cmd_t *)OSA_MemAllocZero(sizeof(card_cmd_t));
    if (cardCmd == NULL)
    {
        return kStatus_SDMMC_OutOfMemory;
    }
#endif

    cardCmd->cmdIndex = kSdmmcSendCsd;
    cardCmd->argument = card->rca << 16;
    cardCmd->respType = kSdmmcRespTypeR2;
    host->currentCmd = cardCmd;
    host->currentData = 0;
    if (kStatus_SDMMC_NoError == SDMMC_SendCmdBlocking(host, 10000U))
    //if (kStatus_SDMMC_NoError == SDMMC_SendCmdBlocking(host, FSL_CARD_COMMAND_TIMEOUT))
    {
        memcpy(card->rawCsd, cardCmd->response, sizeof(card->rawCsd));
        /*The response is from bit 127:8 in R2, corrisponding to cardCmd->response[3]
        :cardCmd->response[0][31:8]*/
        SD_DecodeCsd(cardCmd->response, card);

#if defined FSL_CARD_DRIVER_USING_DYNALLOC
        OSA_MemFree(cardCmd);
#endif
        cardCmd = NULL;
        return kStatus_SDMMC_NoError;
    }

#if defined FSL_CARD_DRIVER_USING_DYNALLOC
    OSA_MemFree(cardCmd);
#endif
    cardCmd = NULL;
    return kStatus_SDMMC_SendCardCmdFailed;
}

/*FUNCTION****************************************************************
 *
 * Function Name: SD_AllSendCid
 * Description: send all_send_cid command
 *
 *END*********************************************************************/
static sdmmc_status_t SD_AllSendCid(sd_t *card)
{
    host_t *host = card->host;
    card_cmd_t *cardCmd = 0;
#if ! defined FSL_CARD_DRIVER_USING_DYNALLOC
    card_cmd_t command = {0};
    cardCmd = &command;
#endif
    assert(card);
#if defined FSL_CARD_DRIVER_USING_DYNALLOC
    cardCmd = (card_cmd_t *)OSA_MemAllocZero(sizeof(card_cmd_t));
    if (cardCmd == NULL)
    {
        return kStatus_SDMMC_OutOfMemory;
    }
#endif

    cardCmd->cmdIndex = kSdmmcAllSendCid;
    cardCmd->argument = 0;
    cardCmd->respType = kSdmmcRespTypeR2;
    card->host->currentCmd = cardCmd;
    host->currentData = 0;
    if (kStatus_SDMMC_NoError == SDMMC_SendCmdBlocking(host, FSL_CARD_COMMAND_TIMEOUT))
    {
        memcpy(card->rawCid, cardCmd->response, sizeof(card->rawCid));
        SD_DecodeCid(cardCmd->response, card);      
#if defined FSL_CARD_DRIVER_USING_DYNALLOC
        OSA_MemFree(cardCmd);
#endif
        cardCmd = NULL;
        return kStatus_SDMMC_NoError;
    }

#if defined FSL_CARD_DRIVER_USING_DYNALLOC
    OSA_MemFree(cardCmd);
#endif
    cardCmd = NULL;
    return kStatus_SDMMC_SendCardCmdFailed;
}

/*FUNCTION****************************************************************
 *
 * Function Name: SD_Init
 * Description: initialize SD memory card
 *
 *END*********************************************************************/
static sdmmc_status_t SD_Init(sd_t *card)
{
    assert(card);
    sdmmc_status_t err = kStatus_SDMMC_NoError;
  //  card->cardType = kCardTypeSd;

    if (kStatus_SDMMC_NoError != SD_AllSendCid(card))
    {
        return kStatus_SDMMC_AllSendCidCmdFailed;
    }

    if (kStatus_SDMMC_NoError != SD_SendRca(card))
    {
        return kStatus_SDMMC_SendRcaCmdFailed;
    }

    if (kStatus_SDMMC_NoError != SD_SendCsd(card))
    {
        return kStatus_SDMMC_SendCsdCmdFailed;
    }

    if (kStatus_SDMMC_NoError != SD_SelectCard(card, true))
    {
        return kStatus_SDMMC_SelectCardCmdFailed;
    }

    if (kStatus_SDMMC_NoError != SD_SendScr(card))
    {
        return kStatus_SDMMC_SendScrCmdFailed;
    }

    if (kStatus_SDMMC_NoError != SDMMC_ConfigClock(card->host, SD_CLK_25MHZ))
    {
        return kStatus_SDMMC_SetCardBusClockFailed;
    }

    if (DOES_HOST_SUPPORT_4BITS(card->host) && DOES_CARD_SUPPORT_4BITS(card))
    {
        if (kStatus_SDMMC_NoError != SD_SetBusWidth(card, kSdBusWidth4Bit))
        {
            return kStatus_SDMMC_SetBusWidthCmdFailed;
        }
        SDMMC_SetHostBusWidth(card->host, kSdhcDtw4Bit);
    }
    if (card->host->capability->supportMask & SDHC_SUPPORT_HIGHSPEED)
    //if (DOES_HOST_SUPPORT_HIGHSPEED(card->host))
    {
        err = SD_SwitchHighspeed(card);
        if ((err != kStatus_SDMMC_NoError) && (kStatus_SDMMC_CardNotSupport != err))
        {
            return kStatus_SDMMC_SwitchFunctionCmdFailed;
        }
        else if (err == kStatus_SDMMC_NoError)
        { 
            if (kStatus_SDMMC_NoError != SDMMC_ConfigClock(card->host, SD_CLK_50MHZ))
            {
                return kStatus_SDMMC_SetCardBusClockFailed;
            }
        }
        else
        {
            err = kStatus_SDMMC_NoError;
        }
    }

    if (SD_SetBlockSize(card, FSL_CARD_DEFAULT_BLOCK_SIZE))
    {
        err = kStatus_SDMMC_SetCardBlockSizeFailed;
    }
    if (err != kStatus_SDMMC_NoError)
    {
        SDMMC_AllocStaticMemory(card->host);
    }
    return err;
}

/*FUNCTION****************************************************************
 *
 * Function Name: SD_AppSendOpCond
 * Description: Send host capacity support information and asks the accessed
 * card to send its operating condition register content.
 *
 *END*********************************************************************/
static sdmmc_status_t SD_AppSendOpCond(sd_t *card, uint32_t acmd41Arg)
{
    host_t *host = card->host;
    card_cmd_t *cardCmd = 0;
    sdmmc_status_t err;
    uint32_t i = FSL_CARD_MAX_VOLT_RETRIES;

#if ! defined FSL_CARD_DRIVER_USING_DYNALLOC
    card_cmd_t command = {0};
    cardCmd = &command;
#endif
    assert(card);
#if defined FSL_CARD_DRIVER_USING_DYNALLOC
    cardCmd = (card_cmd_t *)OSA_MemAllocZero(sizeof(card_cmd_t));
    if (cardCmd == NULL)
    {
        return kStatus_SDMMC_OutOfMemory;
    }
#endif

    cardCmd->cmdIndex = kSdAppSendOpCond;
    cardCmd->argument = acmd41Arg;
    cardCmd->respType = kSdmmcRespTypeR3;
    
    while (i--)
    {
        err = SD_SendApplicationCmd(card);
        if (err != kStatus_SDMMC_NoError)
        {
#if defined FSL_CARD_DRIVER_USING_DYNALLOC
            OSA_MemFree(cardCmd);
#endif
            cardCmd = NULL;
            return err;
        }
        
        host->currentCmd = cardCmd;
        host->currentData = 0;
        if (kStatus_SDMMC_NoError != SDMMC_SendCmdBlocking(host, FSL_CARD_COMMAND_TIMEOUT))
        {
#if defined FSL_CARD_DRIVER_USING_DYNALLOC
            OSA_MemFree(cardCmd);
#endif
            cardCmd = NULL;
            return err;
        }

        if (cardCmd->response[0] & SDMMC_CARD_BUSY)
        {
            if (cardCmd->response[0] & SD_OCR_CCS)
            {
                card->caps |= SD_CARD_CAPS_HIGHCAPACITY;
            }
            err = kStatus_SDMMC_NoError;
            card->ocr = cardCmd->response[0];
            break;
        }
        err = kStatus_SDMMC_TimeoutError;
        SDMMC_DelayMsec(1);
    }

#if defined FSL_CARD_DRIVER_USING_DYNALLOC
    OSA_MemFree(cardCmd);
#endif
    cardCmd = NULL;
    return err;
}

/*FUNCTION****************************************************************
 *
 * Function Name: SD_SendIfCond
 * Description: check card interface condition, which includes host supply
 * voltage information and asks the card whether card supports voltage.
 *
 *END*********************************************************************/
static sdmmc_status_t SD_SendIfCond(sd_t *card)
{
    host_t *host = card->host;
    card_cmd_t *cardCmd = 0;
    sdmmc_status_t err = kStatus_SDMMC_NoError;
#if ! defined FSL_CARD_DRIVER_USING_DYNALLOC
    card_cmd_t command = {0};
    cardCmd = &command;
#endif
    assert(card);
#if defined FSL_CARD_DRIVER_USING_DYNALLOC
    cardCmd = (card_cmd_t *)OSA_MemAllocZero(sizeof(card_cmd_t));
    if (cardCmd == NULL)
    {
        return kStatus_SDMMC_OutOfMemory;
    }
#endif

    cardCmd->cmdIndex = kSdSendIfCond;
    cardCmd->argument = 0x1AA;
    cardCmd->respType = kSdmmcRespTypeR7;
    card->host->currentCmd = cardCmd;
    host->currentData = 0;

    if (kStatus_SDMMC_NoError != SDMMC_SendCmdBlocking(host, FSL_CARD_COMMAND_TIMEOUT))
    {
        err = kStatus_SDMMC_SendCardCmdFailed;
    }
    else if ((cardCmd->response[0] & 0xFF) != 0xAA)
    {
        err = kStatus_SDMMC_CardNotSupport;
    }
#if defined FSL_CARD_DRIVER_USING_DYNALLOC
    OSA_MemFree(cardCmd);
#endif
    cardCmd = NULL;
    return err;
}

/*FUNCTION****************************************************************
 *
 * Function Name: SD_Read
 * Description: read data from specific SD card
 *
 *END*********************************************************************/
static sdmmc_status_t SD_Read(sd_t *card,
                     uint8_t *buffer,
                     uint32_t startBlock,
                     uint32_t blockSize,
                     uint32_t blockCount)
{
    host_t *host = card->host;
    card_cmd_t *cardCmd = 0;
    card_data_t cardData = {0};
#if ! defined FSL_CARD_DRIVER_USING_DYNALLOC
    card_cmd_t command = {0};
    cardCmd = &command;
#endif

    assert(card);
    assert(buffer);
    assert(blockCount);
    assert(blockSize);
    assert(blockSize == FSL_CARD_DEFAULT_BLOCK_SIZE);

    if ((IS_HIGHCAPACITY_CARD(card) && (blockSize != 512))
         || (blockSize > card->blockSize)
         || (blockSize > card->host->capability->maxBlockLength)
         || (blockSize % 4))
    {
        return kStatus_SDMMC_BlockSizeHostNotSupport;
    }

#if defined FSL_CARD_DRIVER_USING_DYNALLOC
    cardCmd = (card_cmd_t *)OSA_MemAllocZero(sizeof(card_cmd_t));
    if (cardCmd == NULL)
    {
        return kStatus_SDMMC_OutOfMemory;
    }
#endif

    cardData.blockSize = blockSize;
    cardData.blockCount = blockCount;
    cardData.buffer = (uint32_t *)buffer;
    cardData.flags |= CARD_DATA_FLAGS_DATA_READ;
    host->currentData = &cardData;

    cardCmd->cmdIndex = kSdmmcReadMultipleBlock;
    if (cardData.blockCount == 1)
    {
        cardCmd->cmdIndex = kSdmmcReadSingleBlock;
    }
    cardCmd->argument = startBlock;
    if (!IS_HIGHCAPACITY_CARD(card))
    {
        cardCmd->argument *= cardData.blockSize;
    }
    cardCmd->respType = kSdmmcRespTypeR1;
    host->currentCmd = cardCmd;

    if ((kStatus_SDMMC_NoError != SDMMC_SendCmdBlocking(host, FSL_CARD_COMMAND_TIMEOUT))
        || (kStatus_SDMMC_NoError != SDMMC_CheckR1Response(cardCmd))
        || (kStatus_SDMMC_NoError != SDMMC_WaitDataTransferComplete(host, FSL_CARD_COMMAND_TIMEOUT)))
    {
#if defined FSL_CARD_DRIVER_USING_DYNALLOC
        OSA_MemFree(cardCmd);
#endif
        cardCmd = NULL;
        return kStatus_SDMMC_SendCardCmdFailed;
    }

#if ! defined BSP_FSL_SDHC_ENABLE_AUTOCMD12
    if (cardData.blockCount > 1)
    {
        if (kStatus_SDMMC_NoError != SD_StopTransmission(card))
        {
#if defined FSL_CARD_DRIVER_USING_DYNALLOC
            OSA_MemFree(cardCmd);
#endif
            cardCmd = NULL;
            return kStatus_SDMMC_StopTransmissionCmdFailed;
        }
    }
#endif
#if defined FSL_CARD_DRIVER_USING_DYNALLOC
    OSA_MemFree(cardCmd);
#endif
    cardCmd = NULL;
    return kStatus_SDMMC_NoError;
}

/*FUNCTION****************************************************************
 *
 * Function Name: SD_Write
 * Description: write data from specific card
 *
 *END*********************************************************************/
static sdmmc_status_t SD_Write(sd_t *card,
                      uint8_t *buffer,
                      uint32_t startBlock,
                      uint32_t blockSize,
                      uint32_t blockCount)
{
    host_t *host = card->host;
    card_cmd_t *cardCmd = 0;
    card_data_t cardData = {0};
#if ! defined FSL_CARD_DRIVER_USING_DYNALLOC
    card_cmd_t command = {0};
    cardCmd = &command;
#endif
    assert(card);
    assert(buffer);
    assert(blockCount);
    assert(blockSize);
    assert(blockSize == FSL_CARD_DEFAULT_BLOCK_SIZE);

    if ((IS_HIGHCAPACITY_CARD(card) && (blockSize != 512))
         || (blockSize > card->blockSize)
         || (blockSize > card->host->capability->maxBlockLength)
         || (blockSize % 4))
    {
        return kStatus_SDMMC_BlockSizeHostNotSupport;
    }

#if defined FSL_CARD_DRIVER_USING_DYNALLOC
    cardCmd = (card_cmd_t *)OSA_MemAllocZero(sizeof(card_cmd_t));
    if (cardCmd == NULL)
    {
        return kStatus_SDMMC_OutOfMemory;
    }
#endif

    cardData.blockSize = blockSize;
    cardData.blockCount = blockCount;
    cardData.buffer = (uint32_t *)buffer;
    host->currentData = &cardData;

    cardCmd->cmdIndex = kSdmmcWriteMultipleBlock;
    if (cardData.blockCount == 1)
    {
        cardCmd->cmdIndex = kSdmmcWriteBlock;
    }
    cardCmd->argument = startBlock;
    if (!IS_HIGHCAPACITY_CARD(card))
    {
        cardCmd->argument *= cardData.blockSize;
    }
    cardCmd->respType = kSdmmcRespTypeR1;
    host->currentCmd = cardCmd;

    if ((kStatus_SDMMC_NoError != SDMMC_SendCmdBlocking(host, FSL_CARD_COMMAND_TIMEOUT))
        || (kStatus_SDMMC_NoError != SDMMC_CheckR1Response(cardCmd))
        || (kStatus_SDMMC_NoError != SDMMC_WaitDataTransferComplete(host, FSL_CARD_COMMAND_TIMEOUT)))
    {
#if defined FSL_CARD_DRIVER_USING_DYNALLOC
        OSA_MemFree(cardCmd);
#endif
        cardCmd = NULL;
        return kStatus_SDMMC_SendCardCmdFailed;
    }

    if (cardData.blockCount > 1)
    {
#if ! defined BSP_FSL_SDHC_ENABLE_AUTOCMD12
        if (kStatus_SDMMC_NoError != SD_StopTransmission(card))
        {
#if defined FSL_CARD_DRIVER_USING_DYNALLOC
            OSA_MemFree(cardCmd);
#endif
            cardCmd = NULL;
            return kStatus_SDMMC_StopTransmissionCmdFailed;
        }
#endif
        if (kStatus_SDMMC_NoError != SD_SendStatus(card))
        {
#if defined FSL_CARD_DRIVER_USING_DYNALLOC
            OSA_MemFree(cardCmd);
#endif
            cardCmd = NULL;
            return kStatus_SDMMC_SendStatusCmdFailed;
        }
    }
#if defined FSL_CARD_DRIVER_USING_DYNALLOC
    OSA_MemFree(cardCmd);
#endif
    cardCmd = NULL;
    return kStatus_SDMMC_NoError;
}

/*FUNCTION****************************************************************
 *
 * Function Name: SD_Erase
 * Description: erase data for the given block range
 *
 *END*********************************************************************/
static sdmmc_status_t SD_Erase(sd_t *card, uint32_t startBlock, uint32_t blockCount)
{
    host_t *host = card->host;
    uint32_t s, e;
    card_cmd_t *cardCmd = 0;
#if ! defined FSL_CARD_DRIVER_USING_DYNALLOC
    card_cmd_t command = {0};
    cardCmd = &command;
#endif
    assert(card);
    assert(blockCount);
#if defined FSL_CARD_DRIVER_USING_DYNALLOC
    cardCmd = (card_cmd_t *)OSA_MemAllocZero(sizeof(card_cmd_t));
    if (cardCmd == NULL)
    {
        return kStatus_SDMMC_OutOfMemory;
    }
#endif

    s = startBlock;
    e = s + blockCount - 1;
    if (!IS_HIGHCAPACITY_CARD(card))
    {
        s = s * FSL_CARD_DEFAULT_BLOCK_SIZE;
        e = e * FSL_CARD_DEFAULT_BLOCK_SIZE;
    }
    cardCmd->cmdIndex = kSdEraseWrBlkStart;
    cardCmd->argument = s;
    cardCmd->respType = kSdmmcRespTypeR1;
    card->host->currentCmd = cardCmd;
    host->currentData = 0;
    if (kStatus_SDMMC_NoError != SDMMC_SendCmdBlocking(host, FSL_CARD_COMMAND_TIMEOUT)
            || (kStatus_SDMMC_NoError != SDMMC_CheckR1Response(cardCmd)))
    {
#if defined FSL_CARD_DRIVER_USING_DYNALLOC
        OSA_MemFree(cardCmd);
#endif
        cardCmd = NULL;
        return kStatus_SDMMC_SendCardCmdFailed;
    }

    cardCmd->cmdIndex = kSdEraseWrBlkEnd;
    cardCmd->argument = e;
    if ((kStatus_SDMMC_NoError != SDMMC_SendCmdBlocking(host, FSL_CARD_COMMAND_TIMEOUT))
        || (kStatus_SDMMC_NoError != SDMMC_CheckR1Response(cardCmd)))
    {
#if defined FSL_CARD_DRIVER_USING_DYNALLOC
        OSA_MemFree(cardCmd);
#endif
        cardCmd = NULL;
        return kStatus_SDMMC_SendCardCmdFailed;
    }

    cardCmd->cmdIndex = kSdmmcErase;
    cardCmd->argument = 0;
    cardCmd->respType = kSdmmcRespTypeR1b;
    if ((kStatus_SDMMC_NoError != SDMMC_SendCmdBlocking(host, FSL_CARD_COMMAND_TIMEOUT))
        || (kStatus_SDMMC_NoError != SDMMC_CheckR1Response(cardCmd)))
    {
#if defined FSL_CARD_DRIVER_USING_DYNALLOC
        OSA_MemFree(cardCmd);
#endif
        cardCmd = NULL;
        return kStatus_SDMMC_SendCardCmdFailed;
    }

#if defined FSL_CARD_DRIVER_USING_DYNALLOC
    OSA_MemFree(cardCmd);
#endif
    cardCmd = NULL;
    return kStatus_SDMMC_NoError;
}


/*FUNCTION****************************************************************
 *
 * Function Name: SDMMC_DecodeScr
 * Description: decode scr register
 *
 *END*********************************************************************/
static void SDMMC_DecodeScr(uint32_t *rawScr, sd_t *card)
{
    sd_scr_t *scr;
    assert(rawScr);
    assert(card);

    scr = &(card->scr);
    scr->scrStructure = (uint8_t)((rawScr[0] & 0xF0000000U) >> 28);
    scr->sdSpec = (uint8_t)((rawScr[0] & 0xF000000) >> 24);
    if ((uint8_t)((rawScr[0] & 0x800000) >> 23))
    {
        scr->flags |= SD_SCR_DATA_STAT_AFTER_ERASE;
    }
    scr->sdSecurity = (uint8_t)((rawScr[0] & 0x700000) >> 20);
    scr->sdBusWidths = (uint8_t)((rawScr[0] & 0xF0000) >> 16);
    if ((uint8_t)((rawScr[0] & 0x8000) >> 15))
    {
        scr->flags |= SD_SCR_SD_SPEC3;
    }
    scr->exSecurity = (uint8_t)((rawScr[0] & 0x7800) >> 10);
    scr->cmdSupport = (uint8_t)(rawScr[0] & 0x3);
    scr->reservedForMan = rawScr[1];

    switch(scr->sdSpec)
    {
        case 0:
            card->version = SD_SPEC_VERSION_1_0;
            break;
        case 1:
            card->version = SD_SPEC_VERSION_1_1;
            break;
        case 2:
            card->version = SD_SPEC_VERSION_2_0;
            if (card->scr.flags & SD_SCR_SD_SPEC3)
            {
                card->version = SD_SPEC_VERSION_3_0;
            }
            break;
        default:
            break;
    }
    if (card->scr.sdBusWidths & SD_SCR_BUS_WIDTHS_4BIT)
    {
        card->caps |= SD_CARD_CAPS_BUSWIDTH_4BITS;
    }
}



/*FUNCTION****************************************************************
 *
 * Function Name: SD_IndentifyCard
 * Description: Identify card on the given host controller
 *
 *END*********************************************************************/
sdmmc_status_t SD_IndentifyCard(host_t *host, sd_t *card)
{
    sdmmc_status_t err = kStatus_SDMMC_NoError;
    uint32_t acmd41Arg = 0;

    assert(card);
    assert(host);

    //card->cardType = kCardTypeUnknown;
    //SDMMC_InitHost(host);

    card->host = host;

    if (SDMMC_ConfigClock(card->host, SDMMC_CLK_400KHZ))
    {
        return kStatus_SDMMC_SetCardBusClockFailed;
    }

    err = SD_GoIdle(card);
    if (err)
    {
        return kStatus_SDMMC_GoIdleCmdFailed;
    }
    //SDHC_GetCapability(&SDHC[host->instance], host->capability); 

    if ((host->capability->supportMask) & SDHC_SUPPORT_V330)
    {
        acmd41Arg |= SD_OCR_VDD_32_33 | SD_OCR_VDD_33_34;
    }
#if defined SDHC_SUPPORT_V300
    if ((host->capability->supportMask) & SDHC_SUPPORT_V300)
    {
        acmd41Arg |= SD_OCR_VDD_29_30;
    }
#endif
    err = SD_SendIfCond(card);
    if (err == kStatus_SDMMC_NoError)
    {
        /* SDHC or SDXC card */
        acmd41Arg |= SD_OCR_HCS;
        card->caps |= SD_CARD_CAPS_SDHC;
    }
    else
    {
        /* SDSC card */
        err = SD_GoIdle(card);
        if (err)
        {
            return kStatus_SDMMC_GoIdleCmdFailed;
        }
    }

    err = SD_AppSendOpCond(card, acmd41Arg);
    if (kStatus_SDMMC_TimeoutError == err)
    {
        /* MMC card */
        return kStatus_SDMMC_NotSupportYet;
    }
    else if (err)
    {
        return kStatus_SDMMC_SendOpCondCmdFailed;
    }

    return SD_Init(card);
}
             
/*FUNCTION****************************************************************
 *
 * Function Name: SD_ReadBlocks
 * Description: read blocks from card with default block size from SD or MMC card
 *
 *END*********************************************************************/
sdmmc_status_t SD_ReadBlocks(sd_t *card,
                    uint8_t *buffer,
                    uint32_t startBlock,
                    uint32_t blockCount)
{
    uint32_t blkCnt, blkLeft, blkDone;
    sdmmc_status_t err = kStatus_SDMMC_NoError;

    assert(card);
    assert(buffer);
    assert(blockCount);

    blkLeft = blockCount;
    blkDone = 0;

    if ((blockCount + startBlock) > card->blockCount)
    {
        return kStatus_SDMMC_InvalidIORange;
    }

    while(blkLeft)
    {
        if (blkLeft > card->host->capability->maxBlockLength)
        {
            blkLeft = blkLeft - card->host->capability->maxBlockLength;
            blkCnt = card->host->capability->maxBlockLength;
        }
        else
        {
            blkCnt = blkLeft;
            blkLeft = 0;
        }

        err = SD_Read(card,
                      buffer + blkDone * FSL_CARD_DEFAULT_BLOCK_SIZE,
                      startBlock + blkDone,
                      FSL_CARD_DEFAULT_BLOCK_SIZE,
                      blkCnt);
        if (err != kStatus_SDMMC_NoError)
        {
            return err;
        }
        blkDone += blkCnt;
    }

    return err;
}

/*FUNCTION****************************************************************
 *
 * Function Name: SD_WriteBlocks
 * Description: write blocks to card with default block size to SD/MMC card
 *
 *END*********************************************************************/
sdmmc_status_t SD_WriteBlocks(sd_t *card,
                     uint8_t *buffer,
                     uint32_t startBlock,
                     uint32_t blockCount)
{
    uint32_t blkCnt, blkLeft, blkDone;
    sdmmc_status_t err = kStatus_SDMMC_NoError;

    assert(card);
    assert(buffer);
    assert(blockCount);

    blkLeft = blockCount;
    blkDone = 0;

    if ((blockCount + startBlock) > card->blockCount)
    {
        return kStatus_SDMMC_InvalidIORange;
    }

    while(blkLeft)
    {
        if (blkLeft > card->host->capability->maxBlockLength)
        {
            blkLeft = blkLeft - card->host->capability->maxBlockLength;
            blkCnt = card->host->capability->maxBlockLength;
        }
        else
        {
            blkCnt = blkLeft;
            blkLeft = 0;
        }

        err = SD_Write(card,
                          buffer + blkDone * FSL_CARD_DEFAULT_BLOCK_SIZE,
                          startBlock + blkDone,
                          FSL_CARD_DEFAULT_BLOCK_SIZE,
                          blkCnt);
        if (err != kStatus_SDMMC_NoError)
        {
            return err;
        }
        blkDone += blkCnt;
    }

    return err;
}
             
/*FUNCTION****************************************************************
 *
 * Function Name: SD_EraseBlocks
 * Description: erase block range from SD/MMC card with default block size
 *
 *END*********************************************************************/
sdmmc_status_t SD_EraseBlocks(sd_t *card, uint32_t startBlock, uint32_t blockCount)
{
    uint32_t blkDone = 0, blkLeft, blkCnt;

    assert(card);
    assert(blockCount);

    blkLeft = blockCount;
    while(blkLeft)
    {
        if (blkLeft > (card->csd.sectorSize + 1))
        {
            blkCnt = card->csd.sectorSize + 1;
            blkLeft = blkLeft - blkCnt;
        }
        else
        {
            blkCnt = blkLeft;
            blkLeft = 0;
        }

        if (kStatus_SDMMC_NoError != SD_Erase(card,
                                              startBlock + blkDone,
                                              blkCnt))
        {
            return kStatus_SDMMC_EraseCmdFailed;
        }

        blkDone += blkCnt;
    }
    return kStatus_SDMMC_NoError;
}

/*************************************************************************************************
 * EOF
 ************************************************************************************************/