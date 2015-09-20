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
#include "card.h"
#include "sdmmc.h"



/*FUNCTION****************************************************************
 *
 * Function Name: SDMMC_CheckReadOnly
 * Description: Checks if the card is ready only
 *
 *END*********************************************************************/
bool SD_CheckReadOnly(sd_card_t *card)
{
    assert(card);
    return ((card->csd.flags & kSD_CsdPermWriteProtect) || (card->csd.flags & kSD_CsdTmpWriteProtect));   
}

/*FUNCTION****************************************************************
 *
 * Function Name: SDMMC_SelectCard
 * Description: Selects or deselects card
 *
 *END*********************************************************************/

static status_t inline SD_SelectCard(sd_card_t *card, bool isSelected)
{
    assert(card);
    return SDMMC_SelectCard(card->host, card->rca, isSelected);
}

/*FUNCTION****************************************************************
 *
 * Function Name: SDMMC_SendStatus
 * Description:  Sends the sd status
 *
 *END*********************************************************************/
static status_t inline SD_SendStatus(sd_card_t *card)
{
    assert(card);

    return SDMMC_SendStatus(card->host, card->rca);
}

/*FUNCTION****************************************************************
 *
 * Function Name: SDMMC_Shutdown
 * Description: Destorys initialized card and shutdown the corresponding
 * host controller
 *
 *END*********************************************************************/
void SD_DeInit(sd_card_t *card)
{
    assert(card);

    SD_SelectCard(card, false);
}


/*FUNCTION****************************************************************
 *
 * Function Name: SDMMC_SendApplicationCmd
 * Description: Sends application commandMemory to card
 *
 *END*********************************************************************/
static status_t inline SD_SendApplicationCmd(sd_card_t *card)
{
    assert(card);

    return SDMMC_SendApplicationCmd(card->host, card->rca);
}

/*FUNCTION****************************************************************
 *
 * Function Name: SDMMC_GoIdle
 * Description: Resets all cards to idle state
 *
 *END*********************************************************************/
static status_t inline SD_GoIdle(sd_card_t *card)
{
    assert(card);

    return SDMMC_GoIdle(card->host);
}

/*FUNCTION****************************************************************
 *
 * Function Name: SDMMC_StopTransmission
 * Description:  Sends stop transmission commandMemory to card to stop ongoing
 * data transferring.
 *
 *END*********************************************************************/
static status_t inline SD_StopTransmission(sd_card_t* card)
{
    assert(card);

    SDMMC_StopTransmission(card->host);
}

/*FUNCTION****************************************************************
 *
 * Function Name: SDMMC_SetBlockSize
 * Description:  Sets the block length in bytes for SDSC cards. For SDHC cards,
 * it does not affect memory read or write commands, always 512 bytes fixed
 * block length is used.
 *
 *END*********************************************************************/
static status_t inline SD_SetBlockSize(sd_card_t *card, uint32_t blockSize)
{
    assert(card);

    SDMMC_SetBlockSize(card->host, blockSize);
}

/*FUNCTION****************************************************************
 *
 * Function Name: SD_DecodeCsd
 * Description: Decodes csd register
 *
 *END*********************************************************************/
static void SD_DecodeCsd(uint32_t *rawCsd, sd_card_t *card)
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
        csd->flags |= kSD_CsdReadBlockPartial;
    }
    if (rawCsd[2] & 0x4000)
    {
        csd->flags |= kSD_CsdReadBlockPartial;
    }
    if (rawCsd[2] & 0x2000)
    {
        csd->flags |= kSD_CsdReadBlockMisalign;
    }
    if (rawCsd[2] & 0x1000)
    {
        csd->flags |= kSD_CsdDsrImplemented;
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
            card->flags |= kSd_IsSDXC;
        }
        card->blockCount = (csd->cSize + 1) * 1024;
    }

    if ((uint8_t)((rawCsd[1] & 0x4000) >> 14))
    {
        csd->flags |= kSD_CsdEraseBlockEnabled;
    }

    csd->sectorSize = (uint8_t)((rawCsd[1] & 0x3F80) >> 7);
    csd->wpGrpSize = (uint8_t)(rawCsd[1] & 0x7F);
    if ((uint8_t)(rawCsd[0] & 0x80000000U))
    {
        csd->flags |= kSD_CsdWPGroupEnabled;
    }
    csd->r2wFactor = (uint8_t)((rawCsd[0] & 0x1C000000) >> 26);
    csd->writeBlkLen = (uint8_t)((rawCsd[0] & 0x3C00000) >> 22);
    if ((uint8_t)((rawCsd[0] & 0x200000) >> 21))
    {
        csd->flags |= kSD_CsdWriteBlockPartial;
    }
    if ((uint8_t)((rawCsd[0] & 0x8000) >> 15))
    {
        csd->flags |= kSD_CsdFileFormatGroup;
    }
    if ((uint8_t)((rawCsd[0] & 0x4000) >> 14))
    {
        csd->flags |= kSD_CsdCopy;
    }
    if ((uint8_t)((rawCsd[0] & 0x2000) >> 13))
    {
        csd->flags |= kSD_CsdPermWriteProtect;
    }
    if ((uint8_t)((rawCsd[0] & 0x1000) >> 12))
    {
        csd->flags |= kSD_CsdTmpWriteProtect;
    }
    csd->fileFormat = (uint8_t)((rawCsd[0] & 0xC00) >> 10);
}

/*FUNCTION****************************************************************
 *
 * Function Name: SD_DecodeCid
 * Description: Decodes cid register
 *
 *END*********************************************************************/
static void SD_DecodeCid(uint32_t *rawCid, sd_card_t *card)
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
 * Description: Sends rca commandMemory to card to get relative card address
 *
 *END*********************************************************************/
static status_t SD_SendRca(sd_card_t *card)
{
    sdhc_host_t *host;
    sdhc_cmd_t command = {0};
    assert(card);

    host = card->host;

    command.index = kSD_SendRelativeAddr;
    command.argument = 0;
    command.responseType = kSDHC_ReponseTypeR6;

    host->currentCmd = &command;
    host->currentData = 0;

    if (kStatus_Success == SDHC_SendCmdBlocking(host, FSL_CARD_COMMAND_TIMEOUT))
    {
        card->rca = command->response[0] >> 16;
        return kStatus_Success;
    }
    
    return kStatus_Fail;
}

/*FUNCTION****************************************************************
 *
 * Function Name: SD_Switch
 * Description: Sends switch commandMemory to card
 *
 *END*********************************************************************/
static status_t SD_Switch(sd_card_t *card, uint32_t mode, uint32_t group, uint32_t value, uint32_t *resp)
{
    sdhc_host_t *host;
    sdhc_cmd_t command = {0};
    sdhc_data_t data = {0};
    assert(card);

    host = card->host;

    command.index = kSD_Switch;
    command.argument = mode << 31 | 0x00FFFFFF;
    command.argument &= ~((uint32_t)(0xF) << (group * 4));
    command.argument |= value << (group * 4);
    command.responseType = kSDHC_ReponseTypeR1;
    
    data.blockSize = 64;
    data.blockCount = 1;
    data.buffer = resp;
    data.isRead = true;

    if (kStatus_Success != SD_SetBlockSize(card, data.blockSize))
    {
        return kStatus_SDMMC_SetCardBlockSizeFailed;
    }
    
    host->currentCmd = &command;
    host->currentData = &data;

    if (kStatus_Success != SDHC_SendCmdBlocking(host, FSL_CARD_COMMAND_TIMEOUT)|| (!SDMMC_R1_ERROR_BITS(command->response[0]))
        || (kStatus_Success != SDHC_WaitDataTransferComplete(host, FSL_CARD_COMMAND_TIMEOUT)))
    {        
        return kStatus_SDMMC_TransferDataFailed;
    }
    
    return kStatus_Success;
}

/*FUNCTION****************************************************************
 *
 * Function Name: SD_DecodeScr
 * Description: Decodes scr register
 *
 *END*********************************************************************/
static void SD_DecodeScr(uint32_t *rawScr, sd_card_t *card)
{
    sd_scr_t *scr;
    assert(rawScr);
    assert(card);

    scr = &(card->scr);
    scr->scrStructure = (uint8_t)((rawScr[0] & 0xF0000000U) >> 28);
    scr->sdSpec = (uint8_t)((rawScr[0] & 0xF000000) >> 24);
    if ((uint8_t)((rawScr[0] & 0x800000) >> 23))
    {
        scr->flags |= kSD_ScrDataStatAfterErase;
    }
    scr->sdSecurity = (uint8_t)((rawScr[0] & 0x700000) >> 20);
    scr->sdBusWidths = (uint8_t)((rawScr[0] & 0xF0000) >> 16);
    if ((uint8_t)((rawScr[0] & 0x8000) >> 15))
    {
        scr->flags |= kSD_ScrSdSpec3;
    }
    scr->exSecurity = (uint8_t)((rawScr[0] & 0x7800) >> 10);
    scr->cmdSupport = (uint8_t)(rawScr[0] & 0x3);
    scr->reservedForMan = rawScr[1];

    switch(scr->sdSpec)
    {
        case 0:
            card->version = kSD_SpecVersion1_0;
            break;
        case 1:
            card->version = kSD_SpecVersion1_1;
            break;
        case 2:
            card->version = kSD_SpecVersion2_0;
            if (card->scr.flags & kSD_ScrSdSpec3)
            {
                card->version = kSD_SpecVersion3_0;
            }
            break;
        default:
            break;
    }
    if (card->scr.sdBusWidths & kSD_BusWidth4Bit)
    {
        card->flags |= kSd_Support4BitWidth;
    }
}

/*FUNCTION****************************************************************
 *
 * Function Name: SD_SendScr
 * Description: Fetches scr register from card
 *
 *END*********************************************************************/
static status_t SD_SendScr(sd_card_t *card)
{
    sdhc_host_t *host;
    sdhc_cmd_t command = {0};
    sdhc_data_t data = {0};
    uint32_t rawScr[2] = {0};
    assert(card);

    host = card->host;
    
    if (kStatus_Success != SD_SendApplicationCmd(card))
    {
        return kStatus_SDMMC_SendAppCmdFailed;
    }
    
    command.index = kSD_AppSendScr;
    command.responseType = kSDHC_ReponseTypeR1;
    command.argument = 0;
    host->currentCmd = &command;

    data.blockSize = 8;
    data.blockCount = 1;
    data.buffer = rawScr;
    data.flags |= CARD_DATA_FLAGS_DATA_READ;
    host->currentData = &data;

    if ((kStatus_Success != SDHC_SendCmdBlocking(host, FSL_CARD_COMMAND_TIMEOUT)) || (!SDMMC_R1_ERROR_BITS(command->response[0]))
        || (kStatus_Success != SDHC_WaitDataTransferComplete(host, FSL_CARD_COMMAND_TIMEOUT)))
    {
        return kStatus_Fail;
    }

    /* Card date is read as big endian. */
    if (kSDHC_EndianModeLittle == host->sdhcConfig->endianMode)
    {
        /* Converts byte sequence when system is little endian. */
        rawScr[0] = SWAP_UINT32_IN_LITTLE_ENDIAN(rawScr[0]);
        rawScr[1] = SWAP_UINT32_IN_LITTLE_ENDIAN(rawScr[1]);
    }
    
    memcpy(card->rawScr, rawScr, sizeof(card->rawScr));

    SD_DecodeScr(rawScr, card);    
    return kStatus_Success;
}

/*FUNCTION****************************************************************
 *
 * Function Name: SD_SwitchHighspeed
 * Description: Switches high speed mode of the specific card
 *
 *END*********************************************************************/
static status_t SD_SwitchHighspeed(sd_card_t *card)
{
    uint32_t response[16] = {0};
    assert(card);

    if ((card->version < kSD_SpecVersion1_0) || (!(card->csd.ccc & kSDMMC_CmdClassSwitch)))
    {
        return kStatus_SDMMC_CardNotSupport;
    }

    if (kStatus_Success != SD_Switch(card, kSD_SwitchCheck, 0, 1, response))
    {
        return kStatus_SDMMC_SwitchFailed;
    }

    if (kSDHC_EndianModeLittle == host->sdhcConfig->endianMode)
    {
        /* Converts byte sequence when system is little endian. */
        response[3] = SWAP_UINT32_IN_LITTLE_ENDIAN(response[3]);
        response[4] = SWAP_UINT32_IN_LITTLE_ENDIAN(response[4]);
    }

    if ((!(response[3] & 0x10000)) || ((response[4] & 0x0f000000) == 0x0F000000))
    {
        return kStatus_SDMMC_CardNotSupport;
    }

    if (kStatus_Success != SD_Switch(card, kSD_SwitchSet, 0, 1, response))
    {
        return kStatus_SDMMC_SwitchFailed;
    }

    /* If swich function group failed, function group will be returned. */
    if ((response[4] & 0x0f000000) != 0x01000000) 
    {
        return kStatus_Fail;
    }

    return kStatus_Success;
}

/*FUNCTION****************************************************************
 *
 * Function Name: SD_SetBusWidth
 * Description: Sets desired buswidth
 *
 *END*********************************************************************/
static uint32_t SD_SetBusWidth(sd_card_t *card, sd_bus_width_t busWidth)
{
    sdhc_host_t *host;
    sdhc_cmd_t command = {0};
    assert(card);

    host = card->host;
    
    if (kStatus_Success != SD_SendApplicationCmd(card))
    {
        return kStatus_SDMMC_SendAppCmdFailed;
    }

    command.index = kSD_AppSetBusWdith;
    command.responseType = kSDHC_ReponseTypeR1;
    switch (busWidth)
    {
        case kSD_BusWidth1Bit:
            command.argument = 0U;
            break;
        case kSD_BusWidth4Bit:
            command.argument = 2U;
            break;
        default:
            return kStatus_InvalidArgument; 
    }

    host->currentCmd = &command;
    host->currentData = 0;

    if (kStatus_Success != SDHC_SendCmdBlocking(host, FSL_CARD_COMMAND_TIMEOUT) || (!SDMMC_R1_ERROR_BITS(command->response[0])))
    {
        return kStatus_SDMMC_SendCommandFailed;
    }
    
    return kStatus_Success;
}



/*FUNCTION****************************************************************
 *
 * Function Name: SD_SendCsd
 * Description: Gets csd from card
 *
 *END*********************************************************************/
static status_t SD_SendCsd(sd_card_t * card)
{
    sdhc_host_t *host;
    sdhc_cmd_t command = {0};
    assert(card);

    host = card->host;
    
    command.index = kSDMMC_SendCsd;
    command.argument = card->rca << 16;
    command.responseType = kSDHC_ReponseTypeR2;

    host->currentCmd = &command;
    host->currentData = 0;

    if (kStatus_Success == SDHC_SendCmdBlocking(host, FSL_CARD_COMMAND_TIMEOUT))
    {
        memcpy(card->rawCsd, command->response, sizeof(card->rawCsd));
        /* The response is from bit 127:8 in R2, corrisponding to command->response[3]:command->response[0][31:8]. */
        SD_DecodeCsd(command->response, card);
    
        return kStatus_Success;
    }

    return kStatus_SDMMC_SendCommandFailed;
}

/*FUNCTION****************************************************************
 *
 * Function Name: SD_AllSendCid
 * Description: Sends all_send_cid commandMemory
 *
 *END*********************************************************************/
static status_t SD_AllSendCid(sd_card_t *card)
{
    sdhc_host_t *host;
    sdhc_cmd_t command = {0};
    assert(card);

    host = card->host;
    
    command.index = kSDMMC_AllSendCid;
    command.argument = 0;
    command.responseType = kSDHC_ReponseTypeR2;

    host->currentCmd = &command;
    host->currentData = 0;

    if (kStatus_Success == SDHC_SendCmdBlocking(host, FSL_CARD_COMMAND_TIMEOUT))
    {
        memcpy(card->rawCid, command->response, sizeof(card->rawCid));
        SD_DecodeCid(command->response, card);      
        
        return kStatus_Success;
    }
    
    return kStatus_SDMMC_SendCommandFailed;
}

/*FUNCTION****************************************************************
 *
 * Function Name: SD_InitCard
 * Description: Initializes SD memory card
 *
 *END*********************************************************************/
static status_t SD_InitCard(sd_card_t *card)
{
    sdhc_sd_clock_config_t sdClockConfig = {0};
    status_t err;
    assert(card);
    
    if (kStatus_Success != SD_AllSendCid(card))
    {
        return kStatus_SDMMC_AllSendCidFailed;
    }
    
    if (kStatus_Success != SD_SendRca(card))
    {
        return kStatus_SDMMC_SendRcaFailed;
    }

    if (kStatus_Success != SD_SendCsd(card))
    {
        return kStatus_SDMMC_SendCsdFailed;
    }
    
    if (kStatus_Success != SD_SelectCard(card, true))
    {
        return kStatus_SDMMC_SelectCardFailed;
    }
 
    if (kStatus_Success != SD_SendScr(card))
    {
        return kStatus_SDMMC_SendScrFailed;
    }

    sdClockConfig.enableSdClock = true;
    sdClockConfig.baseClockFreq = host->capability->sourceClockFreq;
    sdClockConfig.sdClockFreq = SD_CLK_25MHZ;    
    SDHC_SetSdClockConfig(host->base, &sdClockConfig);

    if (DOES_SDHC_SUPPORT_4BITS(card->host) && DOES_SD_SUPPORT_4BITS(card))
    {
        if (kStatus_Success != SD_SetBusWidth(card, kSD_BusWidth4Bit))
        {
            return kStatus_SDMMC_SetBusWidthFailed;
        }
        SDHC_SetDataTransferWidth(card->host->base, kSDHCDtw4Bit);
    }

    if (DOES_SDHC_SUPPORT_HIGHSPEED(card->host))
    {
        err = SD_SwitchHighspeed(card);
        if ((err != kStatus_Success) && (kStatus_SDMMC_CardNotSupport != err))
        {
            return kStatus_SDMMC_SwitchHighSpeedFailed;
        }
        else if (err == kStatus_Success)
        { 
            sdClockConfig.enableSdClock = true;
            sdClockConfig.baseClockFreq = host->capability->sourceClockFreq;
            sdClockConfig.sdClockFreq = SD_CLK_50MHZ;    
            SDHC_SetSdClockConfig(host->base, &sdClockConfig);
        }
        else /* Not support is also OK. */
        {
            err = kStatus_Success;
        }
    }

    if (SD_SetBlockSize(card, FSL_CARD_DEFAULT_BLOCK_SIZE))
    {
        err = kStatus_SDMMC_SetCardBlockSizeFailed;
    }
    
    return kStatus_Success;
}

/*FUNCTION****************************************************************
 *
 * Function Name: SD_AppSendOpCond
 * Description: Sends host capacity support information and asks the accessed
 * card to send its operating condition register content.
 *
 *END*********************************************************************/
static status_t SD_AppSendOpCond(sd_card_t *card, uint32_t acmd41Arg)
{
    
    sdhc_host_t *host;
    sdhc_cmd_t command = {0};
    status_t err;
    uint32_t i = FSL_CARD_MAX_VOLT_RETRIES;
    assert(card);

    host = card->host;
    
    command.index = kSD_AppSendOpCond;
    command.argument = acmd41Arg;
    command.responseType = kSDHC_ReponseTypeR3;
    
    while (i--)
    {
        if (kStatus_Success != SD_SendApplicationCmd(card))
        {
            return kStatus_SDMMC_SendAppCmdFailed;
        }
        
        host->currentCmd = &command;
        host->currentData = 0;

        if (kStatus_Success != SDHC_SendCmdBlocking(host, FSL_CARD_COMMAND_TIMEOUT))
        {
            return kStatus_SDMMC_SendCommandFailed;
        }

        if (command->response[0] & kSD_OcrPowerUpBusy)
        {
            if (command->response[0] & kSD_OcrCCS)
            {
                card->flags |= kSd_HighCapacity;
            }
            err = kStatus_Success;
            card->ocr = command->response[0];
            break;
        }

        err = kStatus_Timeout;
        SDMMC_DelayTimeMsec(1);
    }

    return err;
}

/*FUNCTION****************************************************************
 *
 * Function Name: SD_SendIfCond
 * Description: Checks card interface condition, which includes host supply
 * voltage information and asks the card whether card supports voltage.
 *
 *END*********************************************************************/
static status_t SD_SendIfCond(sd_card_t *card)
{
    sdhc_host_t *host;
    sdhc_cmd_t command = {0};
    status_t err = ;
    assert(card);

    host = card->host;
    
    command.index = kSD_SendIfCond;
    command.argument = 0x1AA;
    command.responseType = kSDHC_ReponseTypeR7;

    host->currentCmd = &command;
    host->currentData = 0;

    if (kStatus_Success != SDHC_SendCmdBlocking(host, FSL_CARD_COMMAND_TIMEOUT))
    {
        return kStatus_SDMMC_SendCommandFailed;
    }
    else if ((command->response[0] & 0xFF) != 0xAA)
    {
        return kStatus_SDMMC_CardNotSupport;
    }
  
    return kStatus_Success;
}

/*FUNCTION****************************************************************
 *
 * Function Name: SD_Read
 * Description: Reads data from specific SD card
 *
 *END*********************************************************************/
static status_t SD_Read(sd_card_t *card, uint8_t *buffer, uint32_t startBlock, uint32_t blockSize, 
                uint32_t blockCount)
{
    sdhc_host_t *host;
    sdhc_cmd_t command = {0};
    sdhc_data_t data = {0};
    assert(card);
    assert(buffer);
    assert(blockCount);
    assert(blockSize);
    assert(blockSize == FSL_CARD_DEFAULT_BLOCK_SIZE);

    host = card->host;

    if (((card->flags & kSd_HighCapacity) && (blockSize != 512)) || (blockSize > card->blockSize)
         || (blockSize > host->capability->maxBlockLength) || (blockSize % 4))
    {
        return kStatus_SDMMC_CardNotSupport;
    }

    data.blockSize = blockSize;
    data.blockCount = blockCount;
    data.buffer = (uint32_t *)buffer;
    data.flags |= CARD_DATA_FLAGS_DATA_READ;

    command.index = kSDMMC_ReadMultipleBlock;
    if (data.blockCount == 1)
    {
        command.index = kSDMMC_ReadSingleBlock;
    }
    command.argument = startBlock;
    if (!(card->flags & kSd_HighCapacity))
    {
        command.argument *= data.blockSize;
    }
    command.responseType = kSDHC_ReponseTypeR1;

    host->currentCmd = &command;
    host->currentData = &data;

    if ((kStatus_Success != SDHC_SendCmdBlocking(host, FSL_CARD_COMMAND_TIMEOUT)) || (!SDMMC_R1_ERROR_BITS(command->response[0]))
        || (kStatus_Success != SDHC_WaitDataTransferComplete(host, FSL_CARD_COMMAND_TIMEOUT)))
    {
        return kStatus_Fail;
    }

    if (host->flags & SDHC_FLAGS_USE_AUTO_CMD12)
    {
        if (data.blockCount > 1)
        {
            if (host->sdhcConfig.enableAutoCmd12)
            {
                if (kStatus_Success != SD_StopTransmission(card))
                { 
                    return kStatus_SDMMC_StopTransmissionFailed;
                }
            }
           
        }
    }
    
    return kStatus_Success;
}

/*FUNCTION****************************************************************
 *
 * Function Name: SD_Write
 * Description: Writes data from specific card
 *
 *END*********************************************************************/
static status_t SD_Write(sd_card_t *card, uint8_t *buffer, uint32_t startBlock, uint32_t blockSize, 
                uint32_t blockCount)
{
    sdhc_host_t *host;
    sdhc_cmd_t command = {0};
    sdhc_data_t data = {0};
    assert(card);
    assert(buffer);
    assert(blockCount);
    assert(blockSize);
    assert(blockSize == FSL_CARD_DEFAULT_BLOCK_SIZE);

    host = card->host;

    if (((card->flags & kSd_HighCapacity) && (blockSize != 512)) || (blockSize > card->blockSize)
         || (blockSize > host->capability->maxBlockLength) || (blockSize % 4))
    {
        return kStatus_SDMMC_CardNotSupport;
    }

    data.blockSize = blockSize;
    data.blockCount = blockCount;
    data.buffer = (uint32_t *)buffer;
    
    command.index = kSDMMC_WriteMultipleBlock;
    if (data.blockCount == 1)
    {
        command.index = kSDMMC_WriteBlock;
    }
    command.argument = startBlock;
    if (!(card->flags & kSd_HighCapacity))
    {
        command.argument *= data.blockSize;
    }
    command.responseType = kSDHC_ReponseTypeR1;

    host->currentCmd = &command;
    host->currentData = &data;

    if ((kStatus_Success != SDHC_SendCmdBlocking(host, FSL_CARD_COMMAND_TIMEOUT))
        || (!SDMMC_R1_ERROR_BITS(command->response[0]))
        || (kStatus_Success != SDHC_WaitDataTransferComplete(host, FSL_CARD_COMMAND_TIMEOUT)))
    {
        return kStatus_Fail;
    }

    if (data.blockCount > 1)
    {
        if (host->sdhcConfig.enableAutoCmd12)
        {
            if (kStatus_Success != SD_StopTransmission(card))
            {
                return kStatus_SDMMC_StopTransmissionFailed;
            }
        }
    }
    
    return kStatus_Success;
}

/*FUNCTION****************************************************************
 *
 * Function Name: SD_Erase
 * Description: Erases data for the given block range
 *
 *END*********************************************************************/
static status_t SD_Erase(sd_card_t *card, uint32_t startBlock, uint32_t blockCount)
{
    sdhc_host_t *host;
    uint32_t s, e;
    sdhc_cmd_t command = {0};
    assert(card);
    assert(blockCount);

    host = card->host;

    s = startBlock;
    e = s + blockCount - 1;
    if (!(card->flags & kSd_HighCapacity))
    {
        s = s * FSL_CARD_DEFAULT_BLOCK_SIZE;
        e = e * FSL_CARD_DEFAULT_BLOCK_SIZE;
    }
    
    command.index = kSD_EraseWrBlkStart;
    command.argument = s;
    command.responseType = kSDHC_ReponseTypeR1;

    host->currentCmd = &command;
    host->currentData = 0;

    if (kStatus_Success != SDHC_SendCmdBlocking(host, FSL_CARD_COMMAND_TIMEOUT) 
        || (!SDMMC_R1_ERROR_BITS(command->response[0])))
    {
        return kStatus_Fail;
    }

    command.index = kSD_EraseWrBlkEnd;
    command.argument = e;
    
    host->currentCmd = &command;
    host->currentData = 0;

    if ((kStatus_Success != SDHC_SendCmdBlocking(host, FSL_CARD_COMMAND_TIMEOUT))
        || (!SDMMC_R1_ERROR_BITS(command->response[0])))
    { 
        return kStatus_Fail;
    }

    command.index = kSDMMC_Erase;
    command.argument = 0;
    command.responseType = kSDHC_ReponseTypeR1b;
    
    host->currentCmd = &command;
    host->currentData = 0;
    
    if ((kStatus_Success != SDHC_SendCmdBlocking(host, FSL_CARD_COMMAND_TIMEOUT))
        || (!SDMMC_R1_ERROR_BITS(command->response[0])))
    {
        return kStatus_Fail;
    }

    return kStatus_Success;
}


/*FUNCTION****************************************************************
 *
 * Function Name: SDMMC_DecodeScr
 * Description: Decodes scr register
 *
 *END*********************************************************************/
static void SDMMC_DecodeScr(uint32_t *rawScr, sd_card_t *card)
{
    sd_scr_t *scr;
    assert(rawScr);
    assert(card);

    scr = &(card->scr);
    scr->scrStructure = (uint8_t)((rawScr[0] & 0xF0000000U) >> 28);
    scr->sdSpec = (uint8_t)((rawScr[0] & 0xF000000) >> 24);
    if ((uint8_t)((rawScr[0] & 0x800000) >> 23))
    {
        scr->flags |= kSD_ScrDataStatAfterErase;
    }
    scr->sdSecurity = (uint8_t)((rawScr[0] & 0x700000) >> 20);
    scr->sdBusWidths = (uint8_t)((rawScr[0] & 0xF0000) >> 16);
    if ((uint8_t)((rawScr[0] & 0x8000) >> 15))
    {
        scr->flags |= kSD_ScrSdSpec3;
    }
    scr->exSecurity = (uint8_t)((rawScr[0] & 0x7800) >> 10);
    scr->cmdSupport = (uint8_t)(rawScr[0] & 0x3);
    scr->reservedForMan = rawScr[1];

    switch(scr->sdSpec)
    {
        case 0:
            card->version = kSD_SpecVersion1_0;
            break;
        case 1:
            card->version = kSD_SpecVersion1_1;
            break;
        case 2:
            card->version = kSD_SpecVersion2_0;
            if (card->scr.flags & kSD_ScrSdSpec3)
            {
                card->version = kSD_SpecVersion3_0;
            }
            break;
        default:
            break;
    }
    if (card->scr.sdBusWidths & kSD_BusWidth4Bit)
    {
        card->flags |= kSd_Support4BitWidth;
    }
}



/*FUNCTION****************************************************************
 *
 * Function Name: SD_Init
 * Description: Identifies card on the given host controller
 *
 *END*********************************************************************/
status_t SD_Init(sd_card_t *card)
{
    status_t err = kStatus_Success;
    sdhc_host_t *host;
    sdhc_sd_clock_config_t sdClockConfig = {0};
    uint32_t acmd41Arg = 0;
    assert(card);
    assert(card->host);

    host = card->host;

    sdClockConfig.enableSdClock = true;
    sdClockConfig.baseClockFreq = host->capability->sourceClockFreq;
    sdClockConfig.sdClockFreq = SDMMC_CLK_400KHZ;    
    SDHC_SetSdClockConfig(host->base, &sdClockConfig);
    
    if (kStatus_Success != SD_GoIdle(card))
    {
        return kStatus_SDMMC_GoIdleFailed;
    }

    if ((host->capability->supportMask) & SDHC_SUPPORT_V330)
    {
        acmd41Arg |= kSD_OcrVdd32_33 | kSD_OcrVdd33_34;
    }
#if defined FSL_FEATURE_SDHC_HAS_V300_SUPPORT && FSL_FEATURE_SDHC_HAS_V300_SUPPORT
    if ((host->capability->supportMask) & SDHC_SUPPORT_V300)
    {
        acmd41Arg |= kSD_OcrVdd29_30;
    }
#endif
    if (kStatus_Success == SD_SendIfCond(card))
    {
        /* SDHC or SDXC card */
        acmd41Arg |= kSD_OcrHCS;
        card->flags |= kSd_IsSDHC;
    }
    else
    {
        /* SDSC card */
        if (kStatus_Success != SD_GoIdle(card))
        {
            return kStatus_SDMMC_GoIdleFailed;
        }
    }

    err = SD_AppSendOpCond(card, acmd41Arg);
    if (kStatus_Timeout == err)
    {
        /* MMC card */
        return kStatus_SDMMC_NotSupportYet;
    }
    else if (err)
    {
        return kStatus_SDMMC_SendOpCondFailed;
    }

    return SD_InitCard(card);
}
             
/*FUNCTION****************************************************************
 *
 * Function Name: SD_ReadBlocks
 * Description: Reads blocks from card with default block size from SD or MMC card
 *
 *END*********************************************************************/
status_t SD_ReadBlocks(sd_card_t *card, uint8_t *buffer, uint32_t startBlock, uint32_t blockCount)
{
    uint32_t blkCnt, blkLeft, blkDone;
    status_t err = kStatus_Success;
    sdhc_host_t *host;
    assert(card);
    assert(buffer);
    assert(blockCount);

    host = card->host;
    blkLeft = blockCount;
    blkDone = 0;

    if ((blockCount + startBlock) > card->blockCount)
    {
        return kStatus_InvalidArgument;
    }

    while(blkLeft)
    {
        if (blkLeft > host->capability->maxBlockLength)
        {
            blkLeft = blkLeft - host->capability->maxBlockLength;
            blkCnt = host->capability->maxBlockLength;
        }
        else
        {
            blkCnt = blkLeft;
            blkLeft = 0;
        }

        err = SD_Read(card, buffer + blkDone * FSL_CARD_DEFAULT_BLOCK_SIZE, startBlock + blkDone,
                      FSL_CARD_DEFAULT_BLOCK_SIZE, blkCnt);
        if (err != kStatus_Success)
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
 * Description: Writes blocks to card with default block size to SD/MMC card
 *
 *END*********************************************************************/
status_t SD_WriteBlocks(sd_card_t *card, uint8_t *buffer, uint32_t startBlock, uint32_t blockCount)
{
    uint32_t blkCnt, blkLeft, blkDone;
    status_t err = kStatus_Success;
    sdhc_host_t *host;
    assert(card);
    assert(buffer);
    assert(blockCount);

    host = card->host;
    blkLeft = blockCount;
    blkDone = 0;

    if ((blockCount + startBlock) > card->blockCount)
    {
        return kStatus_InvalidArgument;
    }

    while(blkLeft)
    {
        if (blkLeft > host->capability->maxBlockLength)
        {
            blkLeft = blkLeft - host->capability->maxBlockLength;
            blkCnt = host->capability->maxBlockLength;
        }
        else
        {
            blkCnt = blkLeft;
            blkLeft = 0;
        }

        err = SD_Write(card, buffer + blkDone * FSL_CARD_DEFAULT_BLOCK_SIZE, startBlock + blkDone,
                          FSL_CARD_DEFAULT_BLOCK_SIZE, blkCnt);
        if (err != kStatus_Success)
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
 * Description: Erases block range from SD/MMC card with default block size
 *
 *END*********************************************************************/
status_t SD_EraseBlocks(sd_card_t *card, uint32_t startBlock, uint32_t blockCount)
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

        if (kStatus_Success != SD_Erase(card, startBlock + blkDone, blkCnt))
        {
            return kStatus_SDMMC_EraseFailed;
        }

        blkDone += blkCnt;
    }
    
    return kStatus_Success;
}

