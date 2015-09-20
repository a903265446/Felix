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
#include "sdmmc.h"

/*FUNCTION****************************************************************
 *
 * Function Name: MMC_CheckReadOnly
 * Description: Checks if the card is ready only
 *
 *END*********************************************************************/
bool MMC_CheckReadOnly(mmc_card_t *card)
{
    assert(card);
    return ((card->csd.flags & kMMC_CsdPermWriteProtect) || (card->csd.flags & kMMC_CsdTmpWriteProtect)); 
}
 
/*FUNCTION****************************************************************
 *
 * Function Name: MMC_SelectCard
 * Description: Selects or deselects card
 *
 *END*********************************************************************/
static status_t inline MMC_SelectCard(mmc_card_t *card, bool isSelected)
{
    assert(card);
    return SDMMC_SelectCard(card->host, card->rca, isSelected);
}

/*FUNCTION****************************************************************
 *
 * Function Name: MMC_SendStatus
 * Description:  Sends the sd status
 *
 *END*********************************************************************/
static status_t inline MMC_SendStatus(mmc_card_t *card)
{
    assert(card);

    return SDMMC_SendStatus(card->host, card->rca);
}

/*FUNCTION****************************************************************
 *
 * Function Name: MMC_DeInit
 * Description: Destories initialized card and shutdown the corresponding
 * host controller
 *
 *END*********************************************************************/
void MMC_DeInit(mmc_card_t *card)
{
    assert(card);
    MMC_SelectCard(card, false);
}

/*FUNCTION****************************************************************
 *
 * Function Name: MMC_SendApplicationCmd
 * Description: SendS application commandMemory to card
 *
 *END*********************************************************************/
static status_t inline MMC_SendApplicationCmd(mmc_card_t *card)
{
    assert(card);

    return SDMMC_SendApplicationCmd(card->host, card->rca);
}

/*FUNCTION****************************************************************
 *
 * Function Name: MMC_SetBlockCount
 * Description:  Sends the set-block-count commandMemory.
 *
 *END*********************************************************************/
static status_t inline MMC_SetBlockCount(mmc_card_t *card, uint32_t blockCount)
{
    assert(card);
    
    return SDMMC_SetBlockCount(card->host, blockCount);
}

/*FUNCTION****************************************************************
 *
 * Function Name: MMC_GoIdle
 * Description: Resets all cards to idle state
 *
 *END*********************************************************************/
static status_t inline MMC_GoIdle(mmc_card_t *card)
{
    assert(card);
    
    return SDMMC_GoIdle(card->host);
}

/*FUNCTION****************************************************************
 *
 * Function Name: MMC_StopTransmission
 * Description:  Sends stop transmission commandMemory to card to stop ongoing
 * data transferring.
 *
 *END*********************************************************************/
static status_t inline MMC_StopTransmission(mmc_card_t *card)
{
    assert(card);

    SDMMC_StopTransmission(card->host);
}

/*FUNCTION****************************************************************
 *
 * Function Name: MMC_SetBlockSize
 * Description:  Sets the block length in bytes for MMC cards.
 *
 *END*********************************************************************/
static status_t inline MMC_SetBlockSize(mmc_card_t *card, uint32_t blockSize)
{
    assert(card);

    SDMMC_SetBlockSize(card->host, blockSize);
}

/*FUNCTION****************************************************************
 *
 * Function Name: MMC_ValidateVolt
 * Description: Validates if the card voltage range equals to host intended voltage range
 *
 *END*********************************************************************/
static status_t MMC_ValidateVolt(mmc_card_t *card, bool hostVoltRange)
{
    sdhc_cmd_t command = {0};
    sdhc_host_t *host;
    uint8_t accessMode;
    status_t err = kStatus_Success;
    assert(card);

    host = card->host;
    /* Save host intended voltage range */
    card->hostVoltRange = kMMC_Volt270to360;
    
    /* Send CMD1, with the intended voltage range in the argument (either 0x00FF8000 or 0x00000080) */
    command.index = kMMC_SendOpCond;
    if (hostVoltRange == kMMC_Volt170to195)
    {
        card->ocr = 0;
        card->ocr |=  (kMMC_Volt170to195 << MMC_OCR_V170TO195_SHIFT);
        command.argument = card->ocr;
    }
    else
    {
        card->ocr = 0;
        card->ocr |= (kMMC_Volt270to360 << MMC_OCR_V270TO360_SHIFT);
        command.argument = card->ocr;
    }
    command.responseType = kSDHC_ReponseTypeR3;

    host->currentCmd = &command;
    host->currentData = 0;
    
    do
    {
        err = SDHC_SendCmdBlocking(host, FSL_CARD_COMMAND_TIMEOUT);
        if (kStatus_Success == err)
        {
            if(!((command->response[0] & MMC_OCR_BUSY_MASK) >> MMC_OCR_BUSY_SHIFT))/* Return busy state */
            {
                continue;
            }
            else
            {
                /* Get the voltage range and access mode in the OCR register */
                card->ocr = command->response[0];
                /* Save raw OCR register content */
                //card->ocr        = command->response[0];
                break;
            }
        }
        else
        {
            break;
        }
    } while(1);

    return err;
}

/*FUNCTION****************************************************************
 *
 * Function Name: MMC_SetRelativeAddress
 * Description: Sets the relative address of the card.
 *
 *END*********************************************************************/
static status_t MMC_SetRelativeAddress(mmc_card_t *card)
{
    sdhc_cmd_t command = {0};
    sdhc_host_t *host;
    assert(card);
    assert(card->host);

    host = card->host;

    /* Send CMD3 with a chosen RCA, with value greater than 1 */
    command.index = kMMC_SetRelativeAddr;
    command.argument = (MMC_DEFAULT_RSA << 16);
    command.responseType = kSDHC_ReponseTypeR1;

    host->currentCmd = &command;
    host->currentData = 0;

    if ((kStatus_Success != SDHC_SendCmdBlocking(host, FSL_CARD_COMMAND_TIMEOUT)) 
        || (!SDMMC_R1_ERROR_BITS(command->response[0]))) 
    {
        card->rca = MMC_DEFAULT_RSA;      
        return kStatus_SDMMC_SendCommandFailed;
    }

    return kStatus_Success;
}

/*FUNCTION****************************************************************
 *
 * Function Name: MMC_CalculateTotalBlockCount
 * Description: Calculates the size of the card.
 *
 *END*********************************************************************/
static void MMC_CalculateTotalBlockCount(mmc_card_t *card)
{
    uint32_t c_size,c_size_mult, mult, read_bl_len, block_len;
    uint32_t blkSizeDef = FSL_CARD_DEFAULT_BLOCK_SIZE;/* Default block size after power on */    
    assert(card);
    
    c_size = card->csd.c_size;
    /*  For higher than 2GB of density of card the maximum possible value should be set to this register (0xFFF) */
    if(c_size != 0xFFF)
    {
        c_size_mult = card->csd.c_size_mult;
        mult = (2 << (c_size_mult + 2 - 1));
        read_bl_len = card->csd.readBlkLen;
        block_len = (2 << (read_bl_len - 1));
        
        card->blockCount = (((c_size + 1)*mult)/blkSizeDef);        
    }
    else /* For higher than 2GB of density of card, the device size represented by sector-count field in the EXT_CSD */
    {
        card->blockCount = card->extCsd.sectorCount;          
        card->flags |= kMMC_HighCapacity;
    }
    card->blockSize = FSL_CARD_DEFAULT_BLOCK_SIZE;
}

/*FUNCTION****************************************************************
 *
 * Function Name: MMC_DecodeCsd
 * Description: Decodes the CSD register content.
 *
 *END*********************************************************************/
static void MMC_DecodeCsd(uint32_t *rawCsd, mmc_card_t *card)
{
    mmc_csd_t *csd;
    assert(rawCsd);
    assert(card);

    csd = &(card->csd);
    csd->csdStructVer = (uint8_t)((rawCsd[3] & 0xC0000000U) >> 30);
    csd->sysSpecVer = (uint8_t)((rawCsd[3] & 0x30000000U) >> 28);
    if(4 == csd->sysSpecVer)
    {
        card->flags |= kMMC_HighSpeed;         
    }
    csd->taac = (uint8_t)((rawCsd[3] & 0xFF0000) >> 16);
    csd->nsac = (uint8_t)((rawCsd[3] & 0xFF00) >> 8);
    csd->tranSpeed = (uint8_t)(rawCsd[3] & 0xFF);
    csd->ccc = (uint16_t)((rawCsd[2] & 0xFFF00000U) >> 20);
    /* Max block length which can be read/write one time */
    csd->readBlkLen = (uint8_t)((rawCsd[2] & 0xF0000) >> 16);
    if (rawCsd[2] & 0x8000)
    {
        csd->flags |= kMMC_CsdReadBlockPartial;
    }
    if (rawCsd[2] & 0x4000)
    {
        csd->flags |= kMMC_CsdWriteBlockMisalign;
    }
    if (rawCsd[2] & 0x2000)
    {
        csd->flags |= kMMC_CsdReadBlockMisalign;
    }
    if (rawCsd[2] & 0x1000)
    {
        csd->flags |= kMMC_CsdDsrImplemented;
    }
    csd->c_size = (uint16_t)(((rawCsd[2]&0x300)<<2) + ((rawCsd[2]&0xFF) << 2) 
                               + (rawCsd[1]&0xC0000000)>>30 );
    csd->vdd_r_cur_min = (uint8_t)((rawCsd[1]&0x38000000)>>27);
    csd->vdd_r_cur_max = (uint8_t)((rawCsd[1]&0x07000000)>>24);
    csd->vdd_w_cur_min = (uint8_t)((rawCsd[1]&0x00E00000)>>21);
    csd->vdd_w_cur_max = (uint8_t)((rawCsd[1]&0x001C0000)>>18);
    csd->c_size_mult = (uint8_t)((rawCsd[1]&0x00038000)>>15);
    csd->eraseGrpSize = (uint8_t)((rawCsd[1]&0x00007C00)>>10);
    csd->eraseGrpSizeMult = (uint8_t)((rawCsd[1]&0x000003E0)>>5);
    csd->wpGrpSize = (uint8_t)(rawCsd[1]&0x0000001F);
    if(rawCsd[0] & 0x80000000)
    {
        csd->flags |= kMMC_CsdWPGroupEnabled;
    }
    csd->defaultEcc = (uint8_t)((rawCsd[0] & 0x60000000) >> 29);
    csd->writeSpeedFactor = (uint8_t)((rawCsd[0] & 0x1C000000) >> 26); 
    csd->maxWriteBlkLen = (uint8_t)((rawCsd[0] & 0x03C00000) >> 22);
    if(rawCsd[0] & 0x00200000)
    {
        csd->flags |= kMMC_CsdWriteBlockPartial;
    }
    if(rawCsd[0] & 0x00010000)
    {
        csd->flags |= kMMC_ContentProtectApp;
    }
    if(rawCsd[0] & 0x00008000)
    {
        csd->flags |= kMMC_CsdFileFormatGroup;
    }
    if(rawCsd[0] & 0x00004000)
    {
        csd->flags |= kMMC_CsdCopy;
    }
    if(rawCsd[0] & 0x00002000)
    {
        csd->flags |= kMMC_CsdPermWriteProtect;
    }
    if(rawCsd[0] & 0x00001000)
    {
        csd->flags |= kMMC_CsdTmpWriteProtect;
    }
    csd->fileFormat = (uint8_t)((rawCsd[0] & 0x00000C00) >> 10);
    csd->eccCode = (uint8_t)((rawCsd[0] & 0x00000300) >> 8);

    /* Calculate the device size */
    MMC_CalculateTotalBlockCount(card); 
}

/*!
 * @brief The divide value used to avoid fload point calculation
 */
#define MULT_DIV_IN_TRAN_SPEED 10;

/*FUNCTION****************************************************************
 *
 * Function Name: MMC_SetToMaxClockInNormalMode
 * Description: Sets the card to max transfer speed in non-high speed mode.
 *
 *END*********************************************************************/
static status_t MMC_SetToMaxClockInNormalMode(mmc_card_t *card)
{
    uint32_t freqUnit;
    uint32_t multFactor;
    uint32_t maxFreqInNormal; 
    sdhc_sd_clock_config_t sdClockConfig = {0};
    assert(card);
    
    /* g_fsdhcCmdUnitInTranSpeed and g_mult10InTranSpeed are used to calculate the max speed 
      in 1 bit mode of card.
      To MMC card: For cards supporting version 4.0, 4.1, and 4.2 of the specification,
      the value shall be 20MHz (0x2A). For cards supporting version 4.3, the value 
      shall be 26 MHz (0x32). In High speed mode, the max fsdhcCmduence is decided by 
      CARD_TYPE in EXT_CSD. 
      To SD card: Note that for current SD Memory Cards, this field shall be always 
      0_0110_010b (032h) which is equal to 25 MHz - the mandatory maximum operating 
      fsdhcCmduency of SD Memory Card. In High-Speed mode, this field shall be always 
      0_1011_010b (05Ah) which is equal to 50 MHz, and when the timing mode returns 
      to the default by CMD6 or CMD0 commandMemory, its value will be 032h. */
    /* FsdhcCmduence unit defined in TRAN-SPEED field in CSD */
    uint32_t g_freqUnitInTranSpeed[] = {100000, 1000000, 10000000, 100000000};
    /* The multiplying value defined in TRAN-SPEED field in CSD */
    uint32_t g_mult10InTranSpeed[]   = {0, 10, 12, 13, 15, 20, 26, 30, 35, 40, 45,\
                               52, 55, 60, 70, 80};
    
    freqUnit = g_freqUnitInTranSpeed[RD_MMC_TRAN_SPEED_FREQ_UNIT(card->csd)];
    multFactor = g_mult10InTranSpeed[RD_MMC_TRAN_SPEED_MULT(card->csd)];
    maxFreqInNormal = (fsdhcCmdUnitInCsd*mult10InCsd)/MULT_DIV_IN_TRAN_SPEED;

    sdClockConfig.enableSdClock = true;
    sdClockConfig.baseClockFreq = host->capability->sourceClockFreq;
    sdClockConfig.sdClockFreq = maxFreqInNormal;    
    SDHC_SetSdClockConfig(host->base, &sdClockConfig);

    return kStatus_Success;
}

/*FUNCTION****************************************************************
 *
 * Function Name: MMC_CalculateLegacyEraseUnitSize
 * Description: Calculate legacy erase unit size
 *
 *END*********************************************************************/
static void MMC_CalculateLegacyEraseUnitSize(mmc_card_t *card)
{
    uint32_t erase_group_size, erase_group_mult;
    assert(card);
    
    erase_group_size = card->csd.eraseGrpSize;
    erase_group_mult = card->csd.eraseGrpSizeMult;
    card->eraseGroupSize = ((erase_group_size + 1)*(erase_group_mult + 1));
    card->writeProtectGroupSize = (card->csd.wpGrpSize + 1);
}

/*FUNCTION****************************************************************
 *
 * Function Name: MMC_CalculateHighCapacityEraseUnitSize
 * Description: Calculate high capacity erase unit size
 *
 *END*********************************************************************/
static void MMC_CalculateHighCapacityEraseUnitSize(mmc_card_t *card)
{
    assert(card);

    if((0 == card->extCsd.HC_ERASE_GRP_SIZE) | (0 == card->extCsd.ERASE_TIMEOUT_MULT) )
    {
        MMC_CalculateLegacyEraseUnitSize(card);
        return;
    }
    card->eraseGroupSize = (card->extCsd.HC_ERASE_GRP_SIZE*1024); 
    card->writeProtectGroupSize = card->extCsd.hc_wp_grp_size;
}

/*FUNCTION****************************************************************
 *
 * Function Name: MMC_CalculateEraseUnitSize
 * Description: Calculate erase unit size of the card
 *
 *END*********************************************************************/
static void MMC_CalculateEraseUnitSize(mmc_card_t *card)
{
    assert(card);

    /* If the master enables bit ¡°0¡± in the extended CSD register byte [175], 
    the slave uses high capacity value for the erase operation */
    if((card->extCsd.ERASE_GROUP_DEF)&0x01)
    {
        MMC_CalculateLegacyEraseUnitSize(card);
    }
    else
    {
        MMC_CalculateHighCapacityEraseUnitSize(card);
    }
}

/*FUNCTION****************************************************************
 *
 * Function Name: MMC_SetExtCsdByte
 * Description: Sets the specific field of the card according to the index and 
 * value inputted by the user.
 *
 *END*********************************************************************/
static status_t MMC_SetExtCsdByte(mmc_card_t *card, mmc_ext_csd_operation_t *operation)
{
    sdhc_cmd_t command = {0};
    sdhc_host_t *host;
    assert(card);
    assert(card->host);
    assert(operation);

    host = card->host;
    uint32_t param = 0;
    status_t err = kStatus_Success;  
    
    param |= (operation->cmdSet << MMC_SWITCH_CMD_SET_SHIFT);
    param |= (operation->value << MMC_SWITCH_VALUE_SHIFT);
    param |= (operation->indexOfByte << MMC_SWITCH_INDEX_OF_BYTE_SHIFT);
    param |= (operation->accessMode << MMC_SWITCH_ACCESS_MODE_SHIFT);
    
    command.index = kMMC_Switch;
    command.argument = param;
    command.responseType = kSDHC_ReponseTypeR1b;/* Send switch commandMemory to set the pointed byte*/

    host->currentCmd = &command;
    host->currentData = 0;

    if ((kStatus_Success != SDHC_SendCmdBlocking(host, FSL_CARD_COMMAND_TIMEOUT)) 
        || (!SDMMC_R1_ERROR_BITS(command->response[0])))    
    {         
        return kStatus_SDMMC_SwitchFailed;
    }

    /* wait for the card to be out of BUSY */
    if(kStatus_Success != MMC_SendStatus(card))
    {
        return kStatus_SDMMC_SendStatusFailed;
    } 

    return kStatus_Success;   
}


/*FUNCTION****************************************************************
 *
 * Function Name: MMC_EnableHighCapacityEraseUnitSize
 * Description: Enables the high capacity erase feature of the card.
 *
 *END*********************************************************************/
static void MMC_EnableHighCapacityEraseUnitSize(mmc_card_t *card)
{
    /* Enable the high capacity erase unit */
    mmc_ext_csd_operation_t operation;
    assert(card);

    operation.accessMode = kMMC_ExtCsdSetBits;
    operation.indexOfByte = kMMC_ExtCsdIndexEraseGroupDef;
    operation.value = 0x01;/* The high capacity erase unit size enablement bit is bit 0*/
    MMC_SetExtCsdByte(card, &operation);
}

/*FUNCTION****************************************************************
 *
 * Function Name: MMC_DecodeExtCsd
 * Description: Decodes the EXT_CSD register
 *
 *END*********************************************************************/
static status_t MMC_DecodeExtCsd(uint32_t *rawExtCsd, mmc_card_t *card)
{
    mmc_ext_csd_t *extCsd;
    assert(rawExtCsd);
    assert(card);
    
    extCsd = &(card->extCsd);
    extCsd->supportedCmdSet = (rawExtCsd[1]&0x000000FF);
    extCsd->bootInfo = (rawExtCsd[70]&0x000000FF);
    if(kMMC_SupportAlterBoot == extCsd->bootInfo)
    {/* Card support alternate boot*/
        card->flags |= MMC_flags_ALTER_BOOT;
    }
    extCsd->bootSizeMult = ((rawExtCsd[71]&0x00FF0000)>>16);
    /* Get boot partition size*/
    card->bootPartitionSize = ((128*1024*extCsd->bootSizeMult)
                                    /FSL_CARD_DEFAULT_BLOCK_SIZE);
      
    extCsd->accessSize = ((rawExtCsd[71]&0x0000FF00)>>8);
    extCsd->HC_ERASE_GRP_SIZE = ((rawExtCsd[71]&0x000000FF));
    extCsd->ERASE_TIMEOUT_MULT = ((rawExtCsd[72]&0xFF000000)>>24);
    extCsd->reliableWriteSectorCount = ((rawExtCsd[72]&0x00FF0000)>>16);
    extCsd->hc_wp_grp_size = ((rawExtCsd[72]&0x0000FF00)>>8);
    extCsd->sleepCurrentVCC = (rawExtCsd[72]&0x000000FF);
    extCsd->sleepCurrentVCCQ = ((rawExtCsd[73]&0xFF000000)>>24);
    extCsd->slpAwkTimeout = ((rawExtCsd[73]&0x0000FF00)>>8);
    extCsd->sectorCount = rawExtCsd[74];
    extCsd->MIN_PERF_W_8_52 = ((rawExtCsd[75]&0x00FF0000)>>16);
    extCsd->MIN_PERF_R_8_52 = ((rawExtCsd[75]&0x0000FF00)>>8);
    extCsd->MIN_PERF_W_8_26_4_52 = (rawExtCsd[75]&0x000000FF);
    extCsd->MIN_PERF_R_8_26_4_52 = ((rawExtCsd[76]&0xFF000000)>>24);
    extCsd->MIN_PERF_W_4_26 = ((rawExtCsd[76]&0x00FF0000)>>16);
    extCsd->MIN_PERF_R_4_26 = ((rawExtCsd[76]&0x0000FF00)>>8);
    extCsd->PWR_CL_26_360 = ((rawExtCsd[77]&0xFF000000)>>24);
    extCsd->PWR_CL_52_360 = ((rawExtCsd[77]&0x00FF0000)>>16);
    extCsd->PWR_CL_26_195 = ((rawExtCsd[77]&0x0000FF00)>>8);
    extCsd->PWR_CL_52_195 = (rawExtCsd[77]&0x000000FF);
    extCsd->cardType = (rawExtCsd[78]&0x000000FF);
    extCsd->csdStrucVer = ((rawExtCsd[79]&0x00FF0000)>>16);
    extCsd->extCsdVer = (rawExtCsd[79]&0x000000FF);
    /* If ext_csd version is V4.3, high capacity erase feature can only be enabled
    in MMC V4.3 */
    if(extCsd->extCsdVer == kMMC_ExtCsdVer13)
    {
        /* Enable the high capacity erase unit */
        MMC_EnableHighCapacityEraseUnitSize(card);
    }
    extCsd->cmdSet = ((rawExtCsd[80]&0xFF000000)>>24);  
    extCsd->cmdSetRev = ((rawExtCsd[80]&0x0000FF00)>>8);
    extCsd->powerClass = ((rawExtCsd[81]&0xFF000000)>>24);
    extCsd->highSpeedTiming = ((rawExtCsd[81]&0x0000FF00)>>8);
    extCsd->busWidth = ((rawExtCsd[82]&0xFF000000)>>24);
    extCsd->erasedMemCnt = ((rawExtCsd[82]&0x0000FF00)>>8);
    extCsd->bootConfig = ((rawExtCsd[83]&0xFF000000)>>24);
    extCsd->bootBusWidth = ((rawExtCsd[83]&0x0000FF00)>>8);
    extCsd->ERASE_GROUP_DEF = ((rawExtCsd[84]&0xFF000000)>>24);
    /* Calculate the erase unit size */
    MMC_CalculateEraseUnitSize(card);
    return kStatus_Success;
} 

/*FUNCTION****************************************************************
 *
 * Function Name: MMC_SendExtCsd
 * Description: Get the content of the EXT_CSD register
 *
 *END*********************************************************************/
static status_t MMC_SendExtCsd(mmc_card_t *card)
{
    sdhc_cmd_t command = {0};
    sdhc_host_t *host;

    assert(card);
    assert(card->host);

    host = card->host;
    sdhc_data_t data = {0};
    uint32_t index;

    command.index = kMMC_SendExtCsd;
    command.argument = 0;
    command.responseType = kSDHC_ReponseTypeR1;

    data.flags = CARD_DATA_FLAGS_DATA_READ;
    data.blockCount = 1;
    data.blockSize = MMC_EXT_CSD_LEN_AS_BYTE;
    data.buffer = card->rawExtCsd;

    host->currentCmd = &command;
    host->currentData = &data;

    if ((kStatus_Success == SDHC_SendCmdBlocking(host, FSL_CARD_COMMAND_TIMEOUT)) 
        && (SDMMC_R1_ERROR_BITS(command->response[0]))
        && (kStatus_Success == SDHC_WaitDataTransferComplete(host, FSL_CARD_COMMAND_TIMEOUT)))
    {        
        /*The response is from bit 127:8 in R2, corrsponding to command->response[3]:command->response[0][31:8]*/
        for(index = 0; index < MMC_EXT_CSD_LEN_AS_WORD; index++)
        {
            if (kSDHC_EndianModeLittle == host->sdhcConfig.endianMode)
            {
                card->rawExtCsd[index] = SWAP_UINT32_IN_LITTLE_ENDIAN(card->rawExtCsd[index]);
            }
        }

        MMC_DecodeExtCsd(card->rawExtCsd, card);
        
        return kStatus_Success;
    }

    return kStatus_SDMMC_SendCommandFailed;
}

/*FUNCTION****************************************************************
 *
 * Function Name: MMC_GetHighSpeedFreqence
 * Description: Gets the bus frequence when card in high speed mode
 *
 *END*********************************************************************/
static status_t MMC_GetHighSpeedFreqence(mmc_card_t *card)
{
    assert(card);

    /* This field defines the type of the card. The only currently valid values \
    for this field are 0x01 and 0x03. */
    if (card->extCsd.cardType & kMMC_CardTypeHSFreq26MHZ)
    {
        card->flags |= kMMC_HighSpeedIs26MHZ;
    }
    else if (card->extCsd.cardType & kMMC_CardTypeHSFreq52MHZ)
    {
        card->flags |= kMMC_HighSpeedIs52MHZ;
    }
    else
    {
        return kStatus_SDMMC_CardNotSupport;
    }
    return kStatus_Success;
}

/*FUNCTION****************************************************************
 *
 * Function Name: MMC_GetPowerClass
 * Description: Gets the power class of the card at specific bus width and 
 * specific voltage
 *
 *END*********************************************************************/
static status_t MMC_GetPowerClass(mmc_card_t *card, uint8_t *powerClass, mmc_bus_width_t busWidth)
{  
    uint8_t mask;  
    assert(card);

    if (kMMC_BusWidth1b == busWidth)
    {
        return kStatus_SDMMC_CardNotSupport;
    }    
    if (kMMC_BusWidth4b == busWidth)
    {
        mask = MMC_EXT_CSD_PWRCLFFVV_4BIT_MASK;/* the mask of 4 bit bus width's power class*/
    }
    else
    {
        mask = MMC_EXT_CSD_PWRCLFFVV_8BIT_MASK; /* the mask of 8 bit bus width's power class*/  
    }
    if ((card->flags & kMMC_HighSpeedIs52MHZ) && (kMMC_Volt170to195 == card->hostVoltRange) )
    {
        *powerClass = ((card->extCsd.PWR_CL_52_195) & mask);
    }
    else if((card->flags & kMMC_HighSpeedIs52MHZ) && (kMMC_Volt270to360 == card->hostVoltRange))
    {
        *powerClass = ((card->extCsd.PWR_CL_52_360) & mask);
    }
    else if((card->flags & kMMC_HighSpeedIs26MHZ) && (kMMC_Volt170to195 == card->hostVoltRange))
    {
        /* 8 bit at 26MHZ/195V*/
        *powerClass = ((card->extCsd.PWR_CL_26_195) & mask);
    }
    else if((card->flags & kMMC_HighSpeedIs26MHZ) && (kMMC_Volt270to360 == card->hostVoltRange))
    {
        /* 8 bit at 26MHZ/360V*/
        *powerClass = ((card->extCsd.PWR_CL_26_360) & mask);
    }
    else
    {
        //have some error
        return kStatus_Fail;
    }
    return kStatus_Success;
}

/*FUNCTION****************************************************************
 *
 * Function Name: MMC_SendTestPattern
 * Description: Sends test pattern to get the functional pin in the MMC bus
 *
 *END*********************************************************************/
static status_t MMC_SendTestPattern(mmc_card_t *card, uint32_t blockSize, uint32_t *buffer)
{
    sdhc_host_t *host;
    sdhc_cmd_t command = {0};
    sdhc_data_t data = {0};
    assert(card);
    assert(card->host);
    assert(blockSize <= FSL_CARD_DEFAULT_BLOCK_SIZE);
    assert(buffer);

    host = card->host;
    
    command.index = kSDMMC_SendTuningBlock;
    command.argument = 0;
    command.responseType = kSDHC_ReponseTypeR1;

    data.blockCount = 1;
    data.blockSize = blockSize;
    data.buffer = buffer;

    host->currentCmd = &command;
    host->currentData = &data;

    if ((kStatus_Success == SDHC_SendCmdBlocking(host, FSL_CARD_COMMAND_TIMEOUT))
        || (!SDMMC_R1_ERROR_BITS(command->response[0]))
        || (kStatus_Success != SDHC_WaitDataTransferComplete(host, FSL_CARD_COMMAND_TIMEOUT)))  
    {     
        return kStatus_SDMMC_SendCommandFailed;
    }

    return kStatus_Success;
}

/*FUNCTION****************************************************************
 *
 * Function Name: MMC_GetTestPattern
 * Description: Gets test pattern reversed by the card after the send-test-pattern
 * commandMemory
 *
 *END*********************************************************************/
static status_t MMC_GetTestPattern(mmc_card_t *card, uint32_t blockSize, uint32_t *buffer)
{
    sdhc_host_t *host;
    sdhc_cmd_t command = {0};
    sdhc_data_t data = {0};
    assert(card);
    assert(card->host);
    assert(blockSize <= FSL_CARD_DEFAULT_BLOCK_SIZE);
    assert(buffer);

    host = card->host;
    
    command.index = kMMC_BusTestRead;
    command.responseType = kSDHC_ReponseTypeR1;

    data.blockCount = 1;
    data.blockSize = blockSize;
    data.buffer = buffer;
    data.flags |= CARD_DATA_FLAGS_DATA_READ;

    host->currentCmd = &command;
    host->currentData = &data;

    if ((kStatus_Success != SDHC_SendCmdBlocking(host, FSL_CARD_COMMAND_TIMEOUT))
        || (!SDMMC_R1_ERROR_BITS(command->response[0]))
        || (kStatus_Success != SDHC_WaitDataTransferComplete(host, FSL_CARD_COMMAND_TIMEOUT)))  
    {         
        return kStatus_SDMMC_SendCommandFailed;
    }

    return kStatus_Success;   
}

/*FUNCTION****************************************************************
 *
 * Function Name: MMC_BusTestProc
 * Description: Bus test procedure to get the functional pin in the bus
 *
 *END*********************************************************************/
static status_t MMC_BusTestProc(mmc_card_t *card, mmc_bus_width_t busWidth)
{
    uint32_t blockSize;
    uint32_t sendPattern[kMMC_BusWidth8b];
    uint32_t recvPattern[kMMC_BusWidth8b];
    bool testPass = false; /* Test procedure not passed*/
    assert(card);
    assert(busWidth <= kMMC_BusWidth8b);
    
    /* For 8 data lines the data block would be (MSB to LSB): 0x0000_0000_0000_AA55,
    For 4 data lines the data block would be (MSB to LSB): 0x0000_005A, 
    For only 1 data line the data block would be: 0x80*/
    if(kMMC_BusWidth8b == busWidth)
    {
        blockSize = 8;
        sendPattern[0] = MMC_8BIT_BUS_TEST_PATTERN;
        sendPattern[1] = 0;        
    }
    else if(kMMC_BusWidth4b == busWidth)
    {
        blockSize = 4;
        sendPattern[0] = MMC_4BIT_BUS_TEST_PATTERN;
    }
    else
    {
        blockSize  = 1;
        sendPattern[0]  = MMC_1BIT_BUS_TEST_PATTERN;
    }
    if(kStatus_Success != MMC_SendTestPattern(card, blockSize, sendPattern))
    {
        return kStatus_Fail;
    }
    if(kStatus_Success != MMC_GetTestPattern(card, blockSize, recvPattern))
    {
        return kStatus_Fail;
    }
    /* XOR the send pattern and recv pattern */
    if(kMMC_BusWidth8b == busWidth)
    {
        if( (sendPattern[0]^recvPattern[0]) == MMC_8BIT_BUS_PATTERN_XOR_RESULT )
        {
            testPass = true; /* Test procedure passed */
        }
    }
    else if(kMMC_BusWidth4b == busWidth)
    {
        if( (sendPattern[0]^recvPattern[0]) == MMC_4BIT_BUS_PATTERN_XOR_RESULT )
        {
            testPass = true; /* Test procedure passed */
        }
    }
    else if(kMMC_BusWidth1b == busWidth)
    {
        if( (sendPattern[0]^recvPattern[0]) == MMC_1BIT_BUS_PATTERN_XOR_RESULT )
        {
            testPass = true; /* Test procedure passed */
        }
    }
    if(!testPass)
    {
        return kStatus_SDMMC_BusTestProcessFailed;
    }
    return kStatus_Success;    
}


/*FUNCTION****************************************************************
 *
 * Function Name: MMC_SetBusWidth
 * Description: Sets the bus width.
 *
 *END*********************************************************************/
static status_t MMC_SetBusWidth(mmc_card_t *card, mmc_bus_width_t busWidth)
{    
    uint8_t powerClassAt8bit;
    mmc_ext_csd_operation_t extCsdOperation;
    assert(card);

    if(kStatus_Success != MMC_GetPowerClass(card, &powerClassAt8bit, busWidth))
    {
        return kStatus_SDMMC_GetPowerClassFailed;
    } 

    /* Set power class of pointed bus width */
    extCsdOperation.accessMode = kMMC_ExtCsdWriteBits;
    extCsdOperation.indexOfByte = kMMC_ExtCsdIndexPowerClass;
    extCsdOperation.value = powerClassAt8bit;
    if(kStatus_Success != MMC_SetExtCsdByte(card, &extCsdOperation))
    {
        return kStatus_SDMMC_SetPowerClassFailed;
    }          

    /* Set bus width of pointed bus width */
    extCsdOperation.accessMode = kMMC_ExtCsdWriteBits;
    extCsdOperation.indexOfByte = kMMC_ExtCsdIndexBusWidth;
    extCsdOperation.value = busWidth;      
    if(kStatus_Success != MMC_SetExtCsdByte(card, &extCsdOperation))
    {
        return kStatus_SDMMC_SetBusWidthFailed;
    }    
    return kStatus_Success;    
}

/*FUNCTION****************************************************************
 *
 * Function Name: MMC_SwitchHighSpeed
 * Description: Switches the card to high speed mode
 *
 *END*********************************************************************/
static status_t MMC_SwitchHighSpeed(mmc_card_t *card)
{    
    mmc_ext_csd_operation_t extCsdOperation; 
    sdhc_sd_clock_config_t sdClockConfig = {0};   
    assert(card);
   
    /* If host support high speed mode, then switch to high speed. */
    if (DOES_SDHC_SUPPORT_HIGHSPEED(card->host))
    {       
        /* Switch to high speed timing */
        extCsdOperation.accessMode = kMMC_ExtCsdWriteBits;
        extCsdOperation.indexOfByte = kMMC_ExtCsdIndexHSTiming;
        extCsdOperation.value = kMMC_HighSpeedTiming;
        if(kStatus_Success != MMC_SetExtCsdByte(card, &extCsdOperation))
        {
            return kStatus_Fail;
        }   
        if(kStatus_Success != MMC_GetHighSpeedFreqence(card))
        {
            return kStatus_Fail;
        }
        /* Switch to corresponding fsdhcCmduence in high speed mode  */
        if((card->flags & kMMC_HighSpeedIs52MHZ) )
        {
            sdClockConfig.enableSdClock = true;
            sdClockConfig.baseClockFreq = host->capability->sourceClockFreq;
            sdClockConfig.sdClockFreq = MMC_CLK_52MHZ;    
            SDHC_SetSdClockConfig(host->base, &sdClockConfig);
        }
        else if((card->flags & kMMC_HighSpeedIs26MHZ))
        {
            sdClockConfig.enableSdClock = true;
            sdClockConfig.baseClockFreq = host->capability->sourceClockFreq;
            sdClockConfig.sdClockFreq = MMC_CLK_26MHZ;    
            SDHC_SetSdClockConfig(host->base, &sdClockConfig);
        }
        else
        {
            return kStatus_Fail;
        }
    }
    return kStatus_Success;    
}

/*FUNCTION****************************************************************
 *
 * Function Name: MMC_DecodeCid
 * Description: Decodes cid register
 *
 *END*********************************************************************/
static void MMC_DecodeCid(uint32_t *rawCid, mmc_card_t *card)
{
    mmc_cid_t *cid;
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
 * Function Name: MMC_AutoSetBusWidth
 * Description: Sets the bus width automatically
 *
 *END*********************************************************************/
static status_t MMC_AutoSetBusWidth(mmc_card_t *card)
{
    uint8_t i;
    mmc_bus_width_t maxBusWidth, busWidth[] = {kMMC_BusWidth1b, kMMC_BusWidth4b, kMMC_BusWidth8b};
    assert(card);
    
    /* Get max width of data bus */
    for(i = 0; i < MMC_BUS_WIDTH_TYPE_NUM; i++)
    {
        if(kStatus_Success == MMC_BusTestProc(card, busWidth[i]))
        {
            maxBusWidth = busWidth[i];
            break;
        } 
    }
    if(i == MMC_BUS_WIDTH_TYPE_NUM)/* Board haven't functional pin. */
    {
        return kStatus_SDMMC_BusTestProcessFailed;
    }

     /* From the EXT_CSD the host can learn the power class of the card, and choose to work with a wider data bus */
    if(kStatus_Success != MMC_SetBusWidth(card, maxBusWidth))
    {
        return kStatus_SDMMC_SetBusWidthFailed;
    }
    if((kMMC_BusWidth4b == maxBusWidth) && DOES_SDHC_SUPPORT_4BITS(card->host))
    {
        SDHC_SetDataTransferWidth(card->host->base, kSDHCDtw4Bit);
    }               
    else if((kMMC_BusWidth8b == maxBusWidth) && DOES_SDHC_SUPPORT_8BITS(card->host))
    {
        SDHC_SetDataTransferWidth(card->host->base, kSDHCDtw8Bit);
    }
    return kStatus_Success;
}

/*FUNCTION****************************************************************
 *
 * Function Name: MMC_AllSendCid
 * Description: Sends all_send_cid commandMemory
 *
 *END*********************************************************************/
static status_t MMC_AllSendCid(mmc_card_t *card)
{
    sdhc_cmd_t command = {0};
    sdhc_data_t data = {0};
    sdhc_host_t *host; 
    assert(card);
    assert(card->host);

    host = card->host;

    command.index = kSDMMC_AllSendCid;
    command.argument = 0;
    command.responseType = kSDHC_ReponseTypeR2;

    host->currentCmd = &command;
    host->currentData = &data;

    if (kStatus_Success == SDHC_SendCmdBlocking(host, FSL_CARD_COMMAND_TIMEOUT))
    {
        memcpy(card->rawCid, command->response, sizeof(card->rawCid));
        MMC_DecodeCid(command->response, card);    
 
        return kStatus_Success;
    }
 
    return kStatus_Fail;
}

/*FUNCTION****************************************************************
 *
 * Function Name: MMC_SendCsd
 * Description: get csd from card
 *
 *END*********************************************************************/
static status_t MMC_SendCsd(mmc_card_t *card)
{
    sdhc_cmd_t command = {0};
    sdhc_data_t *data = {0};
    sdhc_host_t *host;
    assert(card);
    assert(card->host);

    host = card->host;

    command.index = kSDMMC_SendCsd;
    command.argument = card->rca << 16;
    command.responseType = kSDHC_ReponseTypeR2;
    
    host->currentCmd = &command;
    host->currentData = data;

    if (kStatus_Success == SDHC_SendCmdBlocking(host, FSL_CARD_COMMAND_TIMEOUT))
    {
        memcpy(card->rawCsd, command->response, sizeof(card->rawCsd));
        /*The response is from bit 127:8 in R2, corrisponding to command->response[3]:command->response[0][31:8]*/
        MMC_DecodeCsd(command->response, card);

        return kStatus_Success;
    }

    return kStatus_Fail;
}

/*FUNCTION****************************************************************
 *
 * Function Name: MMC_Init
 * Description: Initializes the MMCCARD
 *
 *END*********************************************************************/
status_t MMC_Init(mmc_card_t *card)
{
    status_t bootRes;
    sdhc_sd_clock_config_t sdClockConfig = {0};
    assert(card);
    
    /* Set clock to 400KHz, or less */
    sdClockConfig.enableSdClock = true;
    sdClockConfig.baseClockFreq = host->capability->sourceClockFreq;
    sdClockConfig.sdClockFreq = SDMMC_CLK_400KHZ;    
    SDHC_SetSdClockConfig(host->base, &sdClockConfig);
    
    /* Send CMD0 to reset the bus */
    if(kStatus_Success != MMC_GoIdle(card))
    {
        return kStatus_SDMMC_GoIdleFailed;
    }

    /* Apply power to the bus, communication voltage range (2.7-3.6V) */
    /* Validate the voltage range */
    if(kStatus_Success != MMC_ValidateVolt(card, kMMC_Volt270to360))
    {
        return kStatus_SDMMC_SendOpCondFailed;
    }

    /* Get card CID */
    if(kStatus_Success != MMC_AllSendCid(card))
    {
        return kStatus_SDMMC_AllSendCidFailed;
    }

    /* Set the card address */
    if(kStatus_Success != MMC_SetRelativeAddress(card))
    {
        return kStatus_SDMMC_SetRcaFailed;
    }

    /* Get the CSD register content */
    if(kStatus_Success != MMC_SendCsd(card))
    {
        return kStatus_SDMMC_SendCsdFailed;
    }

    /* If necessary, adjust the host parameters according to the information in the CSD */
    if(card->csd.sysSpecVer == 4)
    {
        card->flags |= kMMC_HighSpeed;
    }
    else
    {
        /* Card is old MMC card */
        return kStatus_SDMMC_CardNotSupport;
    }

    /* Send CMD7 with the card's RCA to place the card in tran state. Puts current selected card in trans state */
    if(kStatus_Success != MMC_SelectCard(card, true))
    {
        return kStatus_SDMMC_SelectCardFailed;
    }

    /* Get EXT_CSD register content */
    if(kStatus_Success != MMC_SendExtCsd(card))
    {
        return kStatus_SDMMC_SendExtCsdFailed;
    }    

    if(kStatus_Success != MMC_AutoSetBusWidth(card))/* Sets card data width and block size  */
    {
        return kStatus_SDMMC_SetBusWidthFailed;
    }

    /* Switch to high speed mode */  
    if(kStatus_Success != MMC_SwitchHighSpeed(card))
    {
        return kStatus_SDMMC_SwitchHighSpeedFailed;
    }

    if (MMC_SetBlockSize(card, FSL_CARD_DEFAULT_BLOCK_SIZE))
    {
        return kStatus_SDMMC_SetCardBlockSizeFailed;
    }

    /* Set default access non-boot partition */
    card->currentPartition = kMMC_AccessBootPartitionNot;
    return kStatus_Success;
}


/*FUNCTION****************************************************************
 *
 * Function Name: MMC_Erase
 * Description:  Erases the MMC card content
 *
 *END*********************************************************************/
static status_t MMC_Erase(mmc_card_t *card, uint32_t eraseGroupStart, uint32_t groupCount)
{
    uint32_t s, e;
    sdhc_cmd_t command = {0};
    assert(card);
    assert(card->host);
    assert(groupCount);    

    host = card->host;

    /* Calculate the start group number and end group number */
    s = eraseGroupStart;
    e = s + groupCount - 1;
    if(card->flags & kMMC_HighCapacity)
    {
        /* The implementation of a higher than 2GB of density of memory will not
      be backwards compatible with the lower densities.First of all the address 
      argument for higher than 2GB of density of memory is changed to be sector 
      address (512B sectors) instead of byte address */
        s = (s * (card->eraseGroupSize));
        e = (e * (card->eraseGroupSize));
    }
    else
    {
        /* The address unit is byte when card capacity is lower than 2GB*/
        s = (s * (card->eraseGroupSize)*FSL_CARD_DEFAULT_BLOCK_SIZE);
        e = (e * (card->eraseGroupSize)*FSL_CARD_DEFAULT_BLOCK_SIZE);
    }

    /* Set the start erase group address */         
    command.index = kMMC_EraseGroupStart;
    command.argument = s;
    command.responseType = kSDHC_ReponseTypeR1;

    host->currentCmd = &command;
    host->currentData = 0;

    if ((kStatus_Success != SDHC_SendCmdBlocking(host, FSL_CARD_COMMAND_TIMEOUT)) 
        || (!SDMMC_R1_ERROR_BITS(command->response[0])))
    { 
        return kStatus_Fail;
    }

    /* Set the end erase group address */
    command.index = kMMC_EraseGroupEnd;
    command.argument = e;

    host->currentCmd = &command;
    host->currentData = 0;

    if ((kStatus_Success != SDHC_SendCmdBlocking(host, FSL_CARD_COMMAND_TIMEOUT))
        || (!SDMMC_R1_ERROR_BITS(command->response[0])))
    {
        return kStatus_Fail;
    }

    /* Start the erase process */
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

    /* Wait the write and program process complete in the card */
    if (kStatus_Success != MMC_SendStatus(card))
    {
        return kStatus_SDMMC_SendStatusFailed;
    }

    return kStatus_Success;
}

/*FUNCTION****************************************************************
 *
 * Function Name: MMC_SelectPartition
 * Description:  Selects the partition used to read/write data after initilized 
 *
 *END*********************************************************************/
status_t MMC_SelectPartition(mmc_card_t *card, mmc_access_partition_t partitionNumber)
{
    mmc_ext_csd_operation_t extCsdOperation;
    uint8_t bootConfig;
    assert(card);
    
    bootConfig  = card->extCsd.bootConfig;
    bootConfig &= ~MMC_BOOT_CONFIG_PART_ACCESS_MASK;
    bootConfig |= (partitionNumber << MMC_BOOT_CONFIG_PART_ACCESS_SHIFT);
    extCsdOperation.accessMode = kMMC_ExtCsdWriteBits;
    extCsdOperation.indexOfByte = kMMC_ExtCsdIndexBootConfig;
    extCsdOperation.value = bootConfig;      
    if(kStatus_Success != MMC_SetExtCsdByte(card, &extCsdOperation))
    {
        return kStatus_SDMMC_ConfigBootFailed;
    }

    card->extCsd.bootConfig = bootConfig;   
    /* Save current access partition number */
    card->currentPartition = partitionNumber;

    return kStatus_Success;
}

/*FUNCTION****************************************************************
 *
 * Function Name: MMC_ConfigBoot
 * Description:  Configures boot activity after power on 
 *
 *END*********************************************************************/
status_t MMC_ConfigBoot(mmc_card_t *card, const mmc_boot_config_t *configPtr)
{
    uint8_t bootParam;
    mmc_ext_csd_operation_t extCsdOperation;
    assert(card);
    assert(configPtr);

    if(kMMC_ExtCsdVer13 != card->extCsd.extCsdVer)/* Only V4.3 support fast boot */
    {
        return kStatus_SDMMC_NotSupportYet;
    }

    /* Set the BOOT_CONFIG field of EXT_CSD */
    bootParam = card->extCsd.bootConfig;

    bootParam &= ~(MMC_BOOT_CONFIG_ACK_MASK);
    bootParam &= ~(MMC_BOOT_CONFIG_PART_ENABLE_MASK);
    
    bootParam |= ((configPtr->enableBootAck ? 1 : 0) << MMC_BOOT_CONFIG_ACK_SHIFT);
    bootParam |= ((configPtr->bootPartition) << MMC_BOOT_CONFIG_PART_ENABLE_SHIFT);
    
    extCsdOperation.accessMode = kMMC_ExtCsdWriteBits;
    extCsdOperation.indexOfByte = kMMC_ExtCsdIndexBootConfig;
    extCsdOperation.value = bootParam;      
    if(kStatus_Success != MMC_SetExtCsdByte(card, &extCsdOperation))
    {
        return kStatus_SDMMC_ConfigBootFailed;
    }    

    card->extCsd.bootConfig = bootParam;

    /*Set BOOT_BUS_WIDTH in EXT_CSD */
    bootParam = card->extCsd.bootBusWidth;

    bootParam &= ~(MMC_BOOT_BUS_WIDTH_RESET_MASK);
    bootParam &= ~(MMC_BOOT_BUS_WIDTH_WIDTH_MASK);
    
    bootParam |= ((configPtr->retainBootBusWidth ? 1 : 0) << MMC_BOOT_BUS_WIDTH_RESET_SHIFT);
    bootParam |= (configPtr->bootBusWidth << MMC_BOOT_BUS_WIDTH_WIDTH_SHIFT);  

    extCsdOperation.accessMode = kMMC_ExtCsdWriteBits;
    extCsdOperation.indexOfByte = kMMC_ExtCsdIndexBusWidth;
    extCsdOperation.value = bootParam;      
    if(kStatus_Success != MMC_SetExtCsdByte(card, &extCsdOperation))
    {
        return kStatus_SDMMC_ConfigBootFailed;
    }     

    card->extCsd.bootBusWidth = bootParam;

    return kStatus_Success;
}

static status_t MMC_CheckIORange(mmc_card_t *card, uint32_t startBlock, uint32_t blockCount)
{
    /* Check address range */
    switch(card->currentPartition)
    {
    case kMMC_AccessBootPartitionNot:
      {
          if((startBlock + blockCount + 1) <= (card->blockCount))
          {
              return kStatus_Fail;
          }
          break;
      }
    case kMMC_AccessBootPartition1:
    case kMMC_AccessBootPartition2:
      {
          /* Boot part1 and part2 have the same partition size */
         if((startBlock+blockCount) > card->bootPartitionSize)
         {
             return kStatus_Fail;
         }
          break;
      }
    }

    return kStatus_Success;
}
/*FUNCTION****************************************************************
 *
 * Function Name: MMC_Read
 * Description: Reads data from specific MMC card
 *
 *END*********************************************************************/
static status_t MMC_Read(mmc_card_t *card, uint8_t *buffer, uint32_t startBlock, uint32_t blockSize,
                 uint32_t blockCount)
{
    sdhc_cmd_t command = {0};
    sdhc_data_t data = {0};
    sdhc_host_t *host;
    assert(card);
    assert(card->host);
    assert(buffer);
    assert(blockCount);
    assert(blockSize);
    assert(blockSize == FSL_CARD_DEFAULT_BLOCK_SIZE);

    host = card->host;

    if (((card->flags & kMMC_HighCapacity) && (blockSize != 512)) || (blockSize > card->blockSize)
         || (blockSize > card->host->capability->maxBlockLength) || (blockSize % 4))
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
    else
    {
        if ((!host->sdhcConfig->enableAutoCmd12) && (card->enablePreDefBlkCnt))
        {
            /* If enabled the pre-define count read/write featue of the card, need to set block count firstly */
            if (kStatus_Success != MMC_SetBlockCount(card, blockCount))
            {
                return kStatus_SDMMC_SetBlockCountFailed;
            }
        }         
    }

    command.argument = startBlock;
    if (!(card->flags & kMMC_HighCapacity))
    {
        command.argument *= data.blockSize;
    }
    command.responseType = kSDHC_ReponseTypeR1;

    host->currentCmd = &command;
    host->currentData = &data;
    
    if (kStatus_Success != SDHC_SendCmdBlocking(host, FSL_CARD_COMMAND_TIMEOUT)
        || (!SDMMC_R1_ERROR_BITS(command->response[0]))
        || (kStatus_Success != SDHC_WaitDataTransferComplete(host, FSL_CARD_COMMAND_TIMEOUT)))
    {
        
        return kStatus_SDMMC_SendCommandFailed;
    }

    if ((!host->sdhcConfig->enableAutoCmd12) && (!card->enablePreDefBlkCnt))  
    {
        if (blockCount > 1)
        {
            if (kStatus_Success != MMC_StopTransmission(card))
            {
                return kStatus_SDMMC_StopTransmissionFailed;
            }
        }
    }
 
    return kStatus_Success;
}

/*FUNCTION****************************************************************
 *
 * Function Name: MMC_Write
 * Description: Writes data from specific MMC card
 *
 *END*********************************************************************/
static status_t MMC_Write(mmc_card_t *card, uint8_t *buffer, uint32_t startBlock, uint32_t blockSize, 
                  uint32_t blockCount)
{
    sdhc_cmd_t command = {0};
    sdhc_data_t data = {0};
    sdhc_host_t *host;
    assert(card);
    assert(card->host);
    assert(buffer);
    assert(blockCount);
    assert(blockSize);
    assert(blockSize == FSL_CARD_DEFAULT_BLOCK_SIZE);

    host = card->host;

    /* Check address range */
    if (((card->flags & kMMC_HighCapacity) && (blockSize != 512)) || (blockSize > card->blockSize)
     || (blockSize > card->host->capability->maxBlockLength) || (blockSize % 4))
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
    else
    {
        if ((!host->sdhcConfig->enableAutoCmd12) && (card->enablePreDefBlkCnt))
        {
            /* If enabled the pre-define count read/write featue of the card, need to set block count firstly */
            if(kStatus_Success != MMC_SetBlockCount(card, blockCount))
            {
                return kStatus_SDMMC_SetBlockCountFailed;
            }
        }
    }
    command.argument = startBlock;
    if (!(card->flags & kMMC_HighCapacity))
    {
        command.argument *= blockSize;
    }
    command.responseType = kSDHC_ReponseTypeR1;

    host->currentCmd = &command;
    host->currentData = &data;

    if (kStatus_Success != SDHC_SendCmdBlocking(host, FSL_CARD_COMMAND_TIMEOUT)
        || (!SDMMC_R1_ERROR_BITS(command->response[0]))
        || (kStatus_Success != SDHC_WaitDataTransferComplete(host, FSL_CARD_COMMAND_TIMEOUT)))
    {
        return kStatus_SDMMC_SendCommandFailed;
    }

    if (blockCount > 1)
    {
        if ((!host->sdhcConfig->enableAutoCmd12) && (!card->enablePreDefBlkCnt))   
        {
            if (kStatus_Success != MMC_StopTransmission(card))
            {
                return kStatus_SDMMC_StopTransmissionFailed;
            }
        }          

        if (kStatus_Success != MMC_SendStatus(card))
        { 
            return kStatus_SDMMC_SendStatusFailed;
        }
    }

    return kStatus_Success;
}
             
/*FUNCTION****************************************************************
 *
 * Function Name: MMC_ReadBlocks
 * Description: Reads blocks from card with default block size from SD or MMC card
 *
 *END*********************************************************************/
status_t MMC_ReadBlocks(mmc_card_t *card,  uint8_t *buffer, uint32_t startBlock, uint32_t blockCount)
{
    uint32_t blkCnt, blkLeft, blkDone;
    status_t err = kStatus_Success;
    sdhc_host_t *host;
    assert(card);
    assert(card->host);
    assert(buffer);
    assert(blockCount);

    host = card->host;
    blkLeft = blockCount;
    blkDone = 0;

    if ( kStatus_Success != MMC_CheckIORange(card, startBlock, blockCount))
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
        err = MMC_Read(card, buffer, startBlock, FSL_CARD_DEFAULT_BLOCK_SIZE, blockCount);

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
 * Function Name: MMC_WriteBlocks
 * Description: Writes blocks to card with default block size to SD/MMC card
 *
 *END*********************************************************************/
status_t MMC_WriteBlocks(mmc_card_t *card, uint8_t *buffer, uint32_t startBlock, uint32_t blockCount)
{
    uint32_t blkCnt, blkLeft, blkDone;
    status_t err = kStatus_Success;
    sdhc_host_t *host;
    assert(card);
    assert(card->host);
    assert(buffer);
    assert(blockCount);

    host = card->host;
    blkLeft = blockCount;
    blkDone = 0;

    if ( kStatus_Success != MMC_CheckIORange(card, startBlock, blockCount))
    {
        return kStatus_InvalidArgument;
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
        
        err = MMC_Write(card, buffer, startBlock, FSL_CARD_DEFAULT_BLOCK_SIZE, blockCount);
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
 * Function Name: MMC_EraseBlocks
 * Description: Erases block range from SD/MMC card with default block size
 *
 *END*********************************************************************/
status_t MMC_EraseBlocks(mmc_card_t *card, uint32_t startBlock, uint32_t blockCount)
{
    uint32_t eraseGroupDone = 0, eraseGroupStart, eraseGroupLeft, eraseGroupCount;
    status_t err = kStatus_Success;
    assert(card);
    assert(blockCount);

    /* startBlock must be group address boundry */
    if ((!(startBlock + 1) % card->eraseGroupSize) || !(blockCount % card->eraseGroupSize))
    {
        return kStatus_InvalidArgument;
    }
    
    if ( kStatus_Success != MMC_CheckIORange(card, startBlock, blockCount))
    {
        return kStatus_InvalidArgument;
    }
    
    eraseGroupStart = ((startBlock + 1)/card->eraseGroupSize - 1);
    eraseGroupCount = blockCount/card->eraseGroupSize;
    eraseGroupLeft = eraseGroupCount;

    while(eraseGroupLeft)
    {
        /* Max groups can be erase one time */
        if (eraseGroupLeft > card->writeProtectGroupSize)
        {
            eraseGroupCount = card->writeProtectGroupSize;
            eraseGroupLeft = eraseGroupLeft - eraseGroupCount;
        }
        else
        {
            eraseGroupCount = eraseGroupLeft;
            eraseGroupLeft = 0;
        }

        err = MMC_Erase(card, (eraseGroupStart + eraseGroupDone), eraseGroupCount);
        if (kStatus_Success != err)
        {
            return kStatus_Fail;
        }

        if (kStatus_Success != MMC_SendStatus(card))
        {
            return kStatus_SDMMC_SendStatusFailed;
        }
        eraseGroupDone += eraseGroupCount;
    }
    return kStatus_Success;
}

