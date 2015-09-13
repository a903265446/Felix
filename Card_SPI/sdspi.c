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
#include <string.h>

#include "sdspi.h"


/*FUNCTION****************************************************************
 *
 * Function Name: SDSPI_WaitReady
 * Description: Waits ready 
 *
 *END*********************************************************************/
#if 1
static status_t SDSPI_WaitReady(sdspi_host_t *host)
{
    uint8_t response;
    uint8_t timingByte = 0xFF; /* The byte need to be sent as read/write data timing requirement*/
    //uint8_t timingByte[2];
    //timingByte[0] = 0xFF;
      
    uint32_t startTime, currentTime, elapsedTime;

    startTime = host->getCurrentTimeMsec();
    do
    {
        if (kStatus_Success != host->exchange(host->base, &timingByte, &response, 1))
        {
            return kStatus_SDSPI_TransferFailed;
        }
        currentTime = host->getCurrentTimeMsec();
        if (currentTime < startTime)
        {
            currentTime += host->timeRangeMsec;
        }
        elapsedTime = (currentTime - startTime);
    } while ((response != 0xFF) && elapsedTime < 500);

    if (response != 0xFF)
    {
        return kStatus_SDSPI_CardIsBusyError;
    }

    return kStatus_Success;
}
#endif

#if 0
static status_t SDSPI_WaitReady(sdspi_host_t *host)
{
    uint8_t response;
    uint32_t startTime, elapsedTime;

    startTime = OSA_TimeGetMsec();
    do
    {
        response = spiSendWord(host, 0xFF);
        elapsedTime = OSA_TimeGetMsec() - startTime;
    } while ((response != 0xFF) && elapsedTime < 500);

    if (response != 0xFF)
    {
        return kStatus_SDSPI_CardIsBusyError;
    }

    return kStatus_Success;
}
#endif



/*FUNCTION****************************************************************
 *
 * Function Name: SDSPI_GenerateCRC7
 * Description: Calculates CRC7
 *
 *END*********************************************************************/
static uint32_t SDSPI_GenerateCRC7(uint8_t *buffer, uint32_t length, uint32_t crc)
{
    uint32_t index;

    static const uint8_t crcTable[] = {
        0x00, 0x09, 0x12, 0x1B, 0x24, 0x2D, 0x36, 0x3F,
        0x48, 0x41, 0x5A, 0x53, 0x6C, 0x65, 0x7E, 0x77
    };

    while (length)
    {
        index = ((crc >> 3) & 0x0F) ^ ((*buffer) >> 4);
        crc = (crc << 4) ^ crcTable[index];

        index = ((crc >> 3) & 0x0F) ^ ((*buffer) & 0x0F);
        crc = (crc << 4) ^ crcTable[index];

        buffer++;
        length--;
    }

    return (crc & 0x7F);
}

static status_t SDSPI_SendCommand(sdspi_host_t *host, sdspi_cmd_t *cmd, uint32_t timeout)
{
    uint8_t buffer[6], response, i;
    uint8_t timingByte = 0xFF; /* The byte need to be sent as read/write data timing requirement*/
    status_t ret = kStatus_Success;

    ret = SDSPI_WaitReady(host);
    if (ret != kStatus_Success)
    {
        return ret;
    }

    buffer[0] = (cmd->cmdIndex | 0x40);
    buffer[1] = ((cmd->argument >> 24U) & 0xFF);
    buffer[2] = ((cmd->argument >> 16) & 0xFF);
    buffer[3] = ((cmd->argument >> 8) & 0xFF);
    buffer[4] = (cmd->argument & 0xFF);
    buffer[5] = (SDSPI_GenerateCRC7(buffer, 5, 0) << 1 | 1);

   /*     buffer[0] = SDSPI_MAKE_CMD(req->cmdIndex);
    buffer[1] = req->argument >> 24 & 0xFF;
    buffer[2] = req->argument >> 16 & 0xFF;
    buffer[3] = req->argument >> 8 & 0xFF;
    buffer[4] = req->argument & 0xFF;
    buffer[5] = (SDSPI_DRV_GenerateCRC7(buffer, 5, 0) << 1) | 1;*/
    
    
    if (host->exchange(host->base, buffer, NULL, sizeof(buffer)))
    {
        return kStatus_SDSPI_TransferFailed;
    }

    /*
    if (cmd->cmdIndex == kSDMMCStopTransmission)
    {
        host->sendByte(host->base, 0xFF);
    }
     */
    
    /* Wait for the response coming, the left most bit which is transfered first in first response byte is 0 */
    for (i = 0; i < 9; i++)
    {
        if (kStatus_Success != host->exchange(host->base, &timingByte, &response, 1))
        {
            return kStatus_SDSPI_TransferFailed;
        }
        //response = spiSendWord(host, 0xFF);

        /* Check if response 0 coming. */
        if (!(response & 0x80))
        {
            break;
        }
    }
    if (response & 0x80)
    {
        return kStatus_Fail;
    }

    cmd->response[0] = response;

    switch (cmd->respType)
    {
        case kSDSPIRespTypeR1:
            break;
        case kSDSPIRespTypeR1b:
        {   
            uint8_t busy = 0;
            uint32_t startTime, currentTime, elapsedTime;
            startTime = host->getCurrentTimeMsec();
            while (busy != 0xFF)
            {
                if (kStatus_Success != host->exchange(host->base, &timingByte, &response, 1))
                {
                    return kStatus_SDSPI_TransferFailed;
                }

                currentTime = host->getCurrentTimeMsec();
                if (currentTime < startTime)
                {
                    currentTime += host->timeRangeMsec;
                }
                elapsedTime = currentTime - startTime;

                if (elapsedTime > timeout)
                {
                    break;
                }
            }

            if (busy != 0xFF)
            {
                return kStatus_SDSPI_CardIsBusyError;
            }
        }
            break;
        case kSDSPIRespTypeR2:
            if (kStatus_Success != host->exchange(host->base, &timingByte, &(cmd->response[1]), 1))
            {
                return kStatus_SDSPI_TransferFailed;
            }
            break;
        case kSDSPIRespTypeR3:
        case kSDSPIRespTypeR7:
            /* Left 4 bytes in response type R3 and R7*/
            if (kStatus_Success != host->exchange(host->base, &timingByte, &(cmd->response[1]), 4))
            {
                return kStatus_SDSPI_TransferFailed;
            }
            break;
        default:
            return kStatus_Fail;
    }
    return kStatus_Success;
}

static status_t SDSPI_GoIdle(sdspi_card_t *card)
{
    uint8_t i, j;
    uint8_t timingByte = 0xFF; /* The byte need to be sent as read/write data timing requirement*/
    sdspi_host_t *host;
    sdspi_cmd_t cmd = {0};
    uint8_t response;

    assert(card);
    assert(card->host);

    host = card->host;
    
     /*
     * SD card will enter SPI mode if the CS is asserted (negative) during the
     * reception of the reset command (CMD0) and the card is in IDLE state.
     */
    for (i = 0; i < 2; i++)
    {
        for (j = 0; j < 10; j++)
        {
            if (host->exchange(host->base, &timingByte, &response, 1))
            //if (host->exchange(host->base, &timingByte, NULL, 1))
            
            {
                return kStatus_Success;
            }
            //spiSendWord(host, 0xFF);
        }

        cmd.cmdIndex = kSDMMCGoIdleState;
        cmd.respType = kSDSPIRespTypeR1;
        if (kStatus_Success != SDSPI_SendCommand(host, &cmd, FSL_SDSPI_TIMEOUT))
        {
            return kStatus_Fail;
        }

        if (cmd.response[0] == SDMMC_SPI_R1_IN_IDLE_STATE)
        {
            break;
        }
    }

    if (cmd.response[0] != SDMMC_SPI_R1_IN_IDLE_STATE)
    {
        return kStatus_Fail;
    }

    return kStatus_Success;
}

/*FUNCTION****************************************************************
 *
 * Function Name: SDSPI_SendIfCond
 * Description: Checks card interface condition, which includes host supply
 * voltage information and asks the card whether card supports voltage.
 *
 *END*********************************************************************/
static status_t SDSPI_SendIfCond(sdspi_card_t *card, uint8_t pattern, uint8_t *response)
{
    sdspi_cmd_t cmd = {0};
    sdspi_host_t *host;

    assert(card);
    assert(card->host);

    host = card->host;

    cmd.cmdIndex = kSDSendIfCond;
    cmd.argument = 0x100 | (pattern & 0xFF);
    cmd.respType = kSDSPIRespTypeR7;
    if (kStatus_Success != SDSPI_SendCommand(host, &cmd, FSL_SDSPI_TIMEOUT))
    {
        return kStatus_Fail;
    }

    memcpy(response, cmd.response, sizeof(cmd.response));

    return kStatus_Success;
}

static status_t SDSPI_SendApplicationCmd(sdspi_card_t *card)
{
    sdspi_host_t *host;
    sdspi_cmd_t cmd = {0};
    assert(card);
    assert(card->host);

    host = card->host;

    cmd.cmdIndex = kSDMMCAppCmd;
    cmd.respType = kSDSPIRespTypeR1;
    if (kStatus_Success != SDSPI_SendCommand(host, &cmd, FSL_SDSPI_TIMEOUT))
    {
        return kStatus_Fail;
    }

    if (cmd.response[0] && !(cmd.response[0] & SDMMC_SPI_R1_IN_IDLE_STATE))
    {
        return kStatus_Fail;
    }

    return kStatus_Success;
}

/*FUNCTION****************************************************************
 *
 * Function Name: SDSPI_AppSendOpCond
 * Description: Gets the card to send its operating condition.
 *
 *END*********************************************************************/
static status_t SDSPI_AppSendOpCond(sdspi_card_t *card, uint32_t argument, uint8_t *response)
{
    sdspi_cmd_t cmd = {0};
    uint32_t startTime, currentTime, elapsedTime = 0;
    sdspi_host_t *host;

    assert(card);
    assert(card->host);
    assert(response);

    host = card->host;

    cmd.cmdIndex = kSDAppSendOpCond;
    cmd.argument = argument;
    cmd.respType = kSDSPIRespTypeR1;

    startTime = host->getCurrentTimeMsec();
    do
    {
        if (kStatus_Success == SDSPI_SendApplicationCmd(card))
        {
            if (kStatus_Success == SDSPI_SendCommand(host, &cmd, FSL_SDSPI_TIMEOUT))
            {
                if (!cmd.response[0])
                {
                    break;
                }
            }
        }

        currentTime = host->getCurrentTimeMsec();
        if (currentTime < startTime)
        {
            currentTime += host->timeRangeMsec;
        }
        elapsedTime = currentTime - startTime;
    } while (elapsedTime < 1000);

    if (response)
    {
        memcpy(response, cmd.response, sizeof(cmd.response));
    }

    if (elapsedTime < 1000)
    {
        return kStatus_Success;
    }
    return kStatus_SDSPI_TimeoutError;
}

static status_t SDSPI_ReadOcr(sdspi_card_t *card)
{
    uint32_t i;
    sdspi_host_t *host;
    sdspi_cmd_t cmd = {0};

    assert(card);
    assert(card->host);

    host = card->host;

    cmd.cmdIndex = kSDMMCReadOcr;
    cmd.respType = kSDSPIRespTypeR3;
    if (kStatus_Success != SDSPI_SendCommand(host, &cmd, FSL_SDSPI_TIMEOUT))
    {
        return kStatus_Fail;
    }
    if (cmd.response[0])
    {
        return kStatus_Fail;
    }

    card->ocr = 0;
    for (i = 4; i > 0; i--)
    {
        card->ocr |= (uint32_t) cmd.response[i] << ((4 - i) * 8);
    }

    return kStatus_Success;
}

/*FUNCTION****************************************************************
 *
 * Function Name: SDSPI_SetBlockSize
 * Description:  Sets the block length in bytes for SDSC cards. For SDHC cards,
 * it does not affect memory read or write commands, always 512 bytes fixed
 * block length is used.
 *
 *END*********************************************************************/
static status_t SDSPI_SetBlockSize(sdspi_card_t *card, uint32_t blockSize)
{
    sdspi_cmd_t cmd = {0};
    sdspi_host_t *host;

    assert(card);
    assert(card->host);

    host = card->host;

    cmd.cmdIndex = kSDMMCSetBlockLen;
    cmd.argument = blockSize;
    cmd.respType = kSDSPIRespTypeR1;

    if (kStatus_Success != SDSPI_SendCommand(host, &cmd, FSL_SDSPI_TIMEOUT))
    {
        return kStatus_Fail;
    }

    return kStatus_Success;
}

/*FUNCTION****************************************************************
 *
 * Function Name: SDSPI_Read
 * Description: Reads data from card
 *
 *END*********************************************************************/
static status_t SDSPI_Read(sdspi_host_t *host, uint8_t *buffer, uint32_t size)
{
    uint32_t startTime, currentTime, elapsedTime;
    uint8_t response, i;
    uint8_t timingByte = 0xFF; /* The byte need to be sent as read/write data timing requirement*/
    assert(host);
    assert(host->exchange);
    assert(buffer);
    assert(size);

    memset(buffer, 0xFF, size);

    /* Wait data block comming */
    startTime = host->getCurrentTimeMsec();
    do
    {
        if (kStatus_Success != host->exchange(host->base, &timingByte, &response, 1))
        {
            return kStatus_SDSPI_TransferFailed;
        }
        currentTime = host->getCurrentTimeMsec();
        if (currentTime < startTime)
        {
            currentTime += host->timeRangeMsec;
        }
        elapsedTime = currentTime - startTime;
    } while ((response == 0xFF) && elapsedTime < 100);

    if (response != SDMMC_SPI_DT_START_SINGLE_BLK)
    {
        return kStatus_Fail;
    }

    if (host->exchange(host->base, buffer, buffer, size))
    {
        return kStatus_SDSPI_TransferFailed;
    }

    for (i = 0; i < 8; i++)
    {
        if (kStatus_Success != host->exchange(host->base, &timingByte, &response, 1))
        {
            return kStatus_SDSPI_TransferFailed;
        }
    }
    /*spi->ops->sendWord(spi, 0xFF);
    spi->ops->sendWord(spi, 0xFF);*/

    return kStatus_Success;
}

/*FUNCTION****************************************************************
 *
 * Function Name: SD_DecodeCsd
 * Description: Decodes csd register
 *
 *END*********************************************************************/
static void SDSPI_DecodeCsd(sdspi_card_t *card, uint8_t *rawCsd)
{
    sd_csd_t *csd;
    assert(rawCsd);
    assert(card);

    csd = &(card->csd);  
    csd->csdStructure = (rawCsd[0] >> 6);
    csd->taac = rawCsd[1];
    csd->nsac = rawCsd[2];
    csd->tranSpeed = rawCsd[3];
    csd->ccc = (((uint16_t)(rawCsd)[4] << 4) | ((uint16_t)(rawCsd)[5]) >> 4);
    csd->readBlkLen = ((rawCsd)[5] & 0xF);
    if (rawCsd[6] & 0x80)
    {
        csd->flags |= SD_CSD_READ_BL_PARTIAL;
    }
    if (rawCsd[6] & 0x40)
    {
        csd->flags |= SD_CSD_WRITE_BLOCK_MISALIGN;
    }
    if (rawCsd[6] & 0x20)
    {
        csd->flags |= SD_CSD_READ_BLOCK_MISALIGN;
    }
    if (rawCsd[6] & 0x10)
    {
        csd->flags |= SD_CSD_DSR_IMP;
    }
    if (csd->csdStructure == 0)
    {
        csd->cSize = ((((rawCsd)[6] & 0x3) << 10) | ((rawCsd)[7] << 2) | (((rawCsd)[8] >> 6)));
        csd->vddRCurrMin = (((rawCsd)[8] >> 3) & 7);
        csd->vddRCurrMax = ((rawCsd)[8] >> 7);
        csd->vddWCurrMin = (((rawCsd)[9] >> 5) & 7);
        csd->vddWCurrMax = ((rawCsd)[9] >> 2);
        csd->cSizeMult = ((((rawCsd)[9] & 3) << 1) | ((rawCsd)[10] >> 7));
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
        csd->cSize = (((uint32_t)(rawCsd)[7] & 0x3f) << 16 | ((rawCsd)[8] << 8) | (rawCsd)[9]);
        if (csd->cSize >= 0xFFFF)
        {
            card->caps |= SDSPI_CAPS_SDXC;
        }
        card->blockCount = (csd->cSize + 1) * 1024;
    }

    if (((rawCsd)[10] >> 6) & 1)
    {
        csd->flags |= SD_CSD_ERASE_BLOCK_ENABLED;
    }

    csd->sectorSize = ((((rawCsd)[10] & 0x3F) << 1) | ((rawCsd)[11] >> 7));
    csd->wpGrpSize = ((rawCsd)[11] & 0x7F);
    if ((rawCsd)[12] >> 7)
    {
        csd->flags |= SD_CSD_WP_GRP_ENABLED;
    }
    csd->r2wFactor = (((rawCsd)[12] >> 2) & 7);
    csd->writeBlkLen = ((((rawCsd)[12] & 3) << 2) | ((rawCsd)[13] >> 6));
    if (((rawCsd)[13] >> 5) & 1)
    {
        csd->flags |= SD_CSD_WRITE_BL_PARTIAL;
    }
    if ((rawCsd)[14] >> 7)
    {
        csd->flags |= SD_CSD_FILE_FORMAT_GROUP;
    }
    if (((rawCsd)[14] >> 16) & 1)
    {
        csd->flags |= SD_CSD_COPY;
    }
    if (((rawCsd)[14] >> 5) & 1)
    {
        csd->flags |= SD_CSD_PERM_WRITE_PROTECT;
    }
    if (((rawCsd)[14] >> 4) & 1)
    {
        csd->flags |= SD_CSD_TMP_WRITE_PROTECT;
    }
    csd->fileFormat = (((rawCsd)[14] >> 2) & 3);
}


/*FUNCTION****************************************************************
 *
 * Function Name: SDSPI_SendCsd
 * Description: Gets CSD register from card
 *
 *END*********************************************************************/
static status_t SDSPI_SendCsd(sdspi_card_t *card)
{
    sdspi_cmd_t cmd = {0};
    sdspi_host_t *host;

    assert(card);
    assert(card->host);

    host = card->host;

    cmd.cmdIndex = kSDMMCSendCsd;
    cmd.respType = kSDSPIRespTypeR1;
    if (kStatus_Success != SDSPI_SendCommand(host, &cmd, FSL_SDSPI_TIMEOUT))
    {
        return kStatus_Fail;
    }

    if (kStatus_Success != (SDSPI_Read(host, (uint8_t *)card->rawCsd, sizeof(card->rawCsd))))
    {
        return kStatus_Fail;
    }

    SDSPI_DecodeCsd(card, card->rawCsd);

    /* No start single block token if found */
    return kStatus_Success;
}

static void SDSPI_SetToMaxFrequencyInNormalMode(sdspi_card_t *card)
{
    uint32_t maxFrequency;
    
    /* rate unit is divided by 1000 */
    static const uint32_t transpeedru[] =
    {
      /* 100Kbps, 1Mbps, 10Mbps, 100Mbps*/
        100, 1000, 10000, 100000,
    };

    /* time value multiplied by 1000 */
    static const uint32_t transpeedtv[] =
    {
           0, 1000, 1200, 1300,
        1500, 2000, 2500, 3000,
        3500, 4000, 4500, 5000,
        5500, 6000, 7000, 8000,
    };

    /* Calculate frequency */
    maxFrequency = transpeedru[RD_SD_CSD_TRAN_SPEED_TRANSFER_RATE_UNIT(card->csd)] 
                 * transpeedtv[RD_SD_CSD_TRAN_SPEED_MULT_FACTOR(card->csd)];
    if (maxFrequency > card->host->busBaudRate)
    {
        maxFrequency = card->host->busBaudRate;
    }

    card->host->setFrequency(card->host->base, maxFrequency);
}

/*FUNCTION****************************************************************
 *
 * Function Name: SDSPI_CheckCapacity
 * Description: Checks card capacity of the card
 *
 *END*********************************************************************/
static void SDSPI_CheckCapacity(sdspi_card_t *card)
{
    uint32_t cSize, cSizeMult, readBlkLen;

    if (card->csd.csdStructure)
    {
        /* SD CSD structure v2.xx */
        cSize = card->csd.cSize;
        if (cSize >= 0xFFFFU)/* Bigger than 32GB */
        {
            /* extended capacity */
            card->caps |= SDSPI_CAPS_SDXC;
        }
        else
        {
            card->caps |= SDSPI_CAPS_SDHC;
        }
        cSizeMult = 10;
        cSize += 1;
        readBlkLen = 9;
        //card->blockSize = 512; /* 512 bytes */
        //card->blockCount = ((cSize + 1U) << 10U);
    }
    else
    {
        /* SD CSD structure v1.xx */
        cSize = (card->csd.cSize + 1);
        cSizeMult = (card->csd.cSizeMult + 2);
        readBlkLen = card->csd.readBlkLen;

        /* Card maximum capacity is 2GB when CSD structure version is 1.0 */
        card->caps |= SDSPI_CAPS_SDSC;
    }

    if (readBlkLen != 9)
    {
        /* Force to use 512-byte length block */
        cSizeMult += (readBlkLen - 9);
        readBlkLen = 9;
    }

    card->blockSize = 1 << readBlkLen;
    card->blockCount = cSize << cSizeMult;

}

/*FUNCTION****************************************************************
 *
 * Function Name: SDSPI_CheckReadOnly
 * Description: Checks if card is read only
 *
 *END*********************************************************************/
bool SDSPI_CheckReadOnly(sdspi_card_t *card)
{
    assert(card);

    card->state &= ~SDSPI_STATE_WRITE_PROTECTED;
    // if (card->cardType != kCardTypeSd)
    // {
    //     return false;
    // }

    if ((card->csd.flags & SD_CSD_PERM_WRITE_PROTECT) || (card->csd.flags & SD_CSD_TMP_WRITE_PROTECT))
    {
        card->state |= SDSPI_STATE_WRITE_PROTECTED;
        return true;
    }

    return false;
}

/*FUNCTION****************************************************************
 *
 * Function Name: SDSPI_SendCid
 * Description: Gets CID information from card
 *
 *END*********************************************************************/
static status_t SDSPI_SendCid(sdspi_card_t *card)
{
    sdspi_cmd_t cmd = {0};
    sdspi_host_t *host;

    assert(card);
    assert(card->host);

    host = card->host;

    cmd.cmdIndex = kSDMMCSendCid;
    cmd.respType = kSDSPIRespTypeR1;

    if (kStatus_Success != SDSPI_SendCommand(host, &cmd, FSL_SDSPI_TIMEOUT))
    {
        return kStatus_Fail;
    }


    if (kStatus_Success != (SDSPI_Read(host, card->rawCid, sizeof(card->rawCid))))
    {
        return kStatus_Fail;
    }

    return kStatus_Success;
}


/*FUNCTION****************************************************************
 *
 * Function Name: SDSPI_InitSd
 * Description: Initializes SD card
 *
 *END*********************************************************************/
static status_t SDSPI_InitSd(sdspi_card_t *card)
{
    uint32_t maxFrequency;
    assert(card);
    assert(card->host);
    assert(card->host->setFrequency);
    assert(card->host->exchange);

    if (kStatus_Success != SDSPI_SendCsd(card))
    {
        return kStatus_Fail;
    }

    SDSPI_SetToMaxFrequencyInNormalMode(card);

    SDSPI_CheckCapacity(card);
    SDSPI_CheckReadOnly(card);

    if (kStatus_Success != SDSPI_SendCid(card))
    {
        return kStatus_Fail;
    }

    return kStatus_Success;
}

status_t SDSPI_Init(sdspi_card_t *card)
{
    sdspi_host_t *host;
    uint32_t acmd41Arg = 0, startTime, currentTime, elapsedTime;
    uint8_t response[5], acmd41Resp[5];
    bool likelySdV1 = false, likelyMmc = false;

    assert(card);
    assert(card->host);
    assert(card->host->setFrequency);
    assert(card->host->exchange);
    assert(card->host->getCurrentTimeMsec);

    host = card->host;

    if (host->setFrequency(host->base, SDMMC_CLK_400KHZ))
    {
        return kStatus_Fail;
    }

    if (kStatus_Success != SDSPI_GoIdle(card))
    {
        return kStatus_Fail;
    }

    if (kStatus_Success != SDSPI_SendIfCond(card, 0xAA, response))
    {
        likelySdV1 = true;
    }
    else if ((response[3] == 0x1) || (response[4] == 0xAA))
    {
        acmd41Arg |= SD_OCR_HCS;
    }
    else
    {
        return kStatus_Fail;
    }

    startTime = host->getCurrentTimeMsec();
    do
    {
        if (kStatus_Success != SDSPI_AppSendOpCond(card, acmd41Arg, acmd41Resp))
        {
            if (likelySdV1)
            {
                likelyMmc = true;
            }
            return kStatus_Fail;
        }

        currentTime = host->getCurrentTimeMsec();
        if (currentTime < startTime)
        {
            currentTime += host->timeRangeMsec;
        }
        elapsedTime = currentTime - startTime;
        if (elapsedTime > 500)
        {
            if (likelySdV1)
            {
                likelyMmc = true; 
            }
            return kStatus_Fail;
        }

        if (!acmd41Resp[0])
        {
            break;
        }
        
    } while(acmd41Resp[0] & SDMMC_SPI_R1_IN_IDLE_STATE);

    if (likelyMmc)
    {
        return kStatus_SDSPI_NotSupportYet;
    }

    if (!likelySdV1)
    {
        card->version = kSDVersion_2_x;
        if (kStatus_Success != SDSPI_ReadOcr(card))
        {
            return kStatus_Fail;
        }
        if (card->ocr & SD_OCR_CCS)
        {
            card->caps = SDSPI_CAPS_HIGH_CAPACITY;
        }
    }
    else
    {
        card->version = kSDVersion_1_x;
    }
    /* Force to use 512-byte length block, no matter which version  */
    if (kStatus_Success != SDSPI_SetBlockSize(card, 512))
    {
        return kStatus_Fail;
    }

    if (kStatus_Success != SDSPI_InitSd(card))
    {
        return kStatus_Fail;
    }
    return kStatus_Success;
}

void SDSPI_DeInit(sdspi_card_t *card)
{
    
}
/*FUNCTION****************************************************************
 *
 * Function Name: SDSPI_StopTransmission
 * Description:  Sends stop transmission command to card to stop ongoing
 * data transferring.
 *
 *END*********************************************************************/
static status_t SDSPI_StopTransmission(sdspi_card_t *card)
{
    sdspi_cmd_t cmd = {0};
    sdspi_host_t *host;

    host = card->host;

    cmd.cmdIndex = kSDMMCStopTransmission;
    cmd.respType = kSDSPIRespTypeR1b;
    if (kStatus_Success != SDSPI_SendCommand(host, &cmd, FSL_SDSPI_TIMEOUT))
    {
        return kStatus_Fail;
    }

    return kStatus_Success;
}

/*FUNCTION****************************************************************
 *
 * Function Name: SDSPI_ReadBlocks
 * Description: Reads blocks from card 
 *
 *END*********************************************************************/
status_t SDSPI_ReadBlocks(sdspi_card_t *card, uint8_t *buffer, uint32_t startBlock, 
                        uint32_t blockCount)
{
    uint32_t offset, i;
    sdspi_cmd_t cmd = {0};
    sdspi_host_t *host;

    assert(card);
    assert(card->host);
    assert(buffer);
    assert(blockCount);

    host = card->host;

    offset = startBlock;
    if (!IS_BLOCK_ACCESS(card))
    {
        offset *= card->blockSize;
    }
    
    cmd.argument = offset;
    cmd.respType = kSDSPIRespTypeR1;

    if (blockCount == 1)
    {
        cmd.cmdIndex = kSDMMCReadSingleBlock;

        if (kStatus_Success != SDSPI_SendCommand(host, &cmd, FSL_SDSPI_TIMEOUT))
        {
            return kStatus_Fail;
        }

        if (kStatus_Success != SDSPI_Read(host, buffer, card->blockSize))
        {
            return kStatus_Fail;
        }
    }
    else
    {
        cmd.cmdIndex = kSDMMCReadMultipleBlock;

        if (kStatus_Success != SDSPI_SendCommand(host, &cmd, FSL_SDSPI_TIMEOUT))
        {
            return kStatus_Fail;
        }

        for (i = 0; i < blockCount; i++)
        {
            if (kStatus_Success != SDSPI_Read(host, buffer, card->blockSize))
            {
                return kStatus_Fail;
            }
            buffer += card->blockSize;
        }
        SDSPI_StopTransmission(card);
    }

    return kStatus_Success;
}

/*FUNCTION****************************************************************
 *
 * Function Name: SDSPI_Write
 * Description: Writes data to card
 *
 *END*********************************************************************/
static status_t SDSPI_Write(sdspi_host_t *host, uint8_t *buffer, uint32_t size, uint8_t token)
{
    uint8_t response, i;
    uint8_t timingByte = 0xFF; /* The byte need to be sent as read/write data timing requirement*/
    assert(host);
    assert(host->exchange);
    
    if (SDSPI_WaitReady(host) != kStatus_Success)
    {
        return kStatus_Fail;
    }

    if (host->exchange(host->base, &token, NULL, 1))
    {
        return kStatus_Fail;
    }

    if (token == SDMMC_SPI_DT_STOP_TRANSFER)
    {
        return kStatus_Success;
    }

    assert(size);
    assert(buffer);

    if (kStatus_Success != host->exchange(host->base, buffer, NULL, size))
    {
        return kStatus_SDSPI_TransferFailed;
    }

    /* Send CRC */
    // spi->ops->sendWord(spi, 0xFF);
    // spi->ops->sendWord(spi, 0xFF);
    /* Get the last two bytes CRC */
    for (i = 0; i < 2; i++)
    {
        if (host->exchange(host->base, &timingByte, NULL, 1))
        {
            return kStatus_SDSPI_TransferFailed;
        }
    }

    // response = spi->ops->sendWord(spi, 0xFF);
    if (host->exchange(host->base, &timingByte, &response, 1))
    {
        return kStatus_SDSPI_TransferFailed;
    }

    if ((response & SDMMC_SPI_DR_MASK) != SDMMC_SPI_DR_ACCEPTED)
    {
        return kStatus_Fail;
    }
    return kStatus_Success;
}

/*FUNCTION****************************************************************
 *
 * Function Name: SDSPI_WriteBlocks
 * Description: Writes blocks to card 
 *
 *END*********************************************************************/
status_t SDSPI_WriteBlocks(sdspi_card_t *card, uint8_t *buffer, uint32_t startBlock, 
                         uint32_t blockCount)
{
    uint32_t offset, i, startTime, currentTime, elapsedTime;
    uint8_t response;
    uint8_t timingByte = 0xFF; /* The byte need to be sent as read/write data timing requirement*/
    sdspi_host_t *host;
    sdspi_cmd_t cmd = {0};

    assert(card);
    assert(card->host);
    assert(buffer);
    assert(blockCount);

    host = card->host;

    if (card->state & SDSPI_STATE_WRITE_PROTECTED)
    {
        return kStatus_SDSPI_WriteProtected;
    }

    offset = startBlock;
    if (!IS_BLOCK_ACCESS(card))
    {
        offset *= card->blockSize;
    }

    if (blockCount == 1)
    {
        cmd.cmdIndex = kSDMMCWriteBlock;
        cmd.argument = offset;
        cmd.respType = kSDSPIRespTypeR1;

        if (kStatus_Success != SDSPI_SendCommand(host, &cmd, FSL_SDSPI_TIMEOUT))
        {
            return kStatus_Fail;
        }
        if (cmd.response[0])
        {
            return kStatus_Fail;
        }

        if (kStatus_Success != SDSPI_Write(host, buffer, card->blockSize, SDMMC_SPI_DT_START_SINGLE_BLK))
        {
            return kStatus_Fail;
        }
    }
    else
    {
#if defined FSL_SDSPI_ENABLE_PRE_ERASE_ON_WRITE
        // if (IS_SD_CARD(card))
        // {
            /* Pre-erase before writing data */
            if (kStatus_Success != SDSPI_SendApplicationCmd(card))
            {
                return kStatus_Fail;
            }
            cmd.cmdIndex = kSDAppSetWrBlkEraseCount;
            cmd.argument = blockCount;
            cmd.respType = kSDSPIRespTypeR1;
            if (kStatus_Success != SDSPI_SendCommand(host->base, &cmd, FSL_SDSPI_TIMEOUT))
            {
                return kStatus_Fail;
            }
            if (req->response[0])
            {
                return kStatus_Fail;
            }
        // }
#endif

        memset(&cmd, 0, sizeof(sdspi_cmd_t));
        cmd.cmdIndex = kSDMMCWriteMultipleBlock;
        cmd.argument = offset;
        cmd.respType = kSDSPIRespTypeR1;

        if (kStatus_Success != SDSPI_SendCommand(host, &cmd, FSL_SDSPI_TIMEOUT))
        {
            return kStatus_Fail;
        }
        if (cmd.response[0])
        {
            return kStatus_Fail;
        }

        for (i = 0; i < blockCount; i++)
        {
            if (kStatus_Success != SDSPI_Write(host, buffer, card->blockSize, SDMMC_SPI_DT_START_MULTI_BLK))
            {
                return kStatus_Fail;
            }
            buffer += card->blockSize;
        }

        SDSPI_Write(host, 0, 0, SDMMC_SPI_DT_STOP_TRANSFER);

        startTime = host->getCurrentTimeMsec();
        do
        {
            // response = spi->ops->sendWord(spi, 0xFF);
            if (host->exchange(host->base, &timingByte, &response, 1))
            {
                return kStatus_SDSPI_TransferFailed;
            }
            currentTime = host->getCurrentTimeMsec();
            if (currentTime < startTime)
            {
                currentTime += host->timeRangeMsec;
            }
            elapsedTime = currentTime - startTime;
        } while ((response != 0xFF) && (elapsedTime < 100));
    }

    return kStatus_Success;
}

/*************************************************************************************************
 * EOF
 ************************************************************************************************/