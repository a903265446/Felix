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

void SDMMC_DelayTimeMsec(uint32_t msec)
{
    uint32_t startTime, currentTime, elapsedTime;
    assert(msec);

    startTime = host->getCurrentTimeMsec();
    do
    {
        currentTime = host->getCurrentTimeMsec();
        if (currentTime < startTime)
        {
            currentTime += host->timeRangeMsec;
        }
        elapsedTime = (currentTime - startTime);
    }    
    while (elapsedTime < msec);
}

/*FUNCTION****************************************************************
 *
 * Function Name: SDMMC_SelectCard
 * Description: Selects or deselects card
 *
 *END*********************************************************************/
status_t SDMMC_SelectCard(sdhc_host_t *host, uint32_t rca, bool isSelected)
{
    sdhc_cmd_t command = {0};
    assert(host);

    command.index = kSDMMCSelectCard;
    if (isSelected)
    {
        command.argument = rca << 16;
        command.responseType = kSDHC_ReponseTypeR1;
    }
    else
    {
        command.argument = 0;
        command.responseType = kSDHC_ReponseTypeNone;
    }

    host->currentCmd = &command;
    host->currentData = NULL;

    if ((kStatus_Success != SDHC_SendCmdBlocking(host, FSL_CARD_COMMAND_TIMEOUT)) || (!SDMMC_R1_ERROR_BITS(command->response[0])))
    {
        return kStatus_SDMMC_SendCommandFailed;
    }

    /* Wait until card to transfer state */
    return kStatus_Success;
}

status_t SDMMC_SendStatus(sdhc_host_t *host, uint32_t rca)
{
    sdhc_cmd_t command = {0};
    uint32_t timeout = 1000;
    assert(host);

    command = &commandMemory;
    command.index = kSDMMCSendStatus;
    command.argument = rca << 16;
    command.responseType = kSDHC_ReponseTypeR1;

    do
    {
        host->currentCmd = &command;
        host->currentData = 0;
        if ((kStatus_Success != SDHC_SendCmdBlocking(host, FSL_CARD_COMMAND_TIMEOUT)) || (!SDMMC_R1_ERROR_BITS(command->response[0])))
        {
            return kStatus_SDMMC_SendCommandFailed;
        }

        if ((command->response[0] & SDMMC_R1_READY_FOR_DATA) && (SDMMC_R1_CURRENT_STATE(command->response[0]) != SDMMC_R1_STATE_PRG))
        {
            break;
        }

        SDMMC_DelayTimeMsec(1);
    } while(timeout--);

    if (!timeout)
    {
        return kStatus_Timeout;
    }

    return kStatus_Success;
}

status_t SDMMC_SendApplicationCmd(sdhc_host_t *host, uint32_t rca)
{
    sdhc_cmd_t command = {0};
    assert(host);

    command.index = kSDMMCAppCmd;
    command.argument = 0;
    command.argument = rca << 16;
    command.responseType = kSDHC_ReponseTypeR1;

    host->currentCmd = &command;
    host->currentData = 0;

    if ((kStatus_Success != SDHC_SendCmdBlocking(host, FSL_CARD_COMMAND_TIMEOUT)) || (!SDMMC_R1_ERROR_BITS(command->response[0])))
    {
        if (command->errors & CARD_CMD_ERR_CMD_TIMEOUT)
        {
            return kStatus_Timeout;
        }
        else
        {
            return kStatus_SDMMC_SendCommandFailed;
        }
    }

    if (!(command->response[0] & SDMMC_R1_APP_CMD))
    {
        return kStatus_SDMMC_CardNotSupport;
    }

    return kStatus_Success;
}

status_t SDMMC_SetBlockCount(sdhc_host_t *host, uint32_t blockCount)
{
    sdhc_cmd_t command = {0};
    assert(host);
    
    command.index = kSDMMCSetBlockCount;
    command.argument = blockCount;
    command.responseType = kSDHC_ReponseTypeR1;

    if ((kStatus_Success != SDHC_SendCmdBlocking(host, FSL_CARD_COMMAND_TIMEOUT)) || (!SDMMC_R1_ERROR_BITS(command->response[0])))  
    {         
        return kStatus_SDMMC_SendCommandFailed;
    }

    return kStatus_Success;   
}

status_t SDMMC_GoIdle(sdhc_host_t *host)
{
    sdhc_cmd_t command = {0};
    assert(host);

    command.index = kSDMMCGoIdleState;

    host->currentCmd = &command;
    host->currentData = 0;

    if (kStatus_Success != SDHC_SendCmdBlocking(host, FSL_CARD_COMMAND_TIMEOUT))
    {
        return kStatus_Fail;
    }

    return kStatus_Success;
}

status_t SDMMC_StopTransmission(sdhc_host_t *host)
{
    assert(host);
    sdhc_cmd_t command = {0};

    command.index = kSDMMCStopTransmission;
    command.isAbortCommand = true;
    command.argument = 0;
    command.responseType = kSDHC_ReponseTypeR1b;
    
    host->currentCmd = &command;
    host->currentData = 0;

    if ((kStatus_Success != SDHC_SendCmdBlocking(host, FSL_CARD_COMMAND_TIMEOUT)) || (!SDMMC_R1_ERROR_BITS(command->response[0])))
    {
        return kStatus_Fail;
    }

    return kStatus_Success;
}

status_t SDMMC_SetBlockSize(sdhc_host_t *host, uint32_t blockSize)
{
    sdhc_cmd_t command = {0};
    assert(host);

    command.index = kSDMMCSetBlockLen;
    command.argument = blockSize;
    command.responseType = kSDHC_ReponseTypeR1;

    host->currentCmd = &command;
    host->currentData = 0;

    if ((kStatus_Success != SDHC_SendCmdBlocking(host, FSL_CARD_COMMAND_TIMEOUT)) || (!SDMMC_R1_ERROR_BITS(command->response[0])))
    {
        return kStatus_Fail;
    }

    return kStatus_Success;
}

status_t SDMMC_SetSdBusFreq(sdhc_host_t *host, uint32_t busFreq)
{
    sdhc_sd_clock_config_t sdClockConfig = {0};
    assert(host);
    if (!busFreq)
    {
        return kStatus_InvalidArgument;
    }

    sdClockConfig.enableSdClock = true;
    sdClockConfig.baseClockFreq = host->capability->sourceClockFreq;
    sdClockConfig.sdClockFreq = SD_CLK_50MHZ;    
    SDHC_SetSdClockConfig(host->base, &sdClockConfig);

    return kStatus_Success;
}