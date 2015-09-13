/*
 * Copyright (c) 2014, Freescale Semiconductor, Inc.
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
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "sdhc.h"
#include "card.h"
#include "sd_disk.h"
#include "diskio.h"
#include "test_sdhc_callback.h"

//#define SDHC_INSTANCE_ID 0
static sd_card_t g_sdhcCard;
// static mmc_card_t g_mmcCard;
static sdhc_host_t g_sdhcHost;

#if _USE_WRITE
DRESULT sd_disk_write(uint8_t pdrv, const uint8_t *buff, uint32_t sector, uint8_t count)
{
    if (pdrv != SD)
    {
        return RES_PARERR;
    }
    // if (g_sdhcCard.cardType == 0 || g_sdhcCard.cardType == kCardTypeUnknown)
    // {
    //     return RES_NOTRDY;
    // }
    if (kStatus_Success != SD_WriteBlocks(&g_sdhcCard, (uint8_t *)buff, sector, count))
    {
        return RES_ERROR;
    }
    return RES_OK;
}
#endif

DRESULT sd_disk_read(uint8_t pdrv, uint8_t *buff, uint32_t sector, uint8_t count)
{
    if (pdrv != SD)
    {
        return RES_PARERR;
    }

    // if (g_sdhcCard.cardType == 0 || g_sdhcCard.cardType == kCardTypeUnknown)
    // {
    //     return RES_NOTRDY;
    // }
    if (kStatus_Success != SD_ReadBlocks(&g_sdhcCard, buff, sector, count))
    {
        return RES_ERROR;
    }
    return RES_OK;
}

#if _USE_IOCTL
DRESULT sd_disk_ioctl(uint8_t pdrv, uint8_t cmd, void *buff)
{
    DRESULT res = RES_OK;

    if (pdrv != SD)
    {
        return RES_PARERR;
    }

    switch(cmd)
    {
        case GET_SECTOR_COUNT:
            if (buff)
            {
                *(uint32_t *)buff = g_sdhcCard.blockCount;
            }
            else
            {
                res = RES_PARERR;
            }
            break;
        case GET_SECTOR_SIZE:
            if (buff)
            {
                *(uint32_t *)buff = g_sdhcCard.blockSize;
            }
            else
            {
                res = RES_PARERR;
            }
            break;
        case GET_BLOCK_SIZE:
            if (buff)
            {
                *(uint32_t *)buff = g_sdhcCard.csd.sectorSize;
            }
            else
            {
                res = RES_PARERR;
            }
            break;
        case CTRL_SYNC:
            res = RES_OK;
            break;
        case SDMMC_GET_TYPE:
            if (buff)
            {
                /*switch (g_sdhcCard.cardType)
                {
                    case kCardTypeMmc:
                        *(uint32_t *)buff = CT_MMC;
                        break;
                    case kCardTypeSd:
                        *(uint32_t *)buff = CT_SD1;
                        if ((g_sdhcCard.caps & SDMMC_CARD_CAPS_SDHC) ||
                            (g_sdhcCard.caps & SDMMC_CARD_CAPS_SDXC))
                        {
                            *(uint32_t *)buff = CT_SD2 | CT_BLOCK;
                        }
                        break;
                    default:
                        res = RES_PARERR;
                        break;
                }*/
                *(uint32_t *)buff = CT_SD1;
                if ((g_sdhcCard.caps & SD_CAPS_SDHC) ||
                    (g_sdhcCard.caps & SD_CAPS_SDXC))
                {
                    *(uint32_t *)buff = CT_SD2 | CT_BLOCK;
                }
            }
            else
            {
                res = RES_PARERR;
            }
            break;
        case SDMMC_GET_CSD:
            if (buff)
            {
                memcpy(buff, g_sdhcCard.rawCsd, sizeof(g_sdhcCard.rawCsd));
            }
            else
            {
                res = RES_PARERR;
            }
            break;
        case SDMMC_GET_CID:
            if (buff)
            {
                memcpy(buff, g_sdhcCard.rawCid, sizeof(g_sdhcCard.rawCid));
            }
            else
            {
                res = RES_PARERR;
            }
            break;
        case SDMMC_GET_OCR:
            if (buff)
            {
                *(uint32_t *)buff = g_sdhcCard.ocr;
            }
            else
            {
                res = RES_PARERR;
            }
            break;
        default:
            res = RES_PARERR;
            break;

    }

    return res;
}
#endif

DSTATUS sd_disk_status(uint8_t pdrv)
{
    if (pdrv != SD)
    {
        return STA_NOINIT;
    }

    return 0;
}

static sdhc_capability_t g_sdhcCapability = {0}; 
#define ADMA_TABLE_MAX_ENTRIES 2
static uint32_t g_admaTable[ADMA_TABLE_MAX_ENTRIES] = {0};

DSTATUS sd_disk_initialize(uint8_t pdrv)
{
    sdhc_host_user_config_t config = {0};
#if ! defined CD_USING_GPIO
    uint32_t retries = 5;
    uint32_t cardPresented = 0;
    status_t status;
#endif

    if (pdrv != SD)
    {
        return STA_NOINIT;
    }

    // SDHC_DeInitHost(SDHC_INSTANCE_ID);
    memset(&g_sdhcHost, 0, sizeof(g_sdhcHost));
    memset(&g_sdhcCard, 0, sizeof(g_sdhcCard));

    config.base = SDHC;

    //config.clock = SDMMC_CLK_100KHZ;
#if defined CD_USING_GPIO
    config.cardDetectMode = kSDHCCardDetectGpio;
#elif defined CD_USING_DAT3
    config.cardDetectMode = kSDHCCardDetectDat3;
    config.cardDetectCallback = sdhc_card_detection;
#elif defined CD_USING_POLL_DAT3
    config.cardDetectMode = kSDHCCardDetectPollDat3;
#else
#error unknown card detect type
#endif

//#if defined SDHC_USING_PIO
    config.transferMode = kSDHCTransModePio;
//#elif defined SDHC_USING_ADMA1
//    config.transferMode = kSDHCTransModeAdma1;
//#else
//    config.transferMode = kSDHCTransModeAdma2;
//#endif

    config.capability = &g_sdhcCapability;

    config.admaTableAddress = g_admaTable;
    config.admaTableMaxEntries = ADMA_TABLE_MAX_ENTRIES;
    
    /* Need to be defined externally. */
    config.createCmdEvent = createCmdEvent;
    config.waitCmdEvent = waitCmdEvent;
    config.notifyCmdEvent = notifyCmdEvent;
    config.deleteCmdEvent = deleteCmdEvent;
    config.createDataEvent = createDataEvent;
    config.waitDataEvent = waitDataEvent;
    config.notifyDataEvent = notifyDataEvent;
    config.deleteDataEvent = deleteDataEvent;
    config.getCurrentTimeMsec = getCurrentTimeMsec;
    config.timeRangeMsec = getTimeRangeMsec();
    config.delayTimeMsec = delayTimeMsec;
    config.cardIntCallback = NULL;
#if defined CD_USING_DAT3
    config.cardInsertedCallback = detectCardInserted;
    config.cardRemovedCallback = detectCardRemoved;
#elif defined CD_USING_GPIO || defined CD_USING_POLL_DAT3
    config.cardInsertedCallback = NULL;
    config.cardRemovedCallback = NULL;
#endif
    config.blockGapCallback = NULL;

    if (kStatus_Success != SDHC_InstallMemCallback(&g_sdhcHost, &config))
    {
        return false;
    }

//    if (kStatus_Success != SDHC_Init(g_sdhcHost.base, &config))
//    {
//        return STA_NOINIT;
//    }

#if ! defined CD_USING_GPIO
    do
    {
        status = SDHC_DetectCard(SDHC_INSTANCE_ID);
        if (kStatus_SDHC_NoCardInsertedError == status)
        {
            retries--;
        }
        else if (kStatus_SDHC_UnknownStatus == status)
        {
            SD_DeInit(&g_sdhcCard);
            SDHC_DeInit(g_sdhcHost.base);
            return STA_NOINIT;
        }
        else
        {
            cardPresented = 1;
            break;
        }
    } while (retries);

    if (!retries && (!cardPresented))
    {
        SD_DeInit(&g_sdhcCard);
        SDHC_DeInit(g_sdhcHost.base);
        memset(&g_sdhcHost, 0, sizeof(g_sdhcHost));
        memset(&g_sdhcCard, 0, sizeof(g_sdhcCard));
        return STA_NOINIT;
    }
#endif

    if (kStatus_Success != SD_Init(&g_sdhcHost, &g_sdhcCard))
    {
        SDHC_DeInit(g_sdhcHost.base);
        return STA_NOINIT;
    }
    return 0;
}
