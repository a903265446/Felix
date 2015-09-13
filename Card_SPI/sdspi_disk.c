#include <assert.h>
#include <stdio.h>
#include <string.h>

#include "sd_disk.h"
#include "diskio.h"


#include "test_sdspi_callback.h"
#include "fsl_dspi_master_driver.h"
//#include "fsl_dspi_edma_master_driver.h"
#include "sdspi.h"
#include "fsl_debug_console.h"

static sdspi_host_t g_spiHost = {0};
static sdspi_card_t g_spiCard = {0};
static dspi_master_state_t g_dspiState;
static dspi_device_t g_dspiDevice = {0};
static uint32_t g_card_initialized = 0;

static status_t setSpiFrequency(SPI_Type *base, uint32_t frequency)
{
    uint32_t calculatedBaudRate;
    calculatedBaudRate = DSPI_HAL_SetBaudRate(base,
            ((dspi_master_state_t *)(g_spiHost.spiState))->whichCtar,
            frequency,
            ((dspi_master_state_t *)(g_spiHost.spiState))->dspiSourceClock);
    if (0 == calculatedBaudRate)
    {
        return 1;
    }
    PRINTF("BaudRate set to %dHz\r\n", calculatedBaudRate);
    return 0;
}



static status_t spiExchange(SPI_Type *base, const uint8_t *in, uint8_t *out, uint32_t size)
{
    if (kStatus_DSPI_Success == DSPI_DRV_MasterTransferBlocking(SDSPI_INSTANCE,
                  NULL, in, out, size, FSL_SDSPI_TIMEOUT))
    {
        return 0;
    }
    return 1;
}

static void reset_all_states()
{
    SDSPI_DeInit(&g_spiCard);
    DSPI_DRV_MasterDeinit(SDSPI_INSTANCE);
#if SPI_USING_DMA
    EDMA_DRV_Deinit();
#endif
    g_card_initialized = 0;

    memset(&g_spiHost, 0, sizeof(g_spiHost));
    memset(&g_spiCard, 0, sizeof(g_spiCard));
    memset(&g_dspiState, 0, sizeof(g_dspiState));
    memset(&g_dspiDevice, 0, sizeof(g_dspiDevice));
#if SPI_USING_DMA
    memset(&g_dmaState, 0, sizeof(g_dmaState));
#endif
}

DSTATUS sd_disk_initialize(uint8_t pdrv)
{
    dspi_master_user_config_t dspiConfig;
    uint32_t calculatedBaudRate;
    sdspi_card_t *card;
    
    uint8_t in[4] = {1, 2, 3, 4};
    uint8_t out[4];
    
    if (pdrv != SD)
    {
        return STA_NOINIT;
    }

    if (g_card_initialized)
    {
        reset_all_states();
    }

    /* Init SPI driver */
    memset(&dspiConfig, 0, sizeof(dspiConfig));
    
    /* Dspi none DMA mode Init*/
    dspiConfig.isChipSelectContinuous = true;
    dspiConfig.isSckContinuous = false;
    dspiConfig.pcsPolarity = kDspiPcs_ActiveLow;
    dspiConfig.whichCtar = kDspiCtar0;
    dspiConfig.whichPcs = kDspiPcs0;

    DSPI_DRV_MasterInit(SDSPI_INSTANCE, &g_dspiState, &dspiConfig);

    g_dspiDevice.dataBusConfig.bitsPerFrame = 8;
    g_dspiDevice.dataBusConfig.clkPhase = kDspiClockPhase_FirstEdge;
    g_dspiDevice.dataBusConfig.clkPolarity = kDspiClockPolarity_ActiveHigh;
    g_dspiDevice.dataBusConfig.direction = kDspiMsbFirst;
    //g_dspiDevice.bitsPerSec = 1000000;
    g_dspiDevice.bitsPerSec = 400000;
    DSPI_DRV_MasterConfigureBus(SDSPI_INSTANCE, &g_dspiDevice, &calculatedBaudRate);

    /* Test SPI driver */
    DSPI_DRV_MasterTransferBlocking(SDSPI_INSTANCE, NULL, in, out, 4, FSL_SDSPI_TIMEOUT);
    DSPI_DRV_MasterTransferBlocking(SDSPI_INSTANCE, NULL, in, out, 2, FSL_SDSPI_TIMEOUT);
    DSPI_DRV_MasterTransferBlocking(SDSPI_INSTANCE, NULL, in, out, 2, FSL_SDSPI_TIMEOUT);
    DSPI_DRV_MasterTransferBlocking(SDSPI_INSTANCE, NULL, in, out, 1, FSL_SDSPI_TIMEOUT);
    DSPI_DRV_MasterTransferBlocking(SDSPI_INSTANCE, NULL, in, out, 2, FSL_SDSPI_TIMEOUT);
    DSPI_DRV_MasterTransferBlocking(SDSPI_INSTANCE, NULL, in, out, 2, FSL_SDSPI_TIMEOUT);
    DSPI_DRV_MasterTransferBlocking(SDSPI_INSTANCE, NULL, in, out, 1, FSL_SDSPI_TIMEOUT);
    DSPI_DRV_MasterTransferBlocking(SDSPI_INSTANCE, NULL, in, out, 1, FSL_SDSPI_TIMEOUT);
    DSPI_DRV_MasterTransferBlocking(SDSPI_INSTANCE, NULL, in, out, 2, FSL_SDSPI_TIMEOUT);
    //while(1);
    
        /* Init SDSPI driver */
#if defined FRDM_K22F
    g_spiHost.base = SPI0;
#endif
#if defined TWR_K22F120M
    g_spiHost.base = SPI1;
#endif
    g_spiHost.setFrequency = setSpiFrequency;
    g_spiHost.exchange = spiExchange;
    g_spiHost.busBaudRate = calculatedBaudRate;
    g_spiHost.getCurrentTimeMsec = get_current_time_msec;
    g_spiHost.timeRangeMsec = get_time_range_msec();
    g_spiHost.spiState = &g_dspiState;
    g_spiHost.spiDevice = &g_dspiDevice;

    g_spiCard.host = &g_spiHost;
    card = &g_spiCard;
    
    /* Initializes card. */
    if (kStatus_Success != SDSPI_Init(card))
    {
        PRINTF("SDSPI_Init failed\r\n");
        DSPI_DRV_MasterDeinit(SDSPI_INSTANCE);
#if defined SPI_USING_DMA
        EDMA_DRV_Deinit();
#endif
        return STA_NOINIT;
    }

    g_card_initialized = 1;
    
    return RES_OK;
}

DSTATUS sd_disk_status(uint8_t pdrv)
{
    if (pdrv != SD)
    {
        return STA_NOINIT;
    }

    return 0;
}
DRESULT sd_disk_read(uint8_t pdrv, uint8_t *buff, uint32_t sector, uint8_t count)
{
    if (pdrv != SD)
    {
        return RES_PARERR;
    }

    if (kStatus_Success != SDSPI_ReadBlocks(&g_spiCard, buff, sector, count))
    {
        return RES_ERROR;
    }
    return RES_OK;
}

DRESULT sd_disk_write(uint8_t pdrv, const uint8_t *buff, uint32_t sector, uint8_t count)
{
    if (pdrv != SD)
    {
        return RES_PARERR;
    }

    if (kStatus_Success != SDSPI_WriteBlocks(&g_spiCard, (uint8_t *)buff, sector, count))
    {
        return RES_ERROR;
    }
    return RES_OK;
}

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
                *(uint32_t *)buff = g_spiCard.blockCount;
            }
            else
            {
                res = RES_PARERR;
            }
            break;
        case GET_SECTOR_SIZE:
            if (buff)
            {
                *(uint32_t *)buff = g_spiCard.blockSize;
            }
            else
            {
                res = RES_PARERR;
            }
            break;
        case GET_BLOCK_SIZE:
            if (buff)
            {
//                if (IS_SD_CARD(&g_spiCard))
//                {
                    // if (g_spiCard.version == kSdCardVersion_1_x)
                    // {
                        //*(uint32_t *)buff = SD_CSD_SECTOR_SIZE(g_spiCard.rawCsd);
                        *(uint32_t *)buff = g_spiCard.csd.sectorSize;
                    // }
                    // else
                    // {
                    //     *(uint32_t *)buff = SDV20_CSD_SECTOR_SIZE(g_spiCard.rawCsd);
                    // }
//                }
//                else
//                {
//                    res = RES_PARERR;
//                }
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
                // switch (g_spiCard.cardType)
                // {
                //     case kCardTypeMmc:
                //         *(uint32_t *)buff = CT_MMC;
                //         break;
                //     case kCardTypeSd:
                //         if (g_spiCard.version == kSdCardVersion_1_x)
                //         {
                //         *(uint32_t *)buff = CT_SD1;
                //         }
                //         else
                //         {
                //             *(uint32_t *)buff = CT_SD2;
                //             if ((g_spiCard.caps & SDSPI_CAPS_SDHC) ||
                //                 (g_spiCard.caps & SDSPI_CAPS_SDXC))
                //             {
                //                 *(uint32_t *)buff |= CT_BLOCK;
                //             }
                //         }
                //         break;
                //     default:
                //         res = RES_PARERR;
                //         break;
                // }
                if (g_spiCard.version == kSDVersion_1_x)
                {
                *(uint32_t *)buff = CT_SD1;
                }
                else
                {
                    *(uint32_t *)buff = CT_SD2;
                    if ((g_spiCard.caps & SDSPI_CAPS_SDHC) ||
                        (g_spiCard.caps & SDSPI_CAPS_SDXC))
                    {
                        *(uint32_t *)buff |= CT_BLOCK;
                    }
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
                memcpy(buff, g_spiCard.rawCsd, sizeof(g_spiCard.rawCsd));
            }
            else
            {
                res = RES_PARERR;
            }
            break;
        case SDMMC_GET_CID:
            if (buff)
            {
                memcpy(buff, g_spiCard.rawCid, sizeof(g_spiCard.rawCid));
            }
            else
            {
                res = RES_PARERR;
            }
            break;
        case SDMMC_GET_OCR:
            if (buff)
            {
                *(uint32_t *)buff = g_spiCard.ocr;
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
