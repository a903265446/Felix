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
#ifndef __CARD_H__
#define __CARD_H__


#include "ksdk_common.h"
#include "spec.h"
#include "sdhc.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*!
 * @addtogroup card_driver
 * @{
 */


/*! @brief Defines the SD/MMC card API's running status. */
typedef enum _sdmmc_status
{   
    kStatus_SDMMC_NotSupportYet          = MAKE_STATUS(kStatusGroup_SDMMC, 0),/*!< Havn't supported */
    kStatus_SDMMC_SendCommandFailed      = MAKE_STATUS(kStatusGroup_SDMMC, 1),/*!< Send command failed */
    kStatus_SDMMC_TransferDataFailed     = MAKE_STATUS(kStatusGroup_SDMMC, 2),/*!< Send command failed */
    kStatus_SDMMC_SetCardBusClockFailed  = MAKE_STATUS(kStatusGroup_SDMMC, 1),/*!< Set card bus clock failed */  
    kStatus_SDMMC_CardStatusError        = MAKE_STATUS(kStatusGroup_SDMMC, 3),/*!< Card status is error */
    kStatus_SDMMC_SetCardBlockSizeFailed = MAKE_STATUS(kStatusGroup_SDMMC, 4),/* Set card block size failed */
    kStatus_SDMMC_CardNotSupport         = MAKE_STATUS(kStatusGroup_SDMMC, 5),/*!< Card doesn't support */ 
    kStatus_SDMMC_InvalidIORange         = MAKE_STATUS(kStatusGroup_SDMMC, 6),/*!< Invalid read/write address range */\
    kStatus_SDMMC_AllSendCidFailed       = MAKE_STATUS(kStatusGroup_SDMMC, 7),/*!< AllSendCid failed */
    kStatus_SDMMC_SendRcaFailed          = MAKE_STATUS(kStatusGroup_SDMMC, 8),/*!< SendRca failed */
    kStatus_SDMMC_SendCsdFailed          = MAKE_STATUS(kStatusGroup_SDMMC, 9),/*!< SendCsd failed */
    kStatus_SDMMC_SelectCardFailed       = MAKE_STATUS(kStatusGroup_SDMMC, 10),/*!< SelectCard failed */
    kStatus_SDMMC_SendScrFailed          = MAKE_STATUS(kStatusGroup_SDMMC, 11),/*!< SendScr failed */ 
    kStatus_SDMMC_SetBusWidthFailed      = MAKE_STATUS(kStatusGroup_SDMMC, 12),/*!< SetBusWidth failed */
    kStatus_SDMMC_GoIdleFailed           = MAKE_STATUS(kStatusGroup_SDMMC, 13),/*!< GoIdle failed */
    kStatus_SDMMC_SendOpCondFailed       = MAKE_STATUS(kStatusGroup_SDMMC, 14),/*!< SendOpCond failed */
    kStatus_SDMMC_EraseFailed            = MAKE_STATUS(kStatusGroup_SDMMC, 15),/*!< Erase failed */
    kStatus_SDMMC_SendAppCmdFailed       = MAKE_STATUS(kStatusGroup_SDMMC, 16),/*!< Send application command failed */
    kStatus_SDMMC_SwitchFailed           = MAKE_STATUS(kStatusGroup_SDMMC, 16),/*!< SwitchFunction failed */ 
    kStatus_SDMMC_StopTransmissionFailed = MAKE_STATUS(kStatusGroup_SDMMC, 17),/*!< StopTransmission failed */
    kStatus_SDMMC_SendStatusFailed       = MAKE_STATUS(kStatusGroup_SDMMC, 18),/*!< SendStatus failed */
    kStatus_SDMMC_SetBlockCountFailed    = MAKE_STATUS(kStatusGroup_SDMMC, 19),/*!< SetBlockCount failed*/
    kStatus_SDMMC_SetRelativeAddrFailed  = MAKE_STATUS(kStatusGroup_SDMMC, 20),/*!< SetRelativeAddr failed */ 
    kStatus_SDMMC_GetPowerClassFailed    = MAKE_STATUS(kStatusGroup_SDMMC, 21),/*!< Get power class failed */
    kStatus_SDMMC_SetPowerClassFailed    = MAKE_STATUS(kStatusGroup_SDMMC, 22),/*!< Set power class failed */
    kStatus_SDMMC_BusTestProcessFailed   = MAKE_STATUS(kStatusGroup_SDMMC, 23),/*!< Bus test process failed */
    kStatus_SDMMC_SwitchHighSpeedFailed  = MAKE_STATUS(kStatusGroup_SDMMC, 24),/*!< Switch high speed failed */
    kStatus_SDMMC_SendExtCsdFailed       = MAKE_STATUS(kStatusGroup_SDMMC, 25),/*!< SendExtCsd failed */
    kStatus_SDMMC_NotEraseGroupAddress   = MAKE_STATUS(kStatusGroup_SDMMC, 26),/*!< Erase address is not erase group address */
    kStatus_SDMMC_ConfigBootFailed       = MAKE_STATUS(kStatusGroup_SDMMC, 27),/*!< Configure boot feature failed */
} sdmmc_status_t;



/*!
 * @brief SD Card Structure
 *
 * Defines the card structure including the necessary fields to identify and
 * describe the card.
 */
typedef struct SDCard 
{
    SDHC_Type *hostBase;         /*!< host regist base address */
    sdhc_host_t * host;          /*!< Host state information */
    uint32_t rca;                /*!< Relative address of the card */
    uint32_t version;            /*!< Card version */
    uint32_t caps;               /*!< Capability mask */
#define SD_CAPS_HIGH_CAPACITY    (1U << 1U)/*!< Card is high capacity */
#define SD_CAPS_BUS_WIDTH_4BITS  (1U << 2U)/*!< Support 4-bit data width */
#define SD_CAPS_SDHC             (1U << 3U)/*!< Card is SDHC */
#define SD_CAPS_SDXC             (1U << 4U)/*!< Card is SDXC */
    uint32_t rawCid[4];          /*!< Raw CID content */
    uint32_t rawCsd[4];          /*!< Raw CSD content */
    uint32_t rawScr[2];          /*!< Raw CSD content */
    uint32_t ocr;                /*!< Raw OCR content */
    sd_cid_t cid;                /*!< CID */
    sd_csd_t csd;                /*!< CSD */
    sd_scr_t scr;                /*!< SCR */
    uint32_t blockCount;         /*!< Card total block number */
    uint32_t blockSize;          /*!< Card block size */
} sd_card_t;

/* Checks if card support 4 bit width */
#define DOES_SD_SUPPORT_4BITS(x)          ((x)->caps & SD_CAPS_BUS_WIDTH_4BITS)
/* Checks if card support high speed mode. */
#define IS_SD_HIGH_CAPACITY(x)            ((x)->caps & SD_CAPS_HIGH_CAPACITY)

typedef struct MMCCard
{
    sdhc_host_t * host;                         /*!< Host state information */
    uint32_t rca;                               /*!< Relative address of the card */
    bool enablePreDefBlkCnt;                    /*!< Enables PRE-DEFINED block count when read/write*/
    uint32_t caps;                              /*!< Capability */
#define MMC_CAPS_HIGH_SPEED        (1U << 0U)   /*!< Card high speed support bit */
#define MMC_CAPS_HIGH_SPEED_52MHZ  (1U << 1U)   /*!< Card support high speed 52MHZ */
#define MMC_CAPS_HIGH_SPEED_26MHZ  (1U << 2U)   /*!< Card support high speed 26MHZ */
#define MMC_CAPS_HIGH_CAPACITY     (1U << 3U)   /*!< Card is high capacity */
#define MMC_CAPS_EXT_CSD           (1U << 4U)   /*!< Card support switch command */
#define MMC_CAPS_ALTER_BOOT        (1U << 5U)   /*!< MMC card support alternate boot */
    uint32_t rawCid[4];                         /*!< CID */
    uint32_t rawCsd[4];                         /*!< CSD */
    uint32_t rawExtCsd[MMC_EXT_CSD_LEN_AS_WORD];/*!< MMC EXT_CSD */
    uint32_t ocr;                               /*!< MMC OCR */
    mmc_cid_t cid;                              /*!< MMC CID */
    mmc_csd_t csd;                              /*!< MMC CSD */
    mmc_ext_csd_t extCsd;                       /*!< MMC EXT_CSD */  
    uint32_t blockCount;                        /*!< Card total block number */
    uint32_t blockSize;                         /*!< Card block size */
    uint32_t eraseGroupSize;                    /*!< Erase group size united as block size */
    uint32_t writeProtectGroupSize;             /*!< Write protect group size united as erase group size */
    uint32_t bootPartitionSize;                 /*!< Boot partition size united as blocks */          
    mmc_volt_range_t hostVoltRange;             /*!< Host intended volt range */ 
    mmc_access_partition_t currentPartition;    /*!< Current access partition */
} mmc_card_t;

#define DOES_MMC_SUPPORT_HIGH_SPEED_52MHZ(x)  ((x)->caps & MMC_CAPS_HIGH_SPEED_52MHZ) 
#define DOES_MMC_SUPPORT_HIGH_SPEED_26MHZ(x)  ((x)->caps & MMC_CAPS_HIGH_SPEED_26MHZ)
/* Checks if card support high speed mode. */
#define IS_MMC_HIGH_CAPACITY(x)               ((x)->caps & MMC_CAPS_HIGH_CAPACITY)


/*!
 * @brief MMC card boot configuration definition.
 */
typedef struct MMCBootConfig
{
    bool bootAckEnable;                             /*!< Boot ACK enable */
    mmc_boot_partition_enable_t bootPartitionEnable;/*!< Boot partition */
    bool retainBootBusWidth;                        /*!< If retain boot bus width */
    mmc_bus_width_t bootBusWidth;                   /*!< Boot bus width */
} mmc_boot_config_t;

/* Maximum loop count to check the card operation volt range */
#define FSL_CARD_MAX_VOLT_RETRIES           (1000U)
/* Default block size */
#define FSL_CARD_DEFAULT_BLOCK_SIZE         (512U) 
/* Card command maximum timeout value */     
#define FSL_CARD_COMMAND_TIMEOUT            (5000U)
/*************************************************************************************************
 * API
 ************************************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @name Card Driver Function
 * @{ 
 */

/*!
 * @brief Initializes the card on a specific host controller.
 *
 * This function initializes the card on a specific SDHC.
 *
 * @param host The pointer to store the host inforamtion.
 * @param card The pointer to store card related information.
 * @return kStatus_SDMMC_NoError on success
 */
status_t SD_Init(sd_card_t *card);

/*! 
 * @name Operate the card
 * @{ 
 */

/*!
 * @brief Reads blocks from the specific card.
 *
 * This function reads blocks from specific card, with default
 * block size defined by SDHC_CARD_DEFAULT_BLOCK_SIZE.
 *
 * @param card The handle of the card
 * @param buffer The buffer to hold the data read from card
 * @param startBlock The start block index
 * @param blockCount The number of blocks to read
 * @return kStatus_SDMMC_NoError on success
 */
status_t SD_ReadBlocks(sd_card_t *card, uint8_t *buffer, uint32_t startBlock, uint32_t blockCount);

/*!
 * @brief Writes blocks of data to the specific card.
 *
 * This function writes blocks to specific card, with default block size defined 
 * by SDHC_CARD_DEFAULT_BLOCK_SIZE.
 *
 * @param card The handle of the card
 * @param buffer The buffer holding the data to be written to the card
 * @param startBlock The start block index
 * @param blockCount The number of blocks to write
 * @return kStatus_SDMMC_NoError on success
 */
status_t SD_WriteBlocks(sd_card_t *card, uint8_t *buffer, uint32_t startBlock, uint32_t blockCount);

/*!
 * @brief Erases blocks of the specific card.
 *
 * This function erases blocks of a specific card, with default block size 
 * defined by the SDHC_CARD_DEFAULT_BLOCK_SIZE.
 *
 * @param card The handle of the card
 * @param startBlock The start block index
 * @param blockCount The number of blocks to erase
 * @return kStatus_SDMMC_NoError on success
 */
status_t SD_EraseBlocks(sd_card_t *card, uint32_t startBlock, uint32_t blockCount);

/*!
 * @brief Checks whether the card is write-protected.
 *
 * This function checks if the card is write-protected via CSD register.
 *
 * @param card The specific card
 * @return kStatus_SDMMC_NoError on success
 */
bool SD_CheckReadOnly(sd_card_t *card);

/*!
 * @brief Deinitializes the card.
 *
 * This function deinitializes the specific card.
 *
 * @param card The specific card
 */
void SD_DeInit(sd_card_t *card);

/*!
 * @brief Initializes the MMC card.
 * @param host Host controller descriptor
 * @param card Card descriptor
 * @return kStatus_SDMMC_NoError on success
 */
status_t MMC_Init(mmc_card_t *card);

/*!
 * Checks the card read only feature.
 * @param card Card descriptor
 * @return Read only or not
 *         - Card is read only
 *         - Card is not read only
 */
bool MMC_CheckReadOnly(mmc_card_t *card);

/*!
 * Selects the partition to access.
 * @param  card Card descriptor
 * @param  partitionNumber The partition number
 * @return kStatus_SDMMC_NoError on success
 */
status_t MMC_SelectAccessPartition(mmc_card_t *card, mmc_access_partition_t partitionNumber);

/*!
 * Reads data blocks from the card.
 * @param  card Card descriptor
 * @param  buffer The buffer to save data blocks.
 * @param  startBlock Start block
 * @param  blockCount Block count
 * @return kStatus_SDMMC_NoError on success
 */
status_t MMC_ReadBlocks(mmc_card_t *card,  uint8_t *buffer, uint32_t startBlock, uint32_t blockCount);

/*!
 * Writes data blocks to the card.
 * @param  card Card descriptor
 * @param  buffer The buffer to save data blocks
 * @param  startBlock Start block number.
 * @param  blockCount Block count
 * @return kStatus_SDMMC_NoError on success
 */
status_t MMC_WriteBlocks(mmc_card_t *card, uint8_t *buffer, uint32_t startBlock, uint32_t blockCount);

/*!
 * Erases blocks of the card.
 * @param  card Card descriptor
 * @param  startBlock Start block
 * @param  blockCount Block count
 * @return kStatus_SDMMC_NoError on success
 */
status_t MMC_EraseBlocks(mmc_card_t *card, uint32_t startBlock, uint32_t blockCount);

/*!
 * Configures boot activity of the card.
 * @param  card Card descriptor
 * @param  configPtr Boot configuration structure
 * @return kStatus_SDMMC_NoError on success
 */
status_t MMC_ConfigBoot(mmc_card_t *card, const mmc_boot_config_t *configPtr);

/*!
 * DeInitializes the card.
 * @param card Card descriptor.
 */

void MMC_DeInit(mmc_card_t *card);


/* @} */
#if defined(__cplusplus)
}
#endif
/*! @} */
#endif  /* __CARD_H__*/

 