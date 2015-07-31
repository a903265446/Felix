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

#ifndef __SDHC_SD_H__
#define __SDHC_SD_H__

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/*! @addtogroup sdhc_carddrv_data_types */
/*! @{ */

#define SD_SCR_BUS_WIDTHS_1BIT  (1 << 0)                /*!< card supports 1 bit mode */
#define SD_SCR_BUS_WIDTHS_4BIT  (1 << 2)                /*!< card supports 4 bit mode */

#define SD_CCC_BASIC            (1 << 0)                /*!< Card command class 0 */
#define SD_CCC_BLOCK_READ       (1 << 2)                /*!< Card command class 2 */
#define SD_CCC_BLOCK_WRITE      (1 << 4)                /*!< Card command class 4 */
#define SD_CCC_ERASE            (1 << 5)                /*!< Card command class 5 */
#define SD_CCC_WRITE_PROTECTION (1 << 6)                /*!< Card command class 6 */
#define SD_CCC_LOCK_CARD        (1 << 7)                /*!< Card command class 7 */
#define SD_CCC_APP_SPEC         (1 << 8)                /*!< Card command class 8 */
#define SD_CCC_IO_MODE          (1 << 9)                /*!< Card command class 9 */
#define SD_CCC_SWITCH           (1 << 10)               /*!< Card command class 10 */

#define SD_OCR_CCS              (1 << 30)               /*!< card capacity status */
#define SD_OCR_HCS              (1 << 30)               /*!< card capacity status */
#define SD_OCR_XPC              (1 << 28)               /*!< SDXC power control */
#define SD_OCR_S18R             (1 << 24)               /*!< switch to 1.8V request */
#define SD_OCR_S18A             SD_OCR_S18R             /*!< switch to 1.8V accepted */

#define SD_HIGHSPEED_BUSY       (0x00020000U)           /*!< SD card high speed busy status bit in CMD6 response */
#define SD_HIGHSPEED_SUPPORTED  (0x00020000U)           /*!< SD card high speed support bit in CMD6 response */

#define SD_OCR_VDD_27_28        (1 << 15)               /*!< VDD 2.7-2.8 */
#define SD_OCR_VDD_28_29        (1 << 16)               /*!< VDD 2.8-2.9 */
#define SD_OCR_VDD_29_30        (1 << 17)               /*!< VDD 2.9-3.0 */
#define SD_OCR_VDD_30_31        (1 << 18)               /*!< VDD 3.0-3.1 */
#define SD_OCR_VDD_31_32        (1 << 19)               /*!< VDD 3.1-3.2 */
#define SD_OCR_VDD_32_33        (1 << 20)               /*!< VDD 3.2-3.3 */
#define SD_OCR_VDD_33_34        (1 << 21)               /*!< VDD 3.3-3.4 */
#define SD_OCR_VDD_34_35        (1 << 22)               /*!< VDD 3.4-3.5 */
#define SD_OCR_VDD_35_36        (1 << 23)               /*!< VDD 3.5-3.6 */

#define SD_SPEC_VERSION_1_0     (1 << 0)                   /*!< SD card version 1.0 */
#define SD_SPEC_VERSION_1_1     (1 << 1)                   /*!< SD card version 1.1 */
#define SD_SPEC_VERSION_2_0     (1 << 2)                   /*!< SD card version 2.0 */
#define SD_SPEC_VERSION_3_0     (1 << 3)                   /*!< SD card version 3.0 */

typedef enum _sd_cmd_t {
    kSdSendRelativeAddr = 3,        /*!< bcr                      R6 */
    kSdSwitch = 6,                  /*!< adtc     [31] mode       R1 */
                                    /*!<          [15:12] func */
                                    /*!<          group 4: current */
                                    /*!<          limit */
                                    /*!<          [11:8] func */
                                    /*!<          group 3: drive */
                                    /*!<          strength */
                                    /*!<          [7:4] func  */
                                    /*!<          group 2: command */
                                    /*!<          system */
                                    /*!<          [3:0] func */
                                    /*!<          group 1: access */
                                    /*!<          mode */
    kSdSendIfCond = 8,              /*!< bcr      [11:8] supply   R7 */
                                    /*!<          voltage */
                                    /*!<          [7:0] check */
                                    /*!<          pattern */
    kSdVoltageSwitch = 11,          /*!< ac                       R1 */
    kSdSpeedClassControl = 20,      /*!< ac       [31:28] speed   R1b */
                                    /*!<          class control */
    kSdEraseWrBlkStart = 32,        /*!< ac       [31:0] data     R1 */
                                    /*!<          address */
    kSdEraseWrBlkEnd = 33,          /*!< ac       [31:0] data     R1 */
                                    /*!<          address */
} sd_cmd_t;

typedef enum _sd_acmd_t {
    kSdAppSetBusWdith = 6,          /*!< ac       [1:0] bus       R1 */
                                    /*!<          width */
    kSdAppStatus = 13,              /*!< adtc                     R1 */
    kSdAppSendNumWrBlocks = 22,     /*!< adtc                     R1 */
    kSdAppSetWrBlkEraseCount = 23,  /*!< ac       [22:0] number   R1 */
                                    /*!<          of blocks */
    kSdAppSendOpCond = 41,          /*!< bcr      [30] HCS        R3 */
                                    /*!<          [28] XPC */
                                    /*!<          [24] S18R */
                                    /*!<          [23:0] VDD */
                                    /*!<          voltage window */
    kSdAppSetClrCardDetect = 42,    /*!< ac       [0] set cd      R1 */
    kSdAppSendScr = 51,             /*!< adtc                     R1 */
} sd_acmd_t;

typedef struct SdCsd {
    uint8_t csdStructure;           /*!< CSD structure [127:126] */
    uint8_t taac;                   /*!< Data read access-time-1 [119:112] */
    uint8_t nsac;                   /*!< Data read access-time-2 in clock cycles (NSAC*100) [111:104] */
    uint8_t tranSpeed;              /*!< Maximum data transfer rate [103:96] */
    uint16_t ccc;                   /*!< Card command classes [95:84] */
    uint8_t readBlkLen;             /*!< Maximum read data block length [83:80] */
    uint16_t flags;                 /*!< Card flags */
#define SD_CSD_READ_BL_PARTIAL           (1<<0)         /*!< Partial blocks for read allowed [79:79]*/
#define SD_CSD_WRITE_BLK_MISALIGN        (1<<1)         /*!< Write block misalignment [78:78]*/
#define SD_CSD_READ_BLK_MISALIGN         (1<<2)         /*!< Read block misalignment [77:77]*/
#define SD_CSD_DSR_IMP                   (1<<3)         /*!< DSR implemented [76:76] */
#define SD_CSD_ERASE_BLK_ENABLED         (1<<4)         /*!< Erase single block enabled [46:46] */
#define SD_CSD_WP_GRP_ENABLED            (1<<5)         /*!< Write protect group enabled [31:31] */
#define SD_CSD_WRITE_BL_PARTIAL          (1<<6)         /*!< Partial blocks for write allowed [21:21]*/
#define SD_CSD_FILE_FORMAT_GROUP         (1<<7)         /*!< File format group [15:15]*/
#define SD_CSD_COPY                      (1<<8)         /*!< Copy flag [14:14]*/
#define SD_CSD_PERM_WRITE_PROTECT        (1<<9)         /*!< Permanent write protection [13:13]*/
#define SD_CSD_TMP_WRITE_PROTECT         (1<<10)        /*!< Temporary write protection [12:12]*/
    uint32_t cSize;             /*!< Device size [73:62] */
    uint8_t vddRCurrMin;        /*!< Maximum read current @VDD min [61:59] */
    uint8_t vddRCurrMax;        /*!< Maximum read current @VDD max [58:56] */
    uint8_t vddWCurrMin;        /*!< Maximum write current @VDD min [55:53] */
    uint8_t vddWCurrMax;        /*!< Maximum write current @VDD max [52:50] */
    uint8_t cSizeMult;          /*!< Device size multiplier [49:47] */
    uint8_t sectorSize;         /*!< Erase sector size [45:39] */
    uint8_t wpGrpSize;          /*!< Write protect group size [38:32] */
    uint8_t r2wFactor;          /*!< Write speed factor [28:26] */
    uint8_t writeBlkLen;        /*!< Maximum write data block length [25:22] */
    uint8_t fileFormat;         /*!< File format [11:10] */
    uint8_t reserved;
} sd_csd_t;

typedef struct SdScr {
    uint8_t scrStructure;       /*!< SCR Structure [63:60] */
    uint8_t sdSpec;             /*!< SD memory card spec. version [59:56] */
    uint16_t flags;             /*!< SCR flags */
#define SD_SCR_DATA_STAT_AFTER_ERASE     (1<<0)         /*!< Data status after erases [55:55]*/
#define SD_SCR_SD_SPEC3                  (1<<1)         /*!< Spec. version 3.00 or higher [47:47]*/
    uint8_t sdSecurity;         /*!< CPRM security support [54:52] */
    uint8_t sdBusWidths;        /*!< Data bus widths supported [51:48] */
    uint8_t exSecurity;         /*!< Extended security support [46:43] */
    uint8_t cmdSupport;         /*!< Command support bits [33:32] */
    uint32_t reservedForMan;    /*!< reserved for manufacturer usage [31:0] */
} sd_scr_t;

typedef struct SdCid {
    uint8_t mid;                    /*!< Manufacturer ID [127:120] */
    uint16_t oid;                   /*!< OEM/Application ID [119:104] */
    uint8_t pnm[6];                 /*!< Product name [103:64] */
    uint8_t prv;                 /*!< Product revision [63:56] */
    uint32_t psn;                   /*!< Product serial number [55:24] */
    uint16_t mdt;                 /*!< Manufacturing date [19:8] */
} sd_cid_t;





/*! @brief Defines the SD card API's running status. */
typedef enum _sd_status
{    
    kStatus_SD_NoError         = 0U, /*!< Success */
    kStatus_SD_InvalidArgument = 1U, /*!< Invalid argument existed. */
    kStatus_SD_Failed          = 2U  /*!< Failed. */      
} sd_status_t;

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*!
 * @brief SDHC Card Structure
 *
 * Defines the card structure including the necessary fields to identify and
 * describe the card.
 */
typedef struct SdhcCard 
{
    sdhc_host_t *host;                                  /*!< Host state information */
    sdcard_type_t cardType;                             /*!< Card type */
    uint32_t rca;                                       /*!< Relative address of the card */
    uint32_t version;                                   /*!< Card version */
    uint32_t caps;                                      /*!< Capability */
#define SDMMC_CARD_CAPS_HIGHSPEED           (1 << 0)    /*!< SD card high speed support bit */
#define SDMMC_CARD_CAPS_HIGHCAPACITY        (1 << 1)    /*!< Card is high capacity */
#define SDMMC_CARD_CAPS_BUSWIDTH_4BITS      (1 << 2)    /*!< 4-bit data width support bit */
#define SDMMC_CARD_CAPS_BUSWIDTH_8BITS      (1 << 3)    /*!< 8-bit data width support bit */
#define SDMMC_CARD_CAPS_SDHC                (1 << 5)    /*!< Card is SDHC */
#define SDMMC_CARD_CAPS_SDXC                (1 << 6)    /*!< Card is SDXC */
    uint32_t rawCid[4];                                 /*!< CID */
    uint32_t rawCsd[4];                                 /*!< CSD */
    uint32_t rawScr[2];                                 /*!< CSD */
    uint32_t ocr;                                       /*!< OCR */
    sdcard_cid_t cid;                                   /*!< CID */
    sdcard_csd_t csd;                                   /*!< CSD */
    sdcard_scr_t scr;                                   /*!< SCR */
    uint32_t blockCount;                                /*!< Card total block number */
    uint32_t blockSize;                                 /*!< Card block size */
} sdhc_card_t;

#define DOES_CARD_SUPPORT_HIGHSPEED(x)      ((x)->caps & SDMMC_CARD_CAPS_HIGHSPEED)
#define DOES_CARD_SUPPORT_4BITS(x)          ((x)->caps & SDMMC_CARD_CAPS_BUSWIDTH_4BITS)
#define IS_HIGHCAPACITY_CARD(x)             ((x)->caps & SDMMC_CARD_CAPS_HIGHCAPACITY)
#define IS_SD_CARD(x)                       ((x)->cardType == kCardTypeSd)
#define IS_MMC_CARD(x)                      ((x)->cardType == kCardTypeMmc)
#define IS_SDIO_CARD(x)                     ((x)->cardType == kCardTypeSdio)
#define CARD_BLOCK_LEN(x)                   ((uint32_t)(1 << (x)))
#define SDHC_CARD_MAX_VOLT_RETRIES          (1000)

#define SDHC_CARD_DEFAULT_BLOCK_SIZE        (512)     /*!< Default block size */


/*************************************************************************************************
 * API
 ************************************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif

/*! @name SDHC CARD DRIVER FUNCTION */
/*@{ */

/*!
 * @brief Initializes the card on a specific host controller.
 *
 * This function initializes the card on a specific SDHC.
 *
 * @param host the pointer to the host struct, it is allocated by user
 * @param card the place to store card related information
 * @return kStatus_SDCARD_NoError on success
 */
sdcard_status_t SDCARD_Init(sdhc_host_t *host, sdhc_card_t *card);

/*!
 * @brief Reads blocks from the specific card.
 *
 * This function reads blocks from specific card, with default
 * block size defined by SDHC_CARD_DEFAULT_BLOCK_SIZE.
 *
 * @param card the handle of the card
 * @param buffer the buffer to hold the data read from card
 * @param startBlock the start block index
 * @param blockCount the number of blocks to read
 * @return kStatus_SDCARD_NoError on success
 */
sdcard_status_t SDCARD_ReadBlocks(sdhc_card_t *card, uint8_t *buffer, uint32_t startBlock, uint32_t blockCount);

/*!
 * @brief Writes blocks of data to the specific card.
 *
 * This function writes blocks to specific card, with default block size defined 
 * by SDHC_CARD_DEFAULT_BLOCK_SIZE.
 *
 * @param card the handle of the card
 * @param buffer the buffer holding the data to be written to the card
 * @param startBlock the start block index
 * @param blockCount the number of blocks to write
 * @return kStatus_SDCARD_NoError on success
 */
sdcard_status_t SDCARD_WriteBlocks(sdhc_card_t *card, uint8_t *buffer, uint32_t startBlock, uint32_t blockCount);

/*!
 * @brief Erases blocks of the specific card.
 *
 * This function erases blocks of a specific card, with default
 * block size defined by the SDHC_CARD_DEFAULT_BLOCK_SIZE.
 *
 * @param card the handle of the card
 * @param startBlock the start block index
 * @param blockCount the number of blocks to erase
 * @return kStatus_SDCARD_NoError on success
 */
sdcard_status_t SDCARD_EraseBlocks(sdhc_card_t *card, uint32_t startBlock, uint32_t blockCount);

/*!
 * @brief Checks whether the card is write-protected.
 *
 * This function checks if the card is write-protected via CSD register.
 *
 * @param card the specific card
 * @return kStatus_SDCARD_NoError on success
 */
bool SDCARD_CheckReadOnly(sdhc_card_t *card);

/*!
 * @brief Deinitializes the card.
 *
 * This function deinitializes the specific card.
 *
 * @param card the specific card
 */
void SDCARD_Shutdown(sdhc_card_t *card);

/*!
 * @brief Checks whether the card is present on a specified host controller.
 *
 * This function checks if there's a card inserted in the SDHC. It is mainly 
 * used in the polling detection pin mode when DAT3 or dedicate CD pin is selected
 * as card detection pin.
 *
 * @param card the specific card
 * @return kStatus_SDCARD_NoError on success
 */
sdcard_status_t SDCARD_DetectCard(sdhc_card_t *card);


/*!
 * @brief IRQ handler for SDHC
 *
 * This function deals with IRQs on the given host controller.
 *
 * @param host the host state inforamtion
 */
void SDCARD_HostIrqHandler(sdhc_host_t* host);

/*@} */
#if defined(__cplusplus)
}
#endif
/*! @} */
#endif  /* __SDHC_CARD_H__*/

/*************************************************************************************************
 * EOF
 ************************************************************************************************/