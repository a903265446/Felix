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

#ifndef __SPEC_H__
#define __SPEC_H__

#include <stdint.h>

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*! @addtogroup sd_mmc_card_spec_data_types */
/*! @{ */


#define SDMMC_CARD_BUSY         ((uint32_t) 1 << 31)     /*!< card initialization complete */

#define SDMMC_CLK_100KHZ        (100000U)
#define SDMMC_CLK_400KHZ        (400000U)
/* SD card high speed clock frequence. */
#define SD_CLK_25MHZ            (25000000U)
#define SD_CLK_50MHZ            (50000000U)
/* MMC card high speed clock frequence. */
#define MMC_CLK_26MHZ           (26000000U)
#define MMC_CLK_52MHZ           (52000000U)

#define SDMMC_R1_OUT_OF_RANGE         ((uint32_t) 1 << 31)      /*!< R1: out of range status bit */
#define SDMMC_R1_ADDRESS_ERROR        (1 << 30)                 /*!< R1: address error status bit */
#define SDMMC_R1_BLK_LEN_ERROR      (1 << 29)                 /*!< R1: block length error status bit */
#define SDMMC_R1_ERASE_SEQ_ERROR      (1 << 28)                 /*!< R1: erase sequence error status bit */
#define SDMMC_R1_ERASE_PARAM          (1 << 27)                 /*!< R1: erase parameter error status bit */
#define SDMMC_R1_WP_VIOLATION         (1 << 26)                 /*!< R1: write protection violation status bit */
#define SDMMC_R1_CARD_IS_LOCKED       (1 << 25)                 /*!< R1: card locked status bit */
#define SDMMC_R1_LOCK_UNLOCK_FAILED   (1 << 24)                 /*!< R1: lock/unlock error status bit */
#define SDMMC_R1_COM_CRC_ERROR        (1 << 23)                 /*!< R1: CRC error status bit */
#define SDMMC_R1_ILLEGAL_COMMAND      (1 << 22)                 /*!< R1: illegal command status bit */
#define SDMMC_R1_CARD_ECC_FAILED      (1 << 21)                 /*!< R1: card ecc error status bit */
#define SDMMC_R1_CC_ERROR             (1 << 20)                 /*!< R1: internal card controller status bit */
#define SDMMC_R1_ERROR                (1 << 19)                 /*!< R1: a general or an unknown error status bit */
#define SDMMC_R1_CID_CSD_OVERWRITE    (1 << 16)                 /*!< R1: cid/csd overwrite status bit */
#define SDMMC_R1_WP_ERASE_SKIP        (1 << 15)                 /*!< R1: write protection erase skip status bit */
#define SDMMC_R1_CARD_ECC_DISABLED    (1 << 14)                 /*!< R1: card ecc disabled status bit */
#define SDMMC_R1_ERASE_RESET          (1 << 13)                 /*!< R1: erase reset status bit */
#define SDMMC_R1_STATUS(x)            ((uint32_t)(x) & 0xFFFFE000U) /*!< R1: status */
#define SDMMC_R1_READY_FOR_DATA       (1 << 8)                  /*!< R1: ready for data status bit */
#define SDMMC_R1_SWITCH_ERROR         (1 << 7)                  /*!< R1: switch error status bit */
#define SDMMC_R1_APP_CMD              (1 << 5)                  /*!< R1: application command enabled status bit */
#define SDMMC_R1_AKE_SEQ_ERROR        (1 << 3)                  /*!< R1: error in the sequence of the authentication process*/
#define SDMMC_R1_ERROR_BITS(x)  (uint32_t)((x) & \
                              (SDMMC_R1_OUT_OF_RANGE | \
                               SDMMC_R1_ADDRESS_ERROR | \
                               SDMMC_R1_BLK_LEN_ERROR | \
                               SDMMC_R1_ERASE_SEQ_ERROR | \
                               SDMMC_R1_ERASE_PARAM | \
                               SDMMC_R1_WP_VIOLATION | \
                               SDMMC_R1_CARD_IS_LOCKED | \
                               SDMMC_R1_LOCK_UNLOCK_FAILED | \
                               SDMMC_R1_COM_CRC_ERROR | \
                               SDMMC_R1_ILLEGAL_COMMAND | \
                               SDMMC_R1_CARD_ECC_FAILED | \
                               SDMMC_R1_CC_ERROR | \
                               SDMMC_R1_ERROR | \
                               SDMMC_R1_CID_CSD_OVERWRITE | \
                               SDMMC_R1_AKE_SEQ_ERROR)) /*!< Check error card status */

#define SDMMC_R1_CURRENT_STATE(x)     (((x) & 0x00001E00U) >> 9)/*!< R1: current state */
#define SDMMC_R1_STATE_IDLE           (0U)                      /*!< R1: current state: idle */
#define SDMMC_R1_STATE_READY          (1U)                      /*!< R1: current state: ready */
#define SDMMC_R1_STATE_IDENT          (2U)                      /*!< R1: current state: ident */
#define SDMMC_R1_STATE_STBY           (3U)                      /*!< R1: current state: stby */
#define SDMMC_R1_STATE_TRAN           (4U)                      /*!< R1: current state: tran */
#define SDMMC_R1_STATE_DATA           (5U)                      /*!< R1: current state: data */
#define SDMMC_R1_STATE_RCV            (6U)                      /*!< R1: current state: rcv */
#define SDMMC_R1_STATE_PRG            (7U)                      /*!< R1: current state: prg */
#define SDMMC_R1_STATE_DIS            (8U)                      /*!< R1: current state: dis */

/* SD card individual commands */
typedef enum _sd_cmd_t {
    kSdSendRelativeAddr = 3,        /*!< bcr                      R6 */
    kSdSwitchFunction = 6,                  /*!< adtc     [31] mode       R1 */
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

/* MMC individual command */
typedef enum _sdmmc_cmd_t {
    kSdmmcGoIdleState = 0,               /*!< bc */
    kSdmmcSendOpCond = 1,                /*!< bcr      [31:0] OCR      R3 */
    kSdmmcAllSendCid = 2,                /*!< bcr                      R2 */
    kSdmmcSetDsr = 4,                    /*!< bc       [31:16] RCA */
    kSdmmcSelectCard = 7,                /*!< ac       [31:16] RCA     R1b */
    kSdmmcSendCsd = 9,                   /*!< ac       [31:16] RCA     R2 */
    kSdmmcSendCid = 10,                  /*!< ac       [31:16] RCA     R2 */
    kSdmmcStopTransmission = 12,         /*!< ac       [31:16] RCA     R1b */
    kSdmmcSendStatus = 13,               /*!< ac       [31:16] RCA     R1 */
    kSdmmcGoInactiveState = 15,          /*!< ac       [31:16] RCA */

    kSdmmcSetBlockLen = 16,              /*!< ac       [31:0] block    R1 */
                                    /*!<          length */
    kSdmmcReadSingleBlock = 17,          /*!< adtc     [31:0] data     R1 */
                                    /*!<          address */
    kSdmmcReadMultipleBlock = 18,        /*!< adtc     [31:0] data     R1 */
                                    /*!<          address */
    kSdmmcSendTuningBlock = 19,          /*!< adtc     [31:0] all      R1 */
                                    /*!<          zero */
    kSdmmcSetBlockCount = 23,            /*!< ac       [31:0] block    R1 */
                                    /*!<          count */
    kSdmmcWriteBlock = 24,               /*!< adtc     [31:0] data     R1 */
                                    /*!<          address */
    kSdmmcWriteMultipleBlock = 25,       /*!< adtc     [31:0] data     R1 */
                                    /*!<          address */
    kSdmmcProgramCsd = 27,               /*!< adtc                     R1 */
    kSdmmcSetWriteProt = 28,             /*!< ac       [31:0] data     R1b */
                                    /*!<          address */
    kSdmmcClrWriteProt = 29,             /*!< ac       [31:0] data     R1b */
                                    /*!<          address */
    kSdmmcSendWriteProt = 30,            /*!< adtc     [31:0] write    R1b */
                                    /*!<          protect data */
                                    /*!<          address */
    kSdmmcErase = 38,                    /*!< ac                       R1 */
    kSdmmcLockUnlock = 42,               /*!< adtc     all zero        R1 */
    kSdmmcAppCmd = 55,                   /*!< ac       [31:16] RCA     R1 */
    kSdmmcGenCmd = 56,                   /*!< adtc     [0] RD/WR       R1 */
    kSdmmcReadOcr = 58,
} sdmmc_cmd_t;

typedef enum _mmc_cmd_t {           /*   type     argument        response */
    kMmcSetRelativeAddr = 3,        /*!< ac       [31:16] RCA     R1 */
    kMmcSleepAwake = 5,             /*!< ac       [31:16] RCA     R1b */
                                     /*!<          [15] flag */
    kMmcSwitch = 6,                 /*!< ac       [31:16] RCA     R1b */
    kMmcSendExtCsd = 8,             /*!< adtc                     R1 */
    kMmcReadDataUntilStop = 11,     /*!< adtc     [31:0] data     R1 */
                                     /*!<          address */
    kMmcBusTestRead = 14,           /*!< adtc                     R1 */
    kMmcWriteDataUntilStop = 20,    /*!< ac       [31:0] data    R1 */
                                     /*!<          address */
    kMmcProgramCid = 26,            /*!< adtc                     R1 */
    kMmcEraseGroupStart = 35,       /*!< ac       [31:0] data     R1 */
                                     /*!<          address */
    kMmcEraseGroupEnd = 36,         /*!< ac       [31:0] data     R1 */
                                     /*!<          address */
    kMmcFastIo = 39,                /*!< ac                       R4 */
    kMmcGoIrqState = 40,            /*!< bcr                      R5 */
} mmc_cmd_t;

#if defined(FSL_CARD_USING_BIG_ENDIAN)
#define swap_be32(x) (x)
#else
#define swap_be32(x) ((uint32_t)((((uint32_t)(x) & (uint32_t)(0xFF)) << 24)) | \
                                 (((uint32_t)(x) & (uint32_t)(0xFF00)) << 8) | \
                                 (((uint32_t)(x) & (uint32_t)(0xFF0000)) >> 8) | \
                                 (((uint32_t)(x) & (uint32_t)(0xFF000000U)) >> 24))
#endif

/* SD card individual register definition */
#define SD_SCR_BUS_WIDTHS_1BIT  (1 << 0)                /*!< card supports 1 bit mode */
#define SD_SCR_BUS_WIDTHS_4BIT  (1 << 2)                /*!< card supports 4 bit mode */

#define SD_CCC_BASIC            (1 << 0)                /*!< Card command class 0 */
#define SD_CCC_BLK_READ       (1 << 2)                /*!< Card command class 2 */
#define SD_CCC_BLK_WRITE      (1 << 4)                /*!< Card command class 4 */
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

typedef enum _sd_buswidth_t {
    kSdBusWidth1Bit = 0,            /*!< SD data bus width 1-bit mode */
    kSdBusWidth4Bit = 2,            /*!< SD data bus width 1-bit mode */
} sd_buswidth_t;

typedef enum _sd_switch_mode_t {
    kSdSwitchCheck = 0,             /*!< SD switch mode 0: check function */
    kSdSwitchSet = 1,               /*!< SD switch mode 1: set function */
} sd_switch_mode_t;

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

/* Read/write block length */
#define SD_BLOCK_LEN(x)                   ((uint32_t)(1 << (x)))

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

/* MMC card individual spec and register definition */
/*
 * These macros enables features of MMC driver:
 *
 * FSL_MMC_SUP_PRE_DEF_BLK_CNT
 *  - Enables PRE-DEFINED block count freature when read/write. 
 *
 */
//#define FSL_MMC_SUP_PRE_DEF_BLK_CNT /* MMC card support pre-defined block count*/

/*
 * These macros define attribute of MMC driver:
 * FSL_MMC_BLK_SIZE_POW_ON
 *  - MMC data block size after power on.
 */
#define FSL_MMC_BLK_SIZE_POW_ON      512


/*!
 * @brief MMC card classified as voltage range.
 */
typedef enum _mmc_type_as_voltage
{
    kMmcHighVoltageCard,            /*!< High voltage MMC card */
    kMmcDualVoltageCard,            /*!< Dual voltage MMC card */
} mmc_type_as_voltage_t;

/*!
 * @brief MMC card classified as density level.
 */
typedef enum _mmc_type_as_density {
    /* MMC card density is less than or equal 2GB,which is access as bytes */ 
    kMmcWithin2GB,      
    /* MMC card density is higher than 2GB, which is access as sector(512bytes) */
    kMmcHigher2GB,       
} mmc_type_as_density_t; 

/*!
 * @brief MMC OCR register fields.
 */
#define MMC_OCR_V170TO195_POS               7U           
#define MMC_OCR_V170TO195_MASK              0x00000080U  /*!< 1.70¨C1.95V */
#define MMC_OCR_V200TO260_POS               8U
#define MMC_OCR_V200TO260_MASK              0x00007F00U  /*!< 2.0¨C2.6V */
#define MMC_OCR_V270TO360_POS               15U
#define MMC_OCR_V270TO360_MASK              0x00FF8000U  /*!< 2.7¨C3.6V */
#define MMC_OCR_ACCESS_MODE_POS             29U
#define MMC_OCR_ACCESS_MODE_MASK            0x60000000U  /*!< Access mode */ 
#define MMC_OCR_BUSY_POS                    31U
#define MMC_OCR_BUSY_MASK                   0x80000000U  /*!< card power up status bit (busy) */

/*!
 * @brief MMC card access mode.
 */
typedef enum _mmc_access_mode
{
    kMmcAccessAsByte = 0,         /*!< The card should be accessed as byte */
    kMmcAccessAsSector = 2,       /*!< The card should be accessed as sector */
} mmc_access_mode_t;

/*!
 * @brief MMC card voltage range.
 */
typedef enum _mmc_voltage_range
{
    kMmcVoltage170to195 = 1,          /*!< Voltage range is 1.70V to 1.95V */
    kMmcVoltage270to360 = 511,        /*!< Voltage range is 2.70V to 3.60V */
} mmc_voltage_range_t;


/* Following is CSD register const */
/*!
 * @brief CSD structure version.
 */
typedef enum _mmc_csd_struc_ver
{
    kMmcCsdStrucVer10,         /*!< CSD version No. 1.0 */
    kMmcCsdStrucVer11,         /*!< CSD version No. 1.1 */
    kMmcCsdStrucVer12,         /*!< CSD version No. 1.2 */
    /*!< Version is coded in the CSD_STRUCTURE byte in the EXT_CSD register */
    kMmcCsdStrucVerInExtcsd,   
} mmc_csd_struc_ver_t;

/*!
 * @brief MMC card specification version.
 */
typedef enum _mmc_spec_ver
{
    kMmcSpecVer0,         /*!< Allocated by MMCA */
    kMmcSpecVer1,         /*!< Allocated by MMCA */
    kMmcSpecVer2,         /*!< Allocated by MMCA */
    kMmcSpecVer3,         /*!< Allocated by MMCA */
    kMmcSpecVer4,         /*!< Version 4.1/4.2/4.3 */
} mmc_spec_ver_t;

/*!< The mult in TRAN-SPEED field */
#define MMC_TRAN_SPEED_FREQ_UNIT_POS          0U   /*!< Frequency unit */
#define MMC_TRAN_SPEED_FREQ_UNIT_MASK         0x07U
#define MMC_TRAN_SPEED_MULT_FACTOR_POS        3U   /*!< Multiplier factor */
#define MMC_TRAN_SPEED_MULT_FACTOR_MASK       0x78U


/*!< Read the value of frequence unit in TRAN-SPEED field */
#define RD_MMC_CSD_TRAN_SPEED_FREQ_UNIT(CSD)  \
(((CSD.tranSpeed) & MMC_TRAN_SPEED_FREQ_UNIT_MASK) >> MMC_TRAN_SPEED_FREQ_UNIT_POS)
#define RD_MMC_CSD_TRAN_SPPED_MULT_FACTOR(CSD) \
(((CSD.tranSpeed) & MMC_TRAN_SPEED_MULT_FACTOR_MASK) >> MMC_TRAN_SPEED_MULT_FACTOR_POS) 



/* Following is EXT_CSD register const */
/*!
 * @brief MMC card EXT_CSD version.
 */
typedef enum _mmc_ext_csd_ver
{
    kMmcExtCsdVer10,    /*!< Revision 1.0 */
    kMmcExtCsdVer11,    /*!< Revision 1.1 */
    kMmcExtCsdVer12,    /*!< Revision 1.2 */
    kMmcExtCsdVer13,    /*!< Revision 1.3 */
} mmc_ext_csd_ver_t;

/*!
 * @brief EXT_CSD register access mode.
 */
typedef enum _mmc_ext_csd_access_mode
{
    /*!< The command set is changed according to the Cmd Setfield of the argument  */
    kMmcExtCsdCommandSet,    
    /*!< The bits in the pointed byte are set, according to the bits in the Value field   */
    kMmcExtCsdSetBits,   
    /*!< The bits in the pointed byte are cleared, according to the bits in the Value field */
    kMmcExtCsdClearBits,  
    /*!< The Value field is written into the pointed byte */
    kMmcExtCsdWriteBits,             
} mmc_ext_csd_access_mode_t;

/*!
 * @brief MMC card command set.
 */
typedef enum _mmc_cmd_set
{
    kMmcStandardMmc,        /*!< Standard MMC */
    kMmcCmdSet1,           
    kMmcCmdSet2,           
    kMmcCmdSet3,           
    kMmcCmdSet4,   
} mmc_cmd_set_t;

/*!
 * @brief Alternative boot support
 */
typedef enum _mmc_alter_boot
{
    kMmcNotSupAlterBoot,  /*!< Device does not support alternate boot method */
    kMmcSupAlterBoot,     /*!< Device supports alternate boot method. */
} mmc_alter_boot_t;

/*!
 * @brief MMC card power class used in PWR_CL_ff_vvv in EXT_CSD.
 */
typedef enum _mmc_power_class
{
    kMmcPowerClassLev0,     /*!< power class level1*/
    kMmcPowerClassLev1,     /*!< power class level2*/
    kMmcPowerClassLev2,
    kMmcPowerClassLev3,
    kMmcPowerClassLev4,
    kMmcPowerClassLev5,
    kMmcPowerClassLev6,
    kMmcPowerClassLev7,
    kMmcPowerClassLev8,
    kMmcPowerClassLev9,
    kMmcPowerClassLev10,    /*!< power class level11*/
} mmc_power_class_t;

/*!
 * @brief The only currently valid values of high speed frequence in CARD_TYPE in EXT_CSD are 0x01 and 0x03.
 */
typedef enum _mmc_high_speed_freq
{
    kMmcHighSpeedFreqAt26MHZ = 1,  /*!< High-Speed MultiMediaCard @ 26MHz */
    kMmcHighSpeedFreqAt52MHZ = 3,  /*!< High-Speed MultiMediaCard @ 52MHz */
} mmc_high_speed_freq_t;

/*!< The power class value bit mask when bus in 4 bit mode */
#define MMC_EXT_CSD_PWRCLFFVV_4BUS_MASK        (0x0FU)
/*!< The power class value bit mask when bus in 8 bit mode */
#define MMC_EXT_CSD_PWRCLFFVV_8BUS_MASK        (0xF0U)

/*!
 * @brief MMC card bus timing.
 */
typedef enum _mmc_bus_timing
{
    kMmcNonehighSpeedTiming,         /*!< MMC card using none high speed timing */
    kMmcHighSpeedTiming,             /*!< MMC card using high speed timing */
} mmc_bus_timing_t;

#define MMC_CARD_TYPE_HIGH_SPEED_26MHZ_POS      0U    /*!< High-Speed MultiMediaCard @ 26MHz */
#define MMC_CARD_TYPE_HIGH_SPEED_26MHZ_MASK     0x01U 
#define MMC_CARD_TYPE_HIGH_SPEED_52MHZ_POS      1U    /*!< High-Speed MultiMediaCard @ 52MHz */
#define MMC_CARD_TYPE_HIGH_SPEED_52MHZ_MASK     0x02U 

/*!
 * @brief Register EXT_CSD read/write function
 *
 */
/*!< Read the high speed frequence 26MHZ bit value */
#define RD_CARD_TYPE_HSFREQ_26MHZ(EXTCSD)  \
(((EXTCSD.cardType) & MMC_CARD_TYPE_HIGH_SPEED_26MHZ_MASK) >> MMC_CARD_TYPE_HIGH_SPEED_26MHZ_POS)
/*!< Read the high speed frequence 52MHZ bit value */
#define RD_CARD_TYPE_HSFREQ_52MHZ(EXTCSD) \
(((EXTCSD.cardType) & MMC_CARD_TYPE_HIGH_SPEED_52MHZ_MASK) >> MMC_CARD_TYPE_HIGH_SPEED_52MHZ_POS)

#define MMC_BUS_WIDTH_TYPE_NUM 3 /* The number of bus width type */
/*!
 * @brief MMC card bus width.
 */
typedef enum _mmc_bus_width
{
    kMmcBusWidth1b = 1,             /* MMC bus width is 1 bit */
    kMmcBusWidth4b = 4,             /* MMC bus width is 4 bits */
    kMmcBusWidth8b = 8,             /* MMC bus width is 8 bits */
} mmc_bus_width_t;

/*!
 * @brief MMC card boot partition enablement.
 */
typedef enum _mmc_boot_partition_enable
{
    kMmcBootNotEnabled           = 0U,      /*!< No boot acknowledge sent (default)*/
    kMmcBootPartition1Enabled    = 1U,      /*!< Boot partition 1 enabled for boot */
    kMmcBootPartition2Enabled    = 2U,      /*!< Boot partition 2 enabled for boot */
    kMmcBootUserAeraEnabled      = 7U,      /*!< User area enabled for boot */ 
} mmc_boot_partition_enable_t;

/*!
 * @breif MMC card boot partition to be accessed.
 */
typedef enum _mmc_access_boot_partition
{
    kMmcAccessBootPartitionNot,   /*!< No access to boot partition (default), normal partition */
    kMmcAccessBootPartition1,     /*!< R/W boot partition 1*/
    kMmcAccessBootPartition2,     /*!< R/W boot partition 2*/
} mmc_access_boot_partition_t;



/*!
 * @brief MMC card boot configuration definition.
 *
 * Following is BOOT_CONFIG definition
 * Bit[7] Reserved
 * Bit[5:3] BOOT_PARTITION_ENABLE
 * Bit[2:0] PARTITION_ACCESS 
 * Following is BOOT_BUS_WIDTH definition
 * Bit[7:3] Reserved
 * Bit[2] RESET_BOOT_BUS_WIDTH (non-volatile)
 * Bit[1:0] : BOOT_BUS_WIDTH (non-volatile)
 * Bit[7:3] Reserved
 * Bit[2] BOOT_ACK mask
 * Bit[1] BOOT_PARTITION_ENABLE mask
 * Bit[0] PARTITION_ACCESS mask
 * Following is BOOT_BUS_WIDTH definition
 * Bit[7:2] Reserved
 * Bit[1] RESET_BOOT_BUS_WIDTH (non-volatile) mask
 * Bit[0] : BOOT_BUS_WIDTH (non-volatile) mask
 */
#define MMC_BOOT_CONFIG_BOOT_PARTITION_ACCESS_POS     0U
#define MMC_BOOT_CONFIG_BOOT_PARTITION_ACCESS_MASK    0x00000007U
#define MMC_BOOT_CONFIG_BOOT_PARTITION_ENABLE_POS     3U
#define MMC_BOOT_CONFIG_BOOT_PARTITION_ENABLE_MASK    0x00000038U
#define MMC_BOOT_CONFIG_BOOT_ACK_POS                  6U
#define MMC_BOOT_CONFIG_BOOT_ACK_MASK                 0x00000040U
#define MMC_BOOT_BUS_WIDTH_WIDTH_POS                  8U
#define MMC_BOOT_BUS_WIDTH_WIDTH_MASK                 0x00000300U
#define MMC_BOOT_BUS_WIDTH_RESET_POS                  10U
#define MMC_BOOT_BUS_WIDTH_RESET_MASK                 0x00000400U  


/*!< The byte index of field in EXT_CSD */
#define MMC_EXT_CSD_POWER_CLASS_INDEX      187   /*!< The index of POWER_CLASS */
#define MMC_EXT_CSD_HS_TIMING_INDEX       185   /*!< The index of HS_TIMING */
#define MMC_EXT_CSD_BUS_WIDTH_INDEX        183
#define MMC_EXT_CSD_ERASE_GRP_DEF_INDEX    175
#define MMC_EXT_CSD_BOOT_CONFIG_INDEX      179
#define MMC_EXT_CSD_BOOT_BUS_WIDTH_INDEX   177

/*!
 * @brief MMC card operation.
 */
typedef struct _mmc_ext_csd_operation
{
    mmc_cmd_set_t cmdSet;
    uint8_t value;
    uint8_t indexOfByte;
    mmc_ext_csd_access_mode_t accessMode;
#define MMC_SWITCH_PARAM_CMD_SET_POS     0U               /* Command set bit position in SWITCH command parameter */
#define MMC_SWITCH_PARAM_CMD_SET_MASK    0x00000007U      
#define MMC_SWITCH_PARAM_VALUE_POS       8U               /*!< The index of the byte which will be operated, 
                                                   The Index field can contain any value from 0-255,
                                                   but only values 0-191 are valid values */
#define MMC_SWITCH_PARAM_VALUE_MASK      0x0000FF00U
#define MMC_SWITCH_PARAM_INDEX_OF_BYTE_POS  16U
#define MMC_SWITCH_PARAM_INDEX_OF_BYTE_MASK 0x00FF0000U
#define MMC_SWITCH_PARAM_ACCESS_MODE_POS  24U 
#define MMC_SWTICH_PARAM_ACCESS_MODE_MASK 0x03000000U
} mmc_ext_csd_operation_t; 

/*!< The length of CID, CSD, EXT_CSD register, unit of length is word(128), byte(512)) */
#define MMC_EXT_CSD_LEN_AS_WORD                  (128U)  
#define MMC_EXT_CSD_LEN_AS_BYTE                  (512U)

/*!
 * @brief The timeout value of sending and receive the response of the command.
 */
#define FSL_MMC_REQUEST_TIMEOUT        1000

/*!< The Minimum RSA value can be assigned to the card */
#define MMC_MINIMUM_RSA                     (2U)     
/*!< MMC card default RSA */    
#define MMC_DEFAULT_RSA   MMC_MINIMUM_RSA   

/*! @brief Bus test pattern when bus is at 8 bit width mode */
#define MMC_8BIT_BUS_TEST_PATTERN     (0x0000AA55)
/*! @brief The XOR result of test pattern when bus is at 8 bit width mode */
#define MMC_8BIT_BUS_PATTERN_XOR_RESULT  (0x0000FFFF)
/*!@brief Bus test pattern when bus is at 4 bit width mode */
#define MMC_4BIT_BUS_TEST_PATTERN     (0x0000005A)
/*!@brief The XOR result of test pattern when bus is at 4 bit width mode */
#define MMC_4BIT_BUS_PATTERN_XOR_RESULT  (0x000000FF) 
/*!@brief Bus test pattern when bus is at 1 bit width mode */
#define MMC_1BIT_BUS_TEST_PATTERN     (0x80)
/*!@brief The XOR result of test pattern when bus is at 1 bit width mode */
#define MMC_1BIT_BUS_PATTERN_XOR_RESULT  (0x000000C0)

/*!
 * @brief MMC card CID register fields.
 */
typedef struct _mmc_cid
{
    uint8_t mid;         /*!< Manufacturer ID */
    uint16_t oid;        /*!< OEM/Application ID */
#define MMC_PRODUCT_NAME_LEN 6     /*!< MMC product name length*/
    uint8_t pnm[MMC_PRODUCT_NAME_LEN];  /*!< Product name */
    uint8_t prv;        /*!< Product revision */
    uint32_t psn;        /*!< Product serial number */
    uint8_t mdt;       /*!< Manufacturing date */
} mmc_cid_t;

/*!
 * @brief MMC card CSD register fields.
 */
typedef struct _mmc_csd
{
    uint8_t csdStructVer;         /*!< CSD structure [127:126]*/
    uint8_t sysSpecVer;           /*!< System specification version */
    uint8_t taac;                 /*!< Data read access-time 1 */
    uint8_t nsac;                 /*!< Data read access-time 2 in CLK cycles (NSAC*100) */
    uint8_t tranSpeed;            /*!< Max. bus clock frequency */
    uint16_t ccc;                 /*!< card command classes */
    uint8_t readBlkLen;           /*!< Max. read data block length */
/*!< Partial blocks for read allowed [79:79]*/
#define MMC_CSD_READ_BL_PARTIAL           (1<<0)   
/*!< Write block misalignment [78:78]*/
#define MMC_CSD_WRITE_BLK_MISALIGN        (1<<1) 
/*!< Read block misalignment [77:77]*/
#define MMC_CSD_READ_BLK_MISALIGN         (1<<2)  
/*!< DSR implemented [76:76] */
#define MMC_CSD_DSR_IMP                   (1<<3)  
/*!< Write protect group enabled [31:31] */
#define MMC_CSD_WP_GRP_ENABLED            (1<<4)  
/*!< Partial blocks for write allowed [21:21]*/
#define MMC_CSD_WRITE_BL_PARTIAL          (1<<5) 
/*!< Content protection application [16:16]*/
#define MMC_CSD_CONTENT_PROT_APP          (1<<6)   
/*!< File format group [15:15]*/
#define MMC_CSD_FILE_FORMAT_GROUP         (1<<7)    
/*!< Copy flag [14:14]*/
#define MMC_CSD_COPY                      (1<<8)   
/*!< Permanent write protection [13:13]*/
#define MMC_CSD_PERM_WRITE_PROTECT        (1<<9)
/*!< Temporary write protection [12:12]*/
#define MMC_CSD_TMP_WRITE_PROTECT         (1<<10)
    uint16_t flags;                /*!< Contain above flags */
    uint16_t c_size;               /*!< Device size */
    uint8_t vdd_r_cur_min;         /*!< Max. read current @ VDD min */
    uint8_t vdd_r_cur_max;         /*!< Max. read current @ VDD max */
    uint8_t vdd_w_cur_min;         /*!< Max. write current @ VDD min */
    uint8_t vdd_w_cur_max;         /*!< Max. write current @ VDD max */
    uint8_t c_size_mult;           /*!< Device size multiplier */
    uint8_t eraseGrpSize;          /*!< Erase group size */
    uint8_t eraseGrpSizeMult;      /*!< Erase group size multiplier */
    uint8_t wpGrpSize;             /*!< Write protect group size */  
    uint8_t defaultEcc;            /*!< Manufacturer default ECC */
    uint8_t writeSpeedFactor;      /*!< Write speed factor */
    uint8_t maxWriteBlkLen;        /*!< Max. write data block length */
    uint8_t fileFormat;            /*!< File format */
    uint8_t eccCode;               /*!< ECC code */  
} mmc_csd_t;

/*!
* @brief MMC card EXT_CSD register fields(unit:byte).
 */
typedef struct _mmc_ext_csd
{
    uint8_t supportedCmdSet;             /*!< Supported Command Sets, [504]*/
    /* Following fields only has effective value in MMC spec V4.3 in MMC mode 
    (card spec version can be read from sysSpecVer field in CSD.): 
    from bootInfo to cardType */
    uint8_t bootInfo;                    /*!< Boot information, [228] */
    uint8_t bootSizeMult;                /*!< Boot partition size, [226] */
    uint8_t accessSize;                  /*!< Access size, [225] */
    uint8_t HC_ERASE_GRP_SIZE;           /*!< High-capacity erase unit size, [224] */
    uint8_t ERASE_TIMEOUT_MULT;          /*!< High-capacity erase timeout, [223] */
    uint8_t reliableWriteSectorCount;    /*!< Reliable write sector count, [222] */
    uint8_t hc_wp_grp_size;              /*!< High-capacity write protect group size, [221] */
    uint8_t sleepCurrentVCC;             /*!< Sleep current (VCC), [220] */
    uint8_t sleepCurrentVCCQ;            /*!< Sleep current (VCCQ), [219] */    
    uint8_t slpAwkTimeout;               /*!< Sleep/awake timeout, [217] */
    uint32_t sectorCount;                /*!< Sector Count, [215:212] */
    /* Following fields only has effective value in MMC mode:
    from MIN_PERF_W_8_52 to PWR_CL_52_195, powerCls, HS_TIMING, busWidth */
    uint8_t MIN_PERF_W_8_52;             /*!< Minimum Write Performance for 8bit @52MHz, [210] */
    uint8_t MIN_PERF_R_8_52;             /*!< Minimum Read Performance for 8bit @52MHz, [209] */
    uint8_t MIN_PERF_W_8_26_4_52;        /*! Minimum Write Performance for 8bit @26MHz/ 4bit @52MHz, [208]*/
    uint8_t MIN_PERF_R_8_26_4_52;        /*!< Minimum read Performance for 8bit @26MHz/ 4bit @52MHz, [207] */
    uint8_t MIN_PERF_W_4_26;             /*!< Minimum Write Performance for 4bit @26MHz, [206] */
    uint8_t MIN_PERF_R_4_26;             /*!< Minimum Read Performance for 4bit @26MHz, [205] */
    uint8_t PWR_CL_26_360;               /*!< Power Class for 26MHz @ 3.6V, [203] */
    uint8_t PWR_CL_52_360;               /*!< Power Class for 52MHz @ 3.6V, [202] */
    uint8_t PWR_CL_26_195;               /*!< Power Class for 26MHz @ 1.95V, [201] */
    uint8_t PWR_CL_52_195;               /*< Power Class for 52MHz @ 1.95V, [200] */
    uint8_t cardType;                    /*!< Card Type, [196] */
    uint8_t csdStrucVer;                 /*!< CSD structure version, [194] */
    uint8_t extCsdVer;                   /*!< Extended CSD revision, [192] */
    uint8_t cmdSet;                      /*!< Command set, [191] */
    uint8_t cmdSetRev;                   /*!< Command set revision, [189] */
    uint8_t powerClass;                  /*!< Power class, [187] */
    uint8_t highSpeedTiming;             /*!< High-speed interface timing, [185] */
    uint8_t busWidth;                    /*!< Bus width mode, [183] */
    uint8_t erasedMemCnt;                /*!< Erased memory content, [181] */
    /* Following fields only has effective value in EMMV V4.3 in MMC mode: 
    from bootConfig to ERASE_GROUP_DEF */    
    uint8_t bootConfig;                  /*!< Boot configuration, [179] */
    uint8_t bootBusWidth;                /*!< Boot bus width, [177] */
    uint8_t ERASE_GROUP_DEF;             /*!< High-density erase group definition, [175] */
} mmc_ext_csd_t;

#endif  /* __sdmmc_H__ */

