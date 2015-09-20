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

/*!
 * @addtogroup card_specification_data_type
 * @{
 */

/* SDMMC card initialization clock frequ*/
#define SDMMC_CLK_400KHZ        (400000U)
/* SD card high speed clock frequence. */
#define SD_CLK_25MHZ            (25000000U)
#define SD_CLK_50MHZ            (50000000U)
/* MMC card high speed clock frequence. */
#define MMC_CLK_26MHZ           (26000000U)
#define MMC_CLK_52MHZ           (52000000U)

/*! @brief Card status bit in R1 */
typedef enum _sdmmc_r1_card_status
{
    kSDMMC_R1OutOfRange            = (1U << 31U),     /*!< R1: out of range status bit */
    kSDMMC_R1AddressError          = (1U << 30U),     /*!< R1: address error status bit */
    kSDMMC_R1BlockLengthError      = (1U << 29U),     /*!< R1: block length error status bit */
    kSDMMC_R1EraseSeqError         = (1U << 28U),     /*!< R1: erase sequence error status bit */
    kSDMMC_R1EraseParam            = (1U << 27U),     /*!< R1: erase parameter error status bit */
    kSDMMC_R1WPViolation           = (1U << 26U),     /*!< R1: write protection violation status bit */
    kSDMMC_R1CardIsLocked          = (1U << 25U),     /*!< R1: card locked status bit */
    kSDMMC_R1LockUnlockFailed      = (1U << 24U),     /*!< R1: lock/unlock error status bit */
    kSDMMC_R1ComCrcError           = (1U << 23U),     /*!< R1: CRC error status bit */
    kSDMMC_R1IllegalCommand        = (1U << 22U),     /*!< R1: illegal command status bit */
    kSDMMC_R1CardEccFailed         = (1U << 21U),     /*!< R1: card ecc error status bit */
    kSDMMC_R1CCError               = (1U << 20U),     /*!< R1: internal card controller status bit */
    kSDMMC_R1Error                 = (1U << 19U),     /*!< R1: a general or an unknown error status bit */
    kSDMMC_R1CidCsdOverwrite       = (1U << 16U),     /*!< R1: cid/csd overwrite status bit */
    kSDMMC_R1WPEraseSkip           = (1U << 15U),     /*!< R1: write protection erase skip status bit */
    kSDMMC_R1CardEccDisabled       = (1U << 14U),     /*!< R1: card ecc disabled status bit */
    kSDMMC_R1EraseReset            = (1U << 13U),     /*!< R1: erase reset status bit */
    kSDMMC_R1ReadyForData          = (1U << 8U),      /*!< R1: ready for data status bit */
    kSDMMC_R1SwitchError           = (1U << 7U),      /*!< R1: switch error status bit */
    kSDMMC_R1AppCmd                = (1U << 5U),      /*!< R1: application command enabled status bit */
    kSDMMC_R1AkeSeqError           = (1U << 3U),      /*!< R1: error in the sequence of the authentication process*/
} sdmmc_r1_card_status_t;

#define SDMMC_R1_ERROR_BITS(x)  (uint32_t)((x) & \
                              (kSDMMC_R1OutOfRange | \
                               kSDMMC_R1AddressError | \
                               kSDMMC_R1BlockLengthError | \
                               kSDMMC_R1EraseSeqError | \
                               kSDMMC_R1EraseParam | \
                               kSDMMC_R1WPViolation | \
                               kSDMMC_R1CardIsLocked | \
                               kSDMMC_R1LockUnlockFailed | \
                               kSDMMC_R1ComCrcError | \
                               kSDMMC_R1IllegalCommand | \
                               kSDMMC_R1CardEccFailed | \
                               kSDMMC_R1CCError | \
                               kSDMMC_R1Error | \
                               kSDMMC_R1CidCsdOverwrite | \
                               kSDMMC_R1AkeSeqError)) /*!< Check error card status */


#define SDMMC_R1_CURRENT_STATE(x)     (((x) & 0x00001E00U) >> 9U)/*!< R1: current state */

/*! @brief CURRENT_STATE filed in R1 */
typedef enum _sdmmc_r1_current_state
{
    kSDMMC_R1StateIdle            = 0U,      /*!< R1: current state: idle */
    kSDMMC_R1StateReady           = 1U,      /*!< R1: current state: ready */
    kSDMMC_R1StateIdent           = 2U,      /*!< R1: current state: identification */
    kSDMMC_R1StateStandby         = 3U,      /*!< R1: current state: standby */
    kSDMMC_R1StateTransfer        = 4U,      /*!< R1: current state: transfer */
    kSDMMC_R1StateSendData        = 5U,      /*!< R1: current state: sending data */
    kSDMMC_R1StateReceiveData     = 6U,      /*!< R1: current state: receiving data */
    kSDMMC_R1StateProgram         = 7U,      /*!< R1: current state: programming */
    kSDMMC_R1StateDisconnect      = 8U,      /*!< R1: current state: disconnect */
} sdmmc_r1_current_state_t;

/*! @brief Error bit in SPI mode R1 */
typedef enum _sdspi_r1_error
{
    kSDSPI_R1InIdleState        = (1U << 0U),   /*!< In idle state */
    kSDSPI_R1EraseReset         = (1U << 1U),   /*!< Erase reset */
    kSDSPI_R1IllegalCommand     = (1U << 2U),   /*!< Illegal command */
    kSDSPI_R1ComCrcError        = (1U << 3U),   /*!< Com crc error */
    kSDSPI_R1EraseSeqError      = (1U << 4U),   /*!< Erase sequence error */
    kSDSPI_R1AddrError          = (1U << 5U),   /*!< Address error */
    kSDSPI_R1ParamError         = (1U << 6U),   /*!< Parameter error */
} sdspi_r1_error_t;

/*! @brief Error bit in SPI mode R2 */
typedef enum _sdspi_r2_error
{
    kSDSPI_R2CardLocked          = (1U << 0U),      /*!< Card is locked */
    kSDSPI_R2WPEraseSkip         = (1U << 1U),      /*!< Write protect erase skip */
    kSDSPI_R2LockUnlockFailed    = (1U << 1U),      /*!< Lock/unlock command failed */
    kSDSPI_R2Error               = (1U << 2U),      /*!< Unknown error */
    kSDSPI_R2CCError             = (1U << 3U),      /*!< Card controller error */
    kSDSPI_R2CardEccFailed       = (1U << 4U),      /*!< Card ecc failed */
    kSDSPI_R2WPViolation         = (1U << 5U),      /*!< Write protect violation */
    kSDSPI_R2EraseParam          = (1U << 6U),      /*!< Erase parameter error */
    kSDSPI_R2OutOfRange          = (1U << 7U),      /*!< Out of range */
    kSDSPI_R2CsdOverwrite        = (1U << 7U),      /*!< CSD overwrite */
} sdspi_r2_error_t;

#define SDSPI_R7_VERSION_SHIFT   28U
#define SDSPI_R7_VERSION_MASK    0xFU
#define SDSPI_R7_VOLT_SHIFT      8U
#define SDSPI_R7_VOLT_MASK       0xFU
#define SDSPI_R7_VOLT_27_36      (0x1U << SDSPI_R7_VOLT_SHIFT)
#define SDSPI_R7_ECHO_SHIFT      0U
#define SDSPI_R7_ECHO_MASK       0xFFU

/* Data error token mask */
#define SDSPI_DATA_ERROR_TOKEN_MASK           0xFU
/*! @brief Data Error Token mask */
typedef enum _sdspi_data_error_token
{
    kSDSPI_DataErrorTokenError            = (1U << 0U),  /*!< Data error */
    kSDSPI_DataErrorTokenCCError          = (1U << 1U),  /*!< CC error */
    kSDSPI_DataErrorTokenCardEccFailed    = (1U << 2U),  /*!< Card ecc error */
    kSDSPI_DataErrorTokenOutOfRange       = (1U << 3U),  /*!< Out of range */
} sdspi_data_error_token_t;

/*! @brief Data Token */
typedef enum _sdspi_data_token
{
    kSDSPI_DataTokenBlockRead           = 0xFEU, /*!< Single block read, multiple block read */
    kSDSPI_DataTokenSingleBlockWrite    = 0xFEU, /*!< Single block write */
    kSDSPI_DataTokenMultipleBlockWrite  = 0xFCU, /*!< Multiple block write */
    kSDSPI_DataTokenStopTransfer        = 0xFDU, /*!< Stop transmission */
} sdspi_data_token_t;

/* Data Response Token mask */
#define SDSPI_DATA_RESPONSE_TOKEN_MASK             (0x1FU)        /*!< Mask for data response bits */
/*! @brief Data Response Token */
typedef enum _sdspi_data_response_token
{
    kSDSPI_DataResponseTokenAccepted     = 0x05U,   /*!< Data accepted */
    kSDSPI_DataResponseTokenCrcError     = 0x0BU,   /*!< Data rejected due to CRC error */
    kSDSPI_DataResponseTokenWriteError   = 0x0DU,   /*!< Data rejected due to write error */ 
} sdspi_data_response_token_t;

/*! @brief SD card individual commands */
typedef enum _sd_cmd_t 
{
    kSD_SendRelativeAddr  = 3U,      /*!< Send Relative Address */
    kSD_Switch            = 6U,      /*!< Switch Function */
    kSD_SendIfCond        = 8U,      /*!< Send Interface Condition */
    kSD_VoltageSwitch     = 11U,     /*!< Voltage Switch */
    kSD_SpeedClassControl = 20U,     /*!< Speed Class control */
    kSD_EraseWrBlkStart   = 32U,     /*!< Write Block Start */
    kSD_EraseWrBlkEnd     = 33U,     /*!< Write Block End */
} sd_cmd_t;

/*! @brief SD/MMC card common commands */
typedef enum _sd_acmd_t 
{
    kSD_AppSetBusWdith        = 6U,  /*!< Set Bus Width */
    kSD_AppStatus             = 13U, /*!< Send SD status */
    kSD_AppSendNumWrBlocks    = 22U, /*!< Send Number Of Written Blocks */
    kSD_AppSetWrBlkEraseCount = 23U, /*!< Set Write Block Erase Count */
    kSD_AppSendOpCond         = 41U, /*!< Send Operation Condition */
    kSD_AppSetClrCardDetect   = 42U, /*!< Set Connnect/Disconnect pull up on detect pin */
    kSD_AppSendScr            = 51U, /*!< Send Scr */
} sd_acmd_t;

/*! @brief MMC individual command */
typedef enum _sdmmc_cmd_t 
{
    kSDMMC_GoIdleState        = 0U,   /*!< Go Idle State */
    kSDMMC_AllSendCid         = 2U,   /*!< All Send Cid */
    kSDMMC_SetDsr             = 4U,   /*!< Set Dsr */
    kSDMMC_SelectCard         = 7U,   /*!< Select Card */
    kSDMMC_SendCsd            = 9U,   /*!< Send Csd */
    kSDMMC_SendCid            = 10U,  /*!< Send Cid */
    kSDMMC_StopTransmission   = 12U,  /*!< Stop Transmission */
    kSDMMC_SendStatus         = 13U,  /*!< Send Status */
    kSDMMC_GoInactiveState    = 15U,  /*!< Go Inactive State */
    kSDMMC_SetBlockLen        = 16U,  /*!< Set Block Length */
    kSDMMC_ReadSingleBlock    = 17U,  /*!< Read Single Block */
    kSDMMC_ReadMultipleBlock  = 18U,  /*!< Read Multiple Block */
    kSDMMC_SendTuningBlock    = 19U,  /*!< Send Tuning Block */
    kSDMMC_SetBlockCount      = 23U,  /*!< Set Block Count */
    kSDMMC_WriteBlock         = 24U,  /*!< Write Block */
    kSDMMC_WriteMultipleBlock = 25U,  /*!< Write Multiple Block */
    kSDMMC_ProgramCsd         = 27U,  /*!< Program Csd */
    kSDMMC_SetWriteProt       = 28U,  /*!< Set Write Protect */
    kSDMMC_ClrWriteProt       = 29U,  /*!< Clear Write Protect */
    kSDMMC_SendWriteProt      = 30U,  /*!< Send Write Protect */
    kSDMMC_Erase              = 38U,  /*!< Erase */
    kSDMMC_LockUnlock         = 42U,  /*!< Lock Unlock */
    kSDMMC_AppCmd             = 55U,  /*!< Send Application Command */
    kSDMMC_GenCmd             = 56U,  /*!< General Purpose Command */
    kSDMMC_ReadOcr            = 58U,  /*!< Read Ocr */
} sdmmc_cmd_t;

/*! @brief MMC card individual commands */
typedef enum _mmc_cmd_t 
{           
    kMMC_SendOpCond         = 1U,   /*!< Send Operation Condition */
    kMMC_SetRelativeAddr    = 3U,   /*!< Set Relative Address */
    kMMC_SleepAwake         = 5U,   /*!< Sleep Awake */
    kMMC_Switch             = 6U,   /*!< Switch */
    kMMC_SendExtCsd         = 8U,   /*!< Send Ext_Csd */
    kMMC_ReadDataUntilStop  = 11U,  /*!< Read Data Until Stop */
    kMMC_BusTestRead        = 14U,  /*!< Test Read */
    kMMC_WriteDataUntilStop = 20U,  /*!< Write Data Until Stop */
    kMMC_ProgramCid         = 26U,  /*!< Program Cid */
    kMMC_EraseGroupStart    = 35U,  /*!< Erase Group Start */
    kMMC_EraseGroupEnd      = 36U,  /*!< Erase Group End */
    kMMC_FastIo             = 39U,  /*!< Fast IO */
    kMMC_GoIrqState         = 40U,  /*!< Go Irq State */
} mmc_cmd_t;

/*! @brief SD card individual register definition */
typedef enum _sdmmc_cmd_class
{
    kSDMMC_CmdClassBasic          = (1U << 0U),  /*!< Card command class 0 */
    kSDMMC_CmdClassBlockRead      = (1U << 2U),  /*!< Card command class 2 */
    kSDMMC_CmdClassBlockWrite     = (1U << 4U),  /*!< Card command class 4 */
    kSDMMC_CmdClassErase          = (1U << 5U),  /*!< Card command class 5 */
    kSDMMC_CmdClassWriteProtect   = (1U << 6U),  /*!< Card command class 6 */
    kSDMMC_CmdClassLockCard       = (1U << 7U),  /*!< Card command class 7 */
    kSDMMC_CmdClassAppSpec        = (1U << 8U),  /*!< Card command class 8 */
    kSDMMC_CmdClassIoMode         = (1U << 9U),  /*!< Card command class 9 */
    kSDMMC_CmdClassSwitch         = (1U << 10U), /*!< Card command class 10 */
} sdmmc_cmd_class_t;

/*! @brief OCR register in SD */
typedef enum _sd_ocr
{ 
    kSD_OcrPowerUpBusy     = (1U << 31U),       /*!< Power up busy status */
    kSD_OcrHCS             = (1U << 30U),       /*!< Card capacity status */
    kSD_OcrCCS             = kSD_OcrHCS,        /*!< Card capacity status */
    kSD_OcrXpc             = (1U << 28U),       /*!< SDXC power control */  
    kSD_OcrS18R            = (1U << 24U),       /*!< Switch to 1.8V request */
    kSD_OcrS18A            = SD_OCR_S18R,       /*!< Switch to 1.8V accepted */ 
    kSD_OcrVdd27_28        = (1U << 15U),       /*!< VDD 2.7-2.8 */
    kSD_OcrVdd28_29        = (1U << 16U),       /*!< VDD 2.8-2.9 */
    kSD_OcrVdd29_30        = (1U << 17U),       /*!< VDD 2.9-3.0 */
    kSD_OcrVdd30_31        = (1U << 18U),       /*!< VDD 2.9-3.0 */
    kSD_OcrVdd31_32        = (1U << 19U),       /*!< VDD 3.0-3.1 */
    kSD_OcrVdd32_33        = (1U << 20U),       /*!< VDD 3.1-3.2 */
    kSD_OcrVdd33_34        = (1U << 21U),       /*!< VDD 3.2-3.3 */
    kSD_OcrVdd34_35        = (1U << 22U),       /*!< VDD 3.3-3.4 */
    kSD_OcrVdd35_36        = (1U << 23U),       /*!< VDD 3.4-3.5 */
} sd_ocr_t;

/*! @brief SD Specification version number */
typedef enum _sd_spec_version
{
    kSD_SpecVersion1_0        = (1U << 0U),        /*!< SD card version 1.0-1.01 */
    kSD_SpecVersion1_1        = (1U << 1U),        /*!< SD card version 1.10 */
    kSD_SpecVersion2_0        = (1U << 2U),        /*!< SD card version 2.00 */
    kSD_SpecVersion3_0        = (1U << 3U),        /*!< SD card version 3.0 */
} sd_spec_version_t;

/*! @brief SD bus width */
typedef enum _sd_bus_width_t
{
    kSD_BusWidth1Bit = (1U << 0U),            /*!< SD data bus width 1-bit mode */
    kSD_BusWidth4Bit = (1U << 2U),            /*!< SD data bus width 1-bit mode */
} sd_bus_width_t;

/*! @brief SD switch mode */
typedef enum _sd_switch_mode_t
 {
    kSD_SwitchCheck = 0U,             /*!< SD switch mode 0: check function */
    kSD_SwitchSet   = 1U              /*!< SD switch mode 1: set function */
} sd_switch_mode_t;

/*! @brief SD card CSD register flag */
typedef enum _sd_csd_flag
{
    kSD_CsdReadBlockPartial          = (1U << 0U),  /*!< Partial blocks for read allowed [79:79] */
    kSD_CsdWriteBlockMisalign        = (1U << 1U),  /*!< Write block misalignment [78:78] */
    kSD_CsdReadBlockMisalign         = (1U << 2U),  /*!< Read block misalignment [77:77] */
    kSD_CsdDsrImplemented            = (1U << 3U),  /*!< DSR implemented [76:76] */
    kSD_CsdEraseBlockEnabled         = (1U << 4U),  /*!< Erase single block enabled [46:46] */
    kSD_CsdWPGroupEnabled            = (1U << 5U),  /*!< Write protect group enabled [31:31] */
    kSD_CsdWriteBlockPartial         = (1U << 6U),  /*!< Partial blocks for write allowed [21:21] */
    kSD_CsdFileFormatGroup           = (1U << 7U),  /*!< File format group [15:15] */
    kSD_CsdCopy                      = (1U << 8U),  /*!< Copy flag [14:14] */
    kSD_CsdPermWriteProtect          = (1U << 9U),  /*!< Permanent write protection [13:13] */
    kSD_CsdTmpWriteProtect           = (1U << 10U), /*!< Temporary write protection [12:12] */
} sd_csd_flag_t;
/*! @brief SD card CSD register */
typedef struct SDCsd 
{
    uint8_t csdStructure;           /*!< CSD structure [127:126] */
    uint8_t taac;                   /*!< Data read access-time-1 [119:112] */
    uint8_t nsac;                   /*!< Data read access-time-2 in clock cycles (NSAC*100) [111:104] */
    uint8_t tranSpeed;              /*!< Maximum data transfer rate [103:96] */
    uint16_t ccc;                   /*!< Card command classes [95:84] */
    uint8_t readBlkLen;             /*!< Maximum read data block length [83:80] */
    uint16_t flags;                 /*!< Card flags */
    uint32_t cSize;             /*!< Device size [73:62] */
/* Following fields from 'vddRCurrMin' to 'cSizeMult' exist in CSD version 1 */
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

#define SD_TRAN_SPEED_RATE_UNIT_MASK 0x07U
#define SD_TRAN_SPEED_RATE_UNIT_SHIFT  0U
#define SD_TRAN_SPEED_TIME_VALUE_MASK         0x78U
#define SD_TRAN_SPEED_TIME_VALUE_SHIFT          2U
/*!< Read the value of frequence unit in TRAN-SPEED field */
#define SD_RD_TRAN_SPEED_RATE_UNIT(CSD)  \
(((CSD.tranSpeed) & SD_TRAN_SPEED_RATE_UNIT_MASK) >> SD_TRAN_SPEED_RATE_UNIT_SHIFT)
#define SD_RD_TRAN_SPEED_MULT(CSD) \
(((CSD.tranSpeed) & SD_TRAN_SPEED_TIME_VALUE_MASK) >> SD_TRAN_SPEED_TIME_VALUE_SHIFT) 


/*! @brief SD card SCR register flag */
typedef enum _sd_scr_flag
{
    kSD_ScrDataStatAfterErase     = (1U << 0U),   /*!< Data status after erases [55:55] */
    kSD_ScrSdSpec3                = (1U << 1U),   /*!< Spec. version 3.00 or higher [47:47]*/
} sd_scr_flag_t;

/*! @brief SD card SCR register */
typedef struct SDScr {
    uint8_t scrStructure;       /*!< SCR Structure [63:60] */
    uint8_t sdSpec;             /*!< SD memory card spec. version [59:56] */
    uint16_t flags;             /*!< SCR flags */
    uint8_t sdSecurity;         /*!< CPRM security support [54:52] */
    uint8_t sdBusWidths;        /*!< Data bus widths supported [51:48] */
    uint8_t exSecurity;         /*!< Extended security support [46:43] */
    uint8_t cmdSupport;         /*!< Command support bits [33:32] */
    uint32_t reservedForMan;    /*!< reserved for manufacturer usage [31:0] */
} sd_scr_t;

/*! @brief SD card CID register */
typedef struct SDCid {
    uint8_t mid;                    /*!< Manufacturer ID [127:120] */
    uint16_t oid;                   /*!< OEM/Application ID [119:104] */
    uint8_t pnm[6];                 /*!< Product name [103:64] */
    uint8_t prv;                    /*!< Product revision [63:56] */
    uint32_t psn;                   /*!< Product serial number [55:24] */
    uint16_t mdt;                   /*!< Manufacturing date [19:8] */
} sd_cid_t;

/* MMC card individual spec and register definition */

/*!
 * @brief MMC card classified as volt range.
 */
typedef enum _mmc_type_as_volt
{
    kMMC_HighVoltageCard = 0U,            /*!< High volt MMC card */
    kMMC_DualVoltageCard = 1U             /*!< Dual volt MMC card */
} mmc_type_as_volt_t;

/*!
 * @brief MMC card classified as density level.
 */
typedef enum _mmc_type_as_density
{
    /* MMC card density is less than or equal 2GB,which is access as bytes */ 
    kMMC_Within2GB = 0U,      
    /* MMC card density is higher than 2GB, which is access as sector(512bytes) */
    kMMC_Higher2GB = 1U       
} mmc_type_as_density_t; 

/*!
 * @brief MMC OCR register fields.
 */
#define MMC_OCR_V170TO195_SHIFT               7U           
#define MMC_OCR_V170TO195_MASK              0x00000080U  /*!< 1.70¨C1.95V */
#define MMC_OCR_V200TO260_SHIFT               8U
#define MMC_OCR_V200TO260_MASK              0x00007F00U  /*!< 2.0¨C2.6V */
#define MMC_OCR_V270TO360_SHIFT               15U
#define MMC_OCR_V270TO360_MASK              0x00FF8000U  /*!< 2.7¨C3.6V */
#define MMC_OCR_ACCESS_MODE_SHIFT             29U
#define MMC_OCR_ACCESS_MODE_MASK            0x60000000U  /*!< Access mode */ 
#define MMC_OCR_BUSY_SHIFT                    31U
#define MMC_OCR_BUSY_MASK                   0x80000000U  /*!< card power up status bit (busy) */

/*!
 * @brief MMC card access mode.
 */
typedef enum _mmc_access_mode
{
    kMMC_AccessAsByte   = 0U,         /*!< The card should be accessed as byte */
    kMMC_AccessAsSector = 2U          /*!< The card should be accessed as sector */
} mmc_access_mode_t;

/*!
 * @brief MMC card volt range.
 */
typedef enum _mmc_volt_range
{
    kMMC_Volt170to195 = 1U,          /*!< Voltage range is 1.70V to 1.95V */
    kMMC_Volt270to360 = 511U,        /*!< Voltage range is 2.70V to 3.60V */
} mmc_volt_range_t;

/*!
 * @brief CSD structure version.
 */
typedef enum _mmc_csd_struc_ver
{
    kMMC_CsdStrucVer10       = 0U,   /*!< CSD version No. 1.0 */
    kMMC_CsdStrucVer11       = 1U,   /*!< CSD version No. 1.1 */
    kMMC_CsdStrucVer12       = 2U,   /*!< CSD version No. 1.2 */
    kMMC_CsdStrucVerInExtcsd = 3U    /*!< Version coded in EXT_CSD */   
} mmc_csd_struc_ver_t;

/*!
 * @brief MMC card specification version.
 */
typedef enum _mmc_spec_ver
{
    kMMC_SpecVer0 = 0U,         /*!< Allocated by MMCA */
    kMMC_SpecVer1 = 1U,         /*!< Allocated by MMCA */
    kMMC_SpecVer2 = 2U,         /*!< Allocated by MMCA */
    kMMC_SpecVer3 = 3U,         /*!< Allocated by MMCA */
    kMMC_SpecVer4 = 4U          /*!< Version 4.1/4.2/4.3 */
} mmc_spec_ver_t;

/*!< The mult in TRAN-SPEED field */
#define MMC_TRAN_SPEED_FREQ_UNIT_SHIFT          0U   /*!< Frequency unit */
#define MMC_TRAN_SPEED_FREQ_UNIT_MASK         0x07U
#define MMC_TRAN_SPEED_MULT_SHIFT        3U   /*!< Multiplier factor */
#define MMC_TRAN_SPEED_MULT_MASK       0x78U

/*!< Read the value of frequence unit in TRAN-SPEED field */
#define RD_MMC_TRAN_SPEED_FREQ_UNIT(CSD)  \
(((CSD.tranSpeed) & MMC_TRAN_SPEED_FREQ_UNIT_MASK) >> MMC_TRAN_SPEED_FREQ_UNIT_SHIFT)
#define RD_MMC_TRAN_SPEED_MULT(CSD) \
(((CSD.tranSpeed) & MMC_TRAN_SPEED_MULT_MASK) >> MMC_TRAN_SPEED_MULT_SHIFT) 

/*!
 * @brief MMC card EXT_CSD version.
 */
typedef enum _mmc_ext_csd_ver
{
    kMMC_ExtCsdVer10 = 0U,    /*!< Revision 1.0 */
    kMMC_ExtCsdVer11 = 1U,    /*!< Revision 1.1 */
    kMMC_ExtCsdVer12 = 2U,    /*!< Revision 1.2 */
    kMMC_ExtCsdVer13 = 3U     /*!< Revision 1.3 */
} mmc_ext_csd_ver_t;

/*!
 * @brief EXT_CSD register access mode.
 */
typedef enum _mmc_ext_csd_access_mode
{
    kMMC_ExtCsdCmdSet = 0U, /*!< The command set is changed according to the Cmd Setfield of the argument  */   
    kMMC_ExtCsdSetBits    = 1U, /*!< The bits in the pointed byte are set, according to the bits in the Value field   */
    kMMC_ExtCsdClearBits  = 2U, /*!< The bits in the pointed byte are cleared, according to the bits in the Value field */
    kMMC_ExtCsdWriteBits  = 3U  /*!< The Value field is written into the pointed byte */          
} mmc_ext_csd_access_mode_t;

/*!
 * @brief MMC card command set.
 */
typedef enum _mmc_cmd_set
{
    kMMC_StandardMmc = 0U,        /*!< Standard MMC */
    kMMC_CmdSet1     = 1U,        /*!< Command set 1 */           
    kMMC_CmdSet2     = 2U,        /*!< Command set 2 */    
    kMMC_CmdSet3     = 3U,        /*!< Command set 3 */    
    kMMC_CmdSet4     = 4U         /*!< Command set 4 */ 
} mmc_cmd_set_t;

/*!
 * @brief Alternative boot support
 */
typedef enum _mmc_alter_boot
{
    kMMC_NotSupportAlterBoot = 0U,  /*!< Device does not support alternate boot method */
    kMMC_SupportAlterBoot    = 1U   /*!< Device supports alternate boot method. */
} mmc_alter_boot_t;

/*!
 * @brief MMC card power class used in PWR_CL_ff_vvv in EXT_CSD.
 */
typedef enum _mmc_power_class
{
    kMMC_PowerClassLev0  = 0U,     /*!< power class level 1 */
    kMMC_PowerClassLev1  = 1U,     /*!< power class level 2 */
    kMMC_PowerClassLev2  = 2U,     /*!< power class level 3 */
    kMMC_PowerClassLev3  = 3U,     /*!< power class level 4 */
    kMMC_PowerClassLev4  = 4U,     /*!< power class level 5 */
    kMMC_PowerClassLev5  = 5U,     /*!< power class level 6 */
    kMMC_PowerClassLev6  = 6U,     /*!< power class level 7 */
    kMMC_PowerClassLev7  = 7U,     /*!< power class level 8 */
    kMMC_PowerClassLev8  = 8U,     /*!< power class level 10 */
    kMMC_PowerClassLev9  = 9U,     /*!< power class level 12 */
    kMMC_PowerClassLev10 = 10U     /*!< power class level 13 */
} mmc_power_class_t;

/*!< The power class value bit mask when bus in 4 bit mode */
#define MMC_EXT_CSD_PWRCLFFVV_4BIT_MASK        (0x0FU)
/*!< The power class value bit mask when bus in 8 bit mode */
#define MMC_EXT_CSD_PWRCLFFVV_8BIT_MASK        (0xF0U)

/*!
 * @brief MMC card bus timing.
 */
typedef enum _mmc_bus_timing
{
    kMMC_NonehighSpeedTiming = 0U, /*!< MMC card using none high speed timing */
    kMMC_HighSpeedTiming     = 1U  /*!< MMC card using high speed timing */
} mmc_bus_timing_t;

/* MMC card type as high speed frequence */
typedef enum _mmc_card_type_hsfreq
{
    kMMC_CardTypeHSFreq26MHZ = 0x0U,
    kMMC_CardTypeHSFreq52MHZ = 0x2U,
} mmc_card_type_hsfreq_t; 

#define MMC_BUS_WIDTH_TYPE_NUM 3 /* The number of bus width type */
/*!
 * @brief MMC card bus width.
 */
typedef enum _mmc_bus_width
{
    kMMC_BusWidth1b = 0U,             /* MMC bus width is 1 bit */
    kMMC_BusWidth4b = 1U,             /* MMC bus width is 4 bits */
    kMMC_BusWidth8b = 2U,             /* MMC bus width is 8 bits */
} mmc_bus_width_t;

/*!
 * @brief MMC card boot partition enablement.
 */
typedef enum _mmc_boot_part_enable
{
    kMMC_BootNotEnabled           = 0U, /*!< No boot acknowledge sent (default) */
    kMMC_BootPart1Enabled    = 1U, /*!< Boot partition 1 enabled for boot */
    kMMC_BootPart2Enabled    = 2U, /*!< Boot partition 2 enabled for boot */
    kMMC_BootUserAeraEnabled      = 7U, /*!< User area enabled for boot */ 
} mmc_boot_part_enable_t;

/*!
 * @breif MMC card boot partition to be accessed.
 */
typedef enum _mmc_access_part
{
    kMMC_AccessBootPartNot = 0U, /*!< No access to boot partition (default), normal partition */
    kMMC_AccessBootPart1   = 1U, /*!< R/W boot partition 1*/
    kMMC_AccessBootPart2   = 2U, /*!< R/W boot partition 2*/
} mmc_access_part_t;

/*!
 * @brief MMC card boot configuration definition.
 */
#define MMC_BOOT_CONFIG_PART_ACCESS_SHIFT     0U
#define MMC_BOOT_CONFIG_PART_ACCESS_MASK    0x00000007U
#define MMC_BOOT_CONFIG_PART_ENABLE_SHIFT     3U
#define MMC_BOOT_CONFIG_PART_ENABLE_MASK    0x00000038U
#define MMC_BOOT_CONFIG_ACK_SHIFT             6U
#define MMC_BOOT_CONFIG_ACK_MASK            0x00000040U
#define MMC_BOOT_BUS_WIDTH_WIDTH_SHIFT        8U
#define MMC_BOOT_BUS_WIDTH_WIDTH_MASK       0x00000300U
#define MMC_BOOT_BUS_WIDTH_RESET_SHIFT        10U
#define MMC_BOOT_BUS_WIDTH_RESET_MASK       0x00000400U  

/*! @brief EXT CSD byte index */
typedef enum _mmc_ext_csd_index
{
    kMMC_ExtCsdIndexEraseGroupDef      = 175U,   /*!< Erase Group Def */
    kMMC_ExtCsdIndexBootBusWidth       = 177U,   /*!< Boot Bus Width */
    kMMC_ExtCsdIndexBootConfig         = 179U,   /*!< Boot Config */
    kMMC_ExtCsdIndexBusWidth           = 183U,   /*!< Bus Width */
    kMMC_ExtCsdIndexHSTiming           = 185U,   /*!< HS Timing */
    kMMC_ExtCsdIndexPowerClass         = 187U,   /*!< Power Class */
    kMMC_ExtCsdIndexCmdSet             = 191U,   /*!< Cmd Set */
} mmc_ext_csd_index_t;

/* Command set bit position in SWITCH command parameter */
#define MMC_SWITCH_CMD_SET_SHIFT          0U 
#define MMC_SWITCH_CMD_SET_MASK           0x00000007U      
#define MMC_SWITCH_VALUE_SHIFT            8U          
#define MMC_SWITCH_VALUE_MASK             0x0000FF00U
#define MMC_SWITCH_INDEX_OF_BYTE_SHIFT    16U
#define MMC_SWITCH_INDEX_OF_BYTE_MASK     0x00FF0000U
#define MMC_SWITCH_ACCESS_MODE_SHIFT      24U 
#define MMC_SWTICH_ACCESS_MODE_MASK       0x03000000U

/*!
 * @brief MMC card operation.
 */
typedef struct MMCExtCsdOperation
{
    mmc_cmd_set_t cmdSet;                 /*!< Command set */
    uint8_t value;                        /*!< The value to set */
    uint8_t indexOfByte;                  /*!< The byte index in EXT_CSD */
    mmc_ext_csd_access_mode_t accessMode; /*!< Access mode */
} mmc_ext_csd_operation_t; 

/*!< The length of CID, CSD, EXT_CSD register, unit of length is word(128), byte(512)) */
#define MMC_EXT_CSD_LEN_AS_WORD                  128U  
#define MMC_EXT_CSD_LEN_AS_BYTE                  512U

/* The Minimum RSA value can be assigned to the card */
#define MMC_MINIMUM_RSA                     2U
/* MMC card default RSA */    
#define MMC_DEFAULT_RSA     MMC_MINIMUM_RSA   

/* Bus test pattern when bus is at 8 bit width mode */
#define MMC_8BIT_BUS_TEST_PATTERN        0x0000AA55U
/* The XOR result of test pattern when bus is at 8 bit width mode */
#define MMC_8BIT_BUS_PATTERN_XOR_RESULT  0x0000FFFFU
/* Bus test pattern when bus is at 4 bit width mode */
#define MMC_4BIT_BUS_TEST_PATTERN        0x0000005AU
/* The XOR result of test pattern when bus is at 4 bit width mode */
#define MMC_4BIT_BUS_PATTERN_XOR_RESULT  0x000000FFU 
/* Bus test pattern when bus is at 1 bit width mode */
#define MMC_1BIT_BUS_TEST_PATTERN        0x80U
/* The XOR result of test pattern when bus is at 1 bit width mode */
#define MMC_1BIT_BUS_PATTERN_XOR_RESULT  0x000000C0U

/* MMC product name length*/
#define MMC_PRODUCT_NAME_LEN 6     
/*!
 * @brief MMC card CID register fields.
 */
typedef struct MMCCid
{
    uint8_t mid;         /*!< Manufacturer ID */
    uint16_t oid;        /*!< OEM/Application ID */
    uint8_t pnm[MMC_PRODUCT_NAME_LEN]; /*!< Product name */
    uint8_t prv;         /*!< Product revision */
    uint32_t psn;        /*!< Product serial number */
    uint8_t mdt;         /*!< Manufacturing date */
} mmc_cid_t;

typedef enum 

typedef enum _mmc_csd_flag
{
    kMMC_CsdReadBlockPartial           = (1U << 0U),  /*!< Partial blocks for read allowed */
    kMMC_CsdWriteBlockMisalign         = (1U << 1U),  /*!< Write block misalignment */
    kMMC_CsdReadBlockMisalign          = (1U << 2U),  /*!< Read block misalignment */
    kMMC_CsdDsrImplemented             = (1U << 3U),  /*!< DSR implemented */
    kMMC_CsdWPGroupEnabled             = (1U << 4U),  /*!< Write protect group enabled */
    kMMC_CsdWriteBlockPartial          = (1U << 5U),  /*!< Partial blocks for write allowed */
    kMMC_ContentProtectApp             = (1U << 6U),  /*!< Content protect application */
    kMMC_CsdFileFormatGroup            = (1U << 7U),  /*!< File format group */
    kMMC_CsdCopy                       = (1U << 8U),  /*!< Copy flag */
    kMMC_CsdPermWriteProtect           = (1U << 9U),  /*!< Permanent write protection */
    kMMC_CsdTmpWriteProtect            = (1U << 10U), /*!< Temporary write protection */
} mmc_csd_flag_t;

/*!
 * @brief MMC card CSD register fields.
 */
typedef struct MMCCsd
{
    uint8_t csdStructVer;         /*!< CSD structure [127:126]*/
    uint8_t sysSpecVer;           /*!< System specification version */
    uint8_t taac;                 /*!< Data read access-time 1 */
    uint8_t nsac;                 /*!< Data read access-time 2 in CLK cycles (NSAC*100) */
    uint8_t tranSpeed;            /*!< Max. bus clock frequency */
    uint16_t ccc;                 /*!< card command classes */
    uint8_t readBlkLen;           /*!< Max. read data block length */
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
typedef struct MMCExtCsd
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
    /* Following fields only has effective value in MMC mode:from MIN_PERF_W_8_52 to PWR_CL_52_195, 
    powerCls, HS_TIMING, busWidth */
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
    /* Following fields only has effective value in EMMV V4.3 in MMC mode: from bootConfig to ERASE_GROUP_DEF */    
    uint8_t bootConfig;                  /*!< Boot configuration, [179] */
    uint8_t bootBusWidth;                /*!< Boot bus width, [177] */
    uint8_t ERASE_GROUP_DEF;             /*!< High-density erase group definition, [175] */
} mmc_ext_csd_t;

#endif  /* __SPEC_H__ */

