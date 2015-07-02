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
#ifndef __SDHC_H__
#define __SDHC_H__

#include <stdbool.h>
#include "fsl_device_registers.h"
#if FSL_FEATURE_SOC_SDHC_COUNT

/*! @addtogroup sdhc */
/*! @{ */

/******************************************************************************
 * Definitions.
 *****************************************************************************/

/******************SDHC atrribute related definition**************************/
/* Block Attributes register (BLKATTR) */
#define SDHC_MAX_BLOCK_COUNT        ((1U << SDHC_BLKATTR_BLKCNT_WIDTH) - 1U)


/******************SDHC setting related definition****************************/
/* System Control register(SYSCTL) */
#define SDHC_MAX_DVS                (16U)
#define SDHC_INITIAL_DVS            (1U)            /* initial value of divisor to calculate clock rate */
#define SDHC_INITIAL_CLKFS          (2U)            /* initial value of clock selector to calculate clock rate */
#define SDHC_NEXT_DVS(x)            do { ((x) += 1U); } while(0)
#define SDHC_PREV_DVS(x)            do { ((x) -= 1U); } while(0)
#define SDHC_MAX_CLKFS              (256U)
#define SDHC_NEXT_CLKFS(x)          do { ((x) <<= 1U); } while(0)
#define SDHC_PREV_CLKFS(x)          do { ((x) >>= 1U); } while(0)
/* Card reset type for signal line */
#define SDHC_RST_TYPE_ALL               SDHC_SYSCTL_RSTA_MASK
#define SDHC_RST_TYPE_CMD               SDHC_SYSCTL_RSTC_MASK
#define SDHC_RST_TYPE_DATA              SDHC_SYSCTL_RSTD_MASK

/* Force Event register(FEVT) */
#define SDHC_ACMD12_NOT_EXEC_ERR_EVENT  SDHC_FEVT_AC12NE_MASK
#define SDHC_ACMD12_TIMEOUT_ERR_EVENT   SDHC_FEVT_AC12TOE_MASK
#define SDHC_ACMD12_CRC_ERR_EVENT       SDHC_FEVT_AC12CE_MASK
#define SDHC_ACMD12_END_BIT_ERR_EVENT   SDHC_FEVT_AC12EBE_MASK
#define SDHC_ACMD12_INDEX_ERR_EVENT     SDHC_FEVT_AC12IE_MASK
#define SDHC_ACMD12_NOT_ISSUE_ERR_EVENT SDHC_FEVT_CNIBAC12E_MASK
#define SDHC_CMD_TIMEOUT_ERR_EVENT      SDHC_FEVT_CTOE_MASK
#define SDHC_CMD_CRC_ERR_EVENT          SDHC_FEVT_CCE_MASK
#define SDHC_CMD_END_BIT_ERR_EVENT      SDHC_FEVT_CEBE_MASK
#define SDHC_CMD_INDEX_ERR_EVENT        SDHC_FEVT_CIE_MASK
#define SDHC_DATA_TIMEOUT_ERR_EVENT     SDHC_FEVT_DTOE_MASK
#define SDHC_DATA_CRC_ERR_EVENT         SDHC_FEVT_DCE_MASK
#define SDHC_DATA_END_BIT_ERR_EVENT     SDHC_FEVT_DEBE_MASK
#define SDHC_ACMD12_ERR_EVENT           SDHC_FEVT_AC12E_MASK
#define SDHC_CARD_INT_EVENT             SDHC_FEVT_CINT_MASK
#define SDHC_DMA_ERROR_EVENT            SDHC_FEVT_DMAE_MASK

/* Transfer Type register(XFERTYP) */
#define SDHC_ENABLE_DMA             SDHC_XFERTYP_DMAEN_MASK

#define SDHC_CMD_TYPE_SUSPEND       (SDHC_XFERTYP_CMDTYP(1U))
#define SDHC_CMD_TYPE_RESUME        (SDHC_XFERTYP_CMDTYP(2U))
#define SDHC_CMD_TYPE_ABORT         (SDHC_XFERTYP_CMDTYP(3U))

#define SDHC_ENABLE_BLOCK_COUNT     SDHC_XFERTYP_BCEN_MASK
#define SDHC_ENABLE_AUTO_CMD12      SDHC_XFERTYP_AC12EN_MASK
#define SDHC_ENABLE_DATA_READ       SDHC_XFERTYP_DTDSEL_MASK
#define SDHC_MULTIPLE_BLOCK         SDHC_XFERTYP_MSBSEL_MASK

#define SDHC_RESP_LEN_136           ((0x1U << SDHC_XFERTYP_RSPTYP_SHIFT) & SDHC_XFERTYP_RSPTYP_MASK)
#define SDHC_RESP_LEN_48            ((0x2U << SDHC_XFERTYP_RSPTYP_SHIFT) & SDHC_XFERTYP_RSPTYP_MASK)
#define SDHC_RESP_LEN_48_BC         ((0x3U << SDHC_XFERTYP_RSPTYP_SHIFT) & SDHC_XFERTYP_RSPTYP_MASK)

#define SDHC_ENABLE_CRC_CHECK       SDHC_XFERTYP_CCCEN_MASK
#define SDHC_ENABLE_INDEX_CHECK     SDHC_XFERTYP_CICEN_MASK
#define SDHC_DATA_PRESENT           SDHC_XFERTYP_DPSEL_MASK

/* Defines the flag corresponding to each response type which will be set in each transfer. */
#define SDHC_REQ_RSPTYPE_NONE    (0U)
#define SDHC_REQ_RSPTYPE_R1      (SDHC_DATA_PRESENT | \
                                  SDHC_ENABLE_CRC_CHECK | \
                                  SDHC_ENABLE_INDEX_CHECK)       /* Response 1 */
#define SDHC_REQ_RSPTYPE_R1B      SDHC_DATA_PRESENT | \
                                  SDHC_ENABLE_CRC_CHECK | \
                                  SDHC_ENABLE_INDEX_CHECK | \
                                  SDHC_RESP_LEN_48_BC)           /* Response 1 with busy */
#define SDHC_REQ_RSPTYPE_R2       SDHC_DATA_PRESENT | \
                                  SDHC_REQ_RSPTYPE_136BITS | \
                                  SDHC_ENABLE_CRC_CHECK)         /* Response 2 */
#define SDHC_REQ_RSPTYPE_R3       SDHC_DATA_PRESENT)             /* Response 3 */
#define SDHC_REQ_RSPTYPE_R4       SDHC_DATA_PRESENT)             /* Response 4 */
#define SDHC_REQ_RSPTYPE_R5       SDHC_DATA_PRESENT | \
                                  SDHC_ENABLE_CRC_CHECK | \
                                  SDHC_ENABLE_INDEX_CHECK)       /* Response 5 */
#define SDHC_REQ_RSPTYPE_R5B      SDHC_DATA_PRESENT | \
                                  SDHC_ENABLE_CRC_CHECK | \
                                  SDHC_ENABLE_INDEX_CHECK | \
                                  SDHC_RESP_LEN_48_BC)           /* Response 5 with busy */
#define SDHC_REQ_RSPTYPE_R6       SDHC_DATA_PRESENT | \
                                  SDHC_ENABLE_CRC_CHECK | \
                                  SDHC_ENABLE_INDEX_CHECK)       /* Response 6 */
#define SDHC_REQ_RSPTYPE_R7       SDHC_DATA_PRESENT | \
                                  SDHC_ENABLE_CRC_CHECK | \
                                  SDHC_ENABLE_INDEX_CHECK)       /* Response 7 */

/***********************SDHC status related definition****************************/
/* Present State Register(PRSSTA) */
#define SDHC_CMD_INHIBIT                SDHC_PRSSTAT_CIHB_MASK
#define SDHC_DAT_INHIBIT                SDHC_PRSSTAT_CDIHB_MASK
#define SDHC_DAT_LINE_ACTIVE            SDHC_PRSSTAT_DLA_MASK
#define SDHC_SD_CLK_STABLE              SDHC_PRSSTAT_SDSTB_MASK
#define SDHC_IPG_CLK_OFF                SDHC_PRSSTAT_IPGOFF_MASK
#define SDHC_SYS_CLK_OFF                SDHC_PRSSTAT_HCKOFF_MASK
#define SDHC_PERIPHERAL_CLK_OFF         SDHC_PRSSTAT_PEROFF_MASK
#define SDHC_SD_CLK_OFF                 SDHC_PRSSTAT_SDOFF_MASK 
#define SDHC_WRITE_TRANSFER_ACTIVE      SDHC_PRSSTAT_WTA_MASK
#define SDHC_READ_TRANSFER_ACTIVE       SDHC_PRSSTAT_RTA_MASK
#define SDHC_BUFF_WRITE_ENABLED         SDHC_PRSSTAT_BWEN_MASK
#define SDHC_BUFF_READ_ENABLED          SDHC_PRSSTAT_BREN_MASK
#define SDHC_CARD_INSERTED              SDHC_PRSSTAT_CINS_MASK
#define SDHC_CMD_LINE_LEVEL             SDHC_PRSSTAT_CLSL_MASK
 /* Data 0 line is busy signal detection line */
#define SDHC_DATA0_LINE_LEVEL           (SDHC_PRSSTAT_DLSL_MASK & (1 << 24U))
#define SDHC_DATA1_LINE_LEVEL           (SDHC_PRSSTAT_DLSL_MASK & (1 << 25U))
#define SDHC_DATA2_LINE_LEVEL           (SDHC_PRSSTAT_DLSL_MASK & (1 << 26U))
#define SDHC_DATA3_LINE_LEVEL           (SDHC_PRSSTAT_DLSL_MASK & (1 << 27U))
#define SDHC_DATA4_LINE_LEVEL           (SDHC_PRSSTAT_DLSL_MASK & (1 << 28U))
#define SDHC_DATA5_LINE_LEVEL           (SDHC_PRSSTAT_DLSL_MASK & (1 << 29U))
#define SDHC_DATA6_LINE_LEVEL           (SDHC_PRSSTAT_DLSL_MASK & (1 << 30U))
#define SDHC_DATA7_LINE_LEVEL           (SDHC_PRSSTAT_DLSL_MASK & (1 << 31U))

/* Interrupt Status register(IRQSTAT) */
#define SDHC_CMD_COMPLETE_INT       SDHC_IRQSTAT_CC_MASK
#define SDHC_DATA_COMPLETE_INT      SDHC_IRQSTAT_TC_MASK
#define SDHC_BLOCK_GAP_EVENT_INT    SDHC_IRQSTAT_BGE_MASK
#define SDHC_DMA_INT                SDHC_IRQSTAT_DINT_MASK
#define SDHC_BUF_WRITE_READY_INT    SDHC_IRQSTAT_BWR_MASK
#define SDHC_BUF_READ_READY_INT     SDHC_IRQSTAT_BRR_MASK
#define SDHC_CARD_INSERTION_INT     SDHC_IRQSTAT_CINS_MASK
#define SDHC_CARD_REMOVAL_INT       SDHC_IRQSTAT_CRM_MASK
#define SDHC_CARD_INT               SDHC_IRQSTAT_CINT_MASK
#define SDHC_CMD_TIMEOUT_ERR_INT    SDHC_IRQSTAT_CTOE_MASK
#define SDHC_CMD_CRC_ERR_INT        SDHC_IRQSTAT_CCE_MASK
#define SDHC_CMD_END_BIT_ERR_INT    SDHC_IRQSTAT_CEBE_MASK
#define SDHC_CMD_INDEX_ERR_INT      SDHC_IRQSTAT_CIE_MASK
#define SDHC_DATA_TIMEOUT_ERR_INT   SDHC_IRQSTAT_DTOE_MASK
#define SDHC_DATA_CRC_ERR_INT       SDHC_IRQSTAT_DCE_MASK
#define SDHC_DATA_END_BIT_ERR_INT   SDHC_IRQSTAT_DEBE_MASK
#define SDHC_AUTO_CMD12_ERR_INT     SDHC_IRQSTAT_AC12E_MASK
#define SDHC_DMA_ERR_INT            SDHC_IRQSTAT_DMAE_MASK

#define SDHC_CMD_ERR_INT            ((uint32_t)(SDHC_CMD_TIMEOUT_ERR_INT | \
                                    SDHC_CMD_CRC_ERR_INT | \
                                    SDHC_CMD_END_BIT_ERR_INT | \
                                    SDHC_CMD_INDEX_ERR_INT))
#define SDHC_DATA_ERR_INT           ((uint32_t)(SDHC_DATA_TIMEOUT_ERR_INT | \
                                    SDHC_DATA_CRC_ERR_INT | \
                                    SDHC_DATA_END_BIT_ERR_INT))
#define SDHC_DATA_ALL_INT           ((uint32_t)(SDHC_DATA_ERR_INT | \
                                    SDHC_DATA_COMPLETE_INT | \
                                    SDHC_BUF_READ_READY_INT | \
                                    SDHC_BUF_WRITE_READY_INT | \
                                    SDHC_DMA_ERR_INT | SDHC_DMA_INT))
#define SDHC_CMD_ALL_INT            ((uint32_t)(SDHC_CMD_ERR_INT | \
                                    SDHC_CMD_COMPLETE_INT | \
                                    SDHC_AUTO_CMD12_ERR_INT))
#define SDHC_CD_ALL_INT             ((uint32_t)(SDHC_CARD_INSERTION_INT | \
                                    SDHC_CARD_REMOVAL_INT))
#define SDHC_ALL_ERR_INT            ((uint32_t)(SDHC_CMD_ERR_INT | \
                                    SDHC_DATA_ERR_INT | \
                                    SDHC_AUTO_CMD12_ERR_INT | \
                                    SDHC_DMA_ERR_INT))

/* Auto CMD12 Error Status Register(AC12ERR) */
#define SDHC_ACMD12_NOT_EXEC_ERR    SDHC_AC12ERR_AC12NE_MASK
#define SDHC_ACMD12_TIMEOUT_ERR     SDHC_AC12ERR_AC12TOE_MASK
#define SDHC_ACMD12_END_BIT_ERR     SDHC_AC12ERR_AC12EBE_MASK
#define SDHC_ACMD12_CRC_ERR         SDHC_AC12ERR_AC12CE_MASK
#define SDHC_ACMD12_INDEX_ERR       SDHC_AC12ERR_AC12IE_MASK
#define SDHC_ACMD12_NOT_ISSUE_ERR   SDHC_AC12ERR_CNIBAC12E_MASK

/* ADMA Error Status register(ADMAES) */
/* ADMA Error State (When ADMA Error Is Occurred.) */
#define SDHC_ADMA_STATE_ERR                      SDHC_ADMAES_ADMAES_MASK
/* ADMA Length Mismatch Error */
#define SDHC_ADMA_LEN_MIS_MATCH_ERR              SDHC_ADMAES_ADMALME_MASK
/* ADMA Descriptor Error */ 
#define SDHC_ADMA_DESCRIPTOR_ERR                 SDHC_ADMAES_ADMADCE_MASK

/* Protocol Control register(SPROCTL) */
#define SDHC_LED_ON           SDHC_PROCTL_LCTL_MASK


/********************SDHC atrribute related enumeration***********************/
/*! @brief Max block length supported by sdhc */
typedef enum _sdhc_max_blk_len {
    kSdhcMaxBlkLen512B  = 0U, /*!< 512 bytes */
    kSdhcMaxBlkLen1024B = 1U, /*!< 1024 bytes */
    kSdhcMaxBlkLen2048B = 2U, /*!< 2048 bytes */
    kSdhcMaxBlkLen4096B = 3U  /*!< 4096 bytes */
} sdhc_max_blk_len_t;


/********************SDHC setting related enumeration*************************/
/*! @brief Control led status to caution user not to remove the active card. */
typedef enum _sdhc_led {
    kSdhcLedOff = 0U,  /*!< LED off */
    kSdhcLedOn  = 1U,  /*!< LED on */
} sdhc_led_t;

/*! @brief Data transfer width */
typedef enum _sdhc_dtw {
    kSdhcDtw1Bit = 0U,  /*!< 1-bit mode */
    kSdhcDtw4Bit = 1U,  /*!< 4-bit mode */
    kSdhcDtw8Bit = 2U,  /*!< 8-bit mode */
} sdhc_dtw_t;

/*! @brief SDHC endian mode */
typedef enum _sdhc_endian {
    kSdhcEndianBig         = 0U, /*!< Big endian mode */
    kSdhcEndianfWordBig    = 1U, /*!< f word big endian mode */
    kSdhcEndianLittle      = 2U, /*!< Little endian mode */
} sdhc_endian_t;

/*! @brief SDHC dma mode */
typedef enum _sdhc_dma_mode {
    kSdhcDmaSimple = 0U, /*!< No DMA or simple DMA is selected */
    kSdhcDmaAdma1  = 1U, /*!< ADMA1 is selected */
    kSdhcDmaAdma2  = 2U, /*!< ADMA2 is selected */
} sdhc_dma_mode_t;

/*! 
* @brief Adma error state 
*
* This state is corresponding to SDHC_ADMA_STATE_ERR to inidicate the ADMA's state
* when error happened.
*/
typedef enum _sdhc_adma_err_state
{
    kSdhcAdmaErrStateOfStopDma,         /*!< Stop DMA */
    kSdhcAdmaErrStateOfFetchDescriptor, /*!< Fetch descriptor */
    kSdhcAdmaErrStateOfChangeAddress,   /*!< Change address */
    kSdhcAdmaErrStateOfTransferData     /*!< Transfer data */
} sdhc_adma_err_state;

/*! @brief MMC card boot ack time out counter value */
typedef enum _sdhc_boot_ack_timeout
{
    kSdhcBootAckTimeoutCntOf0  = 0U,   /*!< Boot ack timeout count value is SDCLK x 2^8 */
    kSdhcBootAckTimeoutCntOf1  = 1U,   /*!< Boot ack timeout count value is SDCLK x 2^9 */
    kSdhcBootAckTimeoutCntOf2  = 2U,   /*!< Boot ack timeout count value is SDCLK x 2^10 */
    kSdhcBootAckTimeoutCntOf3  = 3U,   /*!< Boot ack timeout count value is SDCLK x 2^11 */
    kSdhcBootAckTimeoutCntOf4  = 4U,   /*!< Boot ack timeout count value is SDCLK x 2^12 */
    kSdhcBootAckTimeoutCntOf5  = 5U,   /*!< Boot ack timeout count value is SDCLK x 2^13 */
    kSdhcBootAckTimeoutCntOf6  = 6U,   /*!< Boot ack timeout count value is SDCLK x 2^14 */
    kSdhcBootAckTimeoutCntOf7  = 7U,   /*!< Boot ack timeout count value is SDCLK x 2^15 */
    kSdhcBootAckTimeoutCntOf8  = 8U,   /*!< Boot ack timeout count value is SDCLK x 2^16 */
    kSdhcBootAckTimeoutCntOf9  = 9U,   /*!< Boot ack timeout count value is SDCLK x 2^17 */
    kSdhcBootAckTimeoutCntOf10 = 10U,  /*!< Boot ack timeout count value is SDCLK x 2^18 */
    kSdhcBootAckTimeoutCntOf11 = 11U,  /*!< Boot ack timeout count value is SDCLK x 2^19 */
    kSdhcBootAckTimeoutCntOf12 = 12U,  /*!< Boot ack timeout count value is SDCLK x 2^20 */
    kSdhcBootAckTimeoutCntOf13 = 13U,  /*!< Boot ack timeout count value is SDCLK x 2^21 */
    kSdhcBootAckTimeoutCntOf14 = 14U,  /*!< Boot ack timeout count value is SDCLK x 2^22 */
} sdhc_boot_ack_timeout_t;

/*! @brief MMC card fast boot mode */
typedef enum _sdhc_boot_mode {
    kSdhcbootModeNormal = 0U,  /*!< Normal boot */
    kSdhcbootModeAlter  = 1U,  /*!< Alternative boot */ 
} sdhc_boot_mode_t;

/*! @brief SDIO card related control enablement command */
typedef enum _sdhc_sdio_enable_cmd
{
    kSdhcStopAtBlkGapEnable    = 0U,    /*!< Stop at block gap request */
    kSdhcReadWaitControlEnable = 1U,    /*!< Read wait control to the SDIO card */
    kSdhcIntAtBlkGap           = 2U,    /*!< Interrupt at block gap */
    kSdhcExactBlkNumRead       = 3U     /*!< Exact block number block read */
} sdhc_sdio_enable_cmd_t;


/********************SDHC status related enumration*********************/
/********************SDHC high level related enumeration*****************/
/*!@brief Host related status */
typedef enum _sdhc_status {
    kStatus_SDHC_NoError = 0,               /*!< No error */
    kStatus_SDHC_InitFailed,                /*!< Driver initialization failed */
    kStatus_SDHC_SetClockFailed,            /*!< Failed to set clock of host controller */
    kStatus_SDHC_SetBusWidthFailed,         /*!< Failed to set host's bus mode */
    kStatus_SDHC_HostIsAlreadyInited,       /*!< Host controller is already initialized */
    kStatus_SDHC_HostNotSupport,            /*!< Host not error */
    kStatus_SDHC_HostIsBusyError,           /*!< Bus busy error */
    kStatus_SDHC_DataPrepareError,          /*!< Data preparation error */
    kStatus_SDHC_WaitTimeoutError,          /*!< Wait timeout error */
    kStatus_SDHC_OutOfMemory,               /*!< Out of memory error */
    kStatus_SDHC_IoError,                   /*!< General IO error */
    kStatus_SDHC_CmdIoError,                /*!< CMD I/O error */
    kStatus_SDHC_DataIoError,               /*!< Data I/O error */
    kStatus_SDHC_InvalidParameter,          /*!< Invalid parameter error */
    kStatus_SDHC_RequestFailed,             /*!< Request failed */
    kStatus_SDHC_DmaAddressError,           /*!< DMA address error */
    kStatus_SDHC_Failed,                    /*!< General failed */
    kStatus_SDHC_NoMedium,                  /*!< No medium error */
    kStatus_SDHC_UnknownStatus              /*!< Unknown if card is present */
} sdhc_status_t;


/***************SDHC ADMA descriptor related defintion******************/
/*! @brief SDHC ADMA address alignment size and length alignment size */
#define SDHC_ADMA1_ADDR_ALIGN           (4096U)
#define SDHC_ADMA1_LEN_ALIGN            (4096U)
#define SDHC_ADMA2_ADDR_ALIGN           (4U)
#define SDHC_ADMA2_LEN_ALIGN            (4U)

/*!
* @brief Defines the adma1 descriptor structure.
*
* ADMA1 descriptor table
* |------------------------|---------|--------------------------|
* | Address/page Field     |reserved |         Attribute        |
* |------------------------|---------|--------------------------|
* |31                    12|11      6|05  |04  |03|02 |01 |00   |
* |------------------------|---------|----|----|--|---|---|-----|
* | address or data length | 000000  |Act2|Act1| 0|Int|End|Valid|
* |------------------------|---------|----|----|--|---|---|-----|
*
*
* |------|------|-----------------|-------|-------------|
* | Act2 | Act1 |     Comment     | 31-28 | 27 - 12     |
* |------|------|-----------------|---------------------|
* |   0  |   0  | No op           | Don't care          |
* |------|------|-----------------|-------|-------------|
* |   0  |   1  | Set data length |  0000 | Data Length |
* |------|------|-----------------|-------|-------------|
* |   1  |   0  | Transfer data   | Data address        |
* |------|------|-----------------|---------------------|
* |   1  |   1  | Link descriptor | Descriptor address  |
* |------|------|-----------------|---------------------|
*/
typedef uint32_t sdhc_adma1_descriptor_t;
/* The mask for the control/status field in ADMA1 descriptor */
#define SDHC_ADMA1_DESC_VALID_MASK           (1U << 0U)
#define SDHC_ADMA1_DESC_END_MASK             (1U << 1U)
#define SDHC_ADMA1_DESC_INT_MASK             (1U << 2U)
#define SDHC_ADMA1_DESC_ACT1_MASK            (1U << 4U)
#define SDHC_ADMA1_DESC_ACT2_MASK            (1U << 5U)
#define SDHC_ADMA1_DESC_TYPE_NOP             (SDHC_ADMA1_DESC_VALID_MASK)
#define SDHC_ADMA1_DESC_TYPE_TRAN            (SDHC_ADMA1_DESC_ACT2_MASK | SDHC_ADMA1_DESC_VALID_MASK)
#define SDHC_ADMA1_DESC_TYPE_LINK            (SDHC_ADMA1_DESC_ACT1_MASK | SDHC_ADMA1_DESC_ACT2_MASK | SDHC_ADMA1_DESC_VALID_MASK)
#define SDHC_ADMA1_DESC_TYPE_SET             (SDHC_ADMA1_DESC_ACT1_MASK | SDHC_ADMA1_DESC_VALID_MASK)
#define SDHC_ADMA1_DESC_ADDRESS_SHIFT        (12U)
#define SDHC_ADMA1_DESC_ADDRESS_MASK         (0xFFFFFU)
#define SDHC_ADMA1_DESC_LEN_SHIFT            (12U)
#define SDHC_ADMA1_DESC_LEN_MASK             (0xFFFFU)
#define SDHC_ADMA1_DESC_MAX_LEN_PER_ENTRY    (SDHC_ADMA1_DESC_LEN_MASK + 1)

/*!
* @brief Defines the ADMA2 descriptor structure.
*
* ADMA2 descriptor table
* |----------------|---------------|-------------|--------------------------|
* | Address Field  |     length    | reserved    |         Attribute        |
* |----------------|---------------|-------------|--------------------------|
* |63            32|31           16|15         06|05  |04  |03|02 |01 |00   |
* |----------------|---------------|-------------|----|----|--|---|---|-----|
* | 32-bit address | 16-bit length | 0000000000  |Act2|Act1| 0|Int|End|Valid|
* |----------------|---------------|-------------|----|----|--|---|---|-----|
*
*
* | Act2 | Act1 |     Comment     | Operation                                                         |
* |------|------|-----------------|-------------------------------------------------------------------|
* |   0  |   0  | No op           | Don't care                                                        |
* |------|------|-----------------|-------------------------------------------------------------------|
* |   0  |   1  | Reserved        | Read this line and go to next one                                 |
* |------|------|-----------------|-------------------------------------------------------------------|
* |   1  |   0  | Transfer data   | Transfer data with address and length set in this descriptor line |
* |------|------|-----------------|-------------------------------------------------------------------|
* |   1  |   1  | Link descriptor | Link to another descriptor                                        |
* |------|------|-----------------|-------------------------------------------------------------------|
*/
typedef struct SdhcAdma2Descriptor {
    uint32_t attribute; /*!< The control and status field */
    uint32_t *address;  /*!< The address field */
} sdhc_adma2_descriptor_t;

/* ADMA1 descriptor control and status mask */
#define SDHC_ADMA2_DESC_VALID_MASK           (1U << 0U)
#define SDHC_ADMA2_DESC_END_MASK             (1U << 1U)
#define SDHC_ADMA2_DESC_INT_MASK             (1U << 2U)
#define SDHC_ADMA2_DESC_ACT1_MASK            (1U << 4U)
#define SDHC_ADMA2_DESC_ACT2_MASK            (1U << 5U)
#define SDHC_ADMA2_DESC_TYPE_NOP             (SDHC_ADMA2_DESC_VALID_MASK)
#define SDHC_ADMA2_DESC_TYPE_RCV             (SDHC_ADMA2_DESC_ACT1_MASK | SDHC_ADMA2_DESC_VALID_MASK)
#define SDHC_ADMA2_DESC_TYPE_TRAN            (SDHC_ADMA2_DESC_ACT2_MASK | SDHC_ADMA2_DESC_VALID_MASK)
#define SDHC_ADMA2_DESC_TYPE_LINK            (SDHC_ADMA2_DESC_ACT1_MASK | SDHC_ADMA2_DESC_ACT2_MASK | SDHC_ADMA2_DESC_VALID_MASK)
#define SDHC_ADMA2_DESC_LEN_SHIFT            (16U)
#define SDHC_ADMA2_DESC_LEN_MASK             (0xFFFFU)
#define SDHC_ADMA2_DESC_MAX_LEN_PER_ENTRY    (SDHC_ADMA2_DESC_LEN_MASK)


/********************SDHC attribute saving related structure*********************/
/* Define the bit mask of the capability flag */
/* Voltage Support 3.3 V */ 
#define SDHC_SUPPORT_V330                   (1U << 0U)
/* Voltage Support 3.0 V */
#define SDHC_SUPPORT_V300                   (1U << 1U)
/* High Speed Support */
#define SDHC_SUPPORT_HIGHSPEED              (1U << 2U)
/* DMA Support */
#define SDHC_SUPPORT_DMA                    (1U << 3U)
/* ADMA Support */
#define SDHC_SUPPORT_ADMA                   (1U << 4U)
/* Suspend/Resume Support */
#define SDHC_SUPPORT_SUSPEND_RESUME         (1U << 5U)
/* Voltage Support 1.8 V */
#define SDHC_SUPPORT_V180                   (1U << 6U)
/* Support external dma */
#define SDHC_SUPPORT_EXDMA                  (1U << 7U)
/*! 
 * @brief Structure to save the capability inforamtion of SDHC */
typedef struct SdhcCapability
{
    uint32_t specVer;           /*!< Specification version */
    uint32_t vendorVer;         /*!< Verdor version */
    uint32_t maxBlkLen;        /*!< Max block length */
    uint32_t supportMask;      /*!< Logic or of support flag bit mask defined above */
} sdhc_capability_t;


/********************SDHC dynamical setting related structure*********************/
/*! @brief SD protocol unit clock(SD_CLK) configuration */
typedef struct SdhcSdClkConfig
{
    bool sdClkEnable;      /*!< Enable or disable the SD_CLK */
    uint32_t baskClkFreq;  /*!< Sdhc module clock which is base clock of SD_CLK */
    uint32_t sdClkFreq;    /*!< Dest SD_CLK clock frequence want to set */
} sdhc_sd_clk_config_t;

/*! @brief Structure to fill command argument/transfer type/data block register content */
typedef struct SdhcCmdConfig
{
    uint32_t dataBlkSize; /*!< Cmd data Block size */
    uint32_t dataBlkCount;/*!< Cmd data Block count */
    uint32_t argument;    /*!< Cmd argument defined as the card's specification */
    uint32_t cmdIndex;     /*!< Cmd index defined as the card's specification */
    uint32_t cmdFlags;    /*!< Logic or of the bit mask from SDHC_ENABLE_DMA to SDHC_DATA_PRESENT */
} sdhc_cmd_config_t;


/********************SDHC initialization setting related structure*********************/
/* Enables the boot ACK. */
#define SDHC_ENABLE_BOOT_ACK                        (1U << 0U)
/* Enables the fast boot. */
#define SDHC_ENABLE_FAST_BOOT                       (1U << 1U)
/* Enables the automatic stop at the block gap. */
#define SDHC_ENABLE_BOOT_STOP_AT_BLK_GAP_FLAG       (1U << 2U)
/*! @brief Data structure to configure the MMC boot feature */
typedef struct SdhcBootParam
{
    sdhc_boot_ack_timeout_t ackTimeout;/*!< Timeout value for the boot Ack */
    sdhc_boot_mode_t mode;             /*!< Boot mode selection. */
    uint32_t blockCount;               /*!< Stop at block gap value of automatic mode */
    uint32_t enableMask;               /*!< Logic or of boot related enable bit mask defined above */
} sdhc_boot_param_t;

/* Driver's alternative feature */
/* Uses SDHC_ENABLE_AUTOCMD12 to make the driver enable Auto Cmd12 feature */
#define SDHC_ENABLE_AUTOCMD12
/* Uses SDHC_ENABLE_ADMA1 to make the driver enable ADMA1. */
#define SDHC_ENABLE_ADMA1
/* Uses SDHC_CAUTION_WHEN_ACCESS_CARD to make the driver set the led on before transaction. */
#define SDHC_CAUTION_WHEN_ACCESS_CARD
/* Uses SDHC_USING_BIG_ENDIAN to enable the big endian mode. */
/* Uses SDHC_POLL_CARD_DETECT to enable the polling to card detection pin. */

/* DAT3 As Card Detection Pin */
#define SDHC_ENABLE_DAT3_AS_CARD_DETECT_PIN         (1U << 0U)
/* Card detection test level is selected for test purpose. Insert card will set CDTL bit. */
#define SDHC_ENABLE_CARD_DETECT_LEVEL_FOR_TEST      (1U << 1U)
/* Wakeup event on the card interrupt */
#define SDHC_ENABLE_WAKEUP_ON_CARD_INT              (1U << 2U)
/* Wakeup event on the card insertion */
#define SDHC_ENABLE_WAKEUP_ON_CARD_INSERT           (1U << 3U)
/* Wakeup event on card removal. */
#define SDHC_ENABLE_WAKEUP_ON_CARD_REMOVE           (1U << 4U)
/* When internal DMA is not active, the external DMA request will be sent out */
#define SDHC_ENABLE_EXTERNAL_DMA_REQ                (1U << 5U)
/*! 
* @brief Data structure to initialize the SDHC 
*
* This structure contains the basic configuration need to be set by the user.
* For other configuration items to SDHC which is not contained by this structure,
* endian mode is set by using SDHC_USING_BIG_ENDIAN, write/read watermark 
* level is set to default 0x80 in the init function.
*/
typedef struct SdhcConfig
{
    sdhc_dma_mode_t dmaMode;          /*!< Sets the DMA mode. */
    uint32_t enableMask;              /*!< Logic or of SD protol unit enable bit mask defined above */
    sdhc_boot_param_t* bootParams;    /*!< MMC card boot related configuration */
} sdhc_config_t;

/********************SDHC high level related structure*****************/

/*************************************************************************************************
 * API
 ************************************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif

/*! @name SDHC  FUNCTION */
/*@{ */


/************************SDHC init/reset functions****************************/
/*!
* @brief SDHC module intialization function.
* 
* Configures the SDHC according to the user input information.
*
* @param base SDHC base address
* @param initConfig configuration information
*/
void SDHC_Init(SDHC_Type * base, const sdhc_config_t* initConfig);

/*!
* @brief Resets the SDHC module.
*
* Resets the hardware register to the default state.
*
* @param base SDHC base address
*/
void SDHC_Reset(SDHC_Type * base);


/**********SDHC get static attribute(will not change dynamically) functions***************/
/*!
* @brief Get the capability information of SDHC
*
* @param base SDHC base address
* @attribute the structure to save the capability information of SDHC
*/
void SDHC_GetCapability(SDHC_Type * base, sdhc_capability_t* capability);


/********************SDHC dynamical setting/getting related functions*********************/
/***************Circuit/bus environment setting related functions******************/
/*!
* @brief Resets the specific part circuit.
*
* @param base SDHC base address
* @param type the type of reset(SDHC_RST_TYPE_ALL, SDHC_RST_TYPE_CMD or SDHC_RST_TYPE_DATA)
* @param timeout timeout for reset
* @return 0 on success, else on error
*/
uint32_t SDHC_ResetSpecificBlock(SDHC_Type * base, uint32_t resetType, 
                           uint32_t timeout);

/*!
* @brief Sets SDHC SD protol unit clock.
*
* @param base SDHC base address
* @param sdClkConfig SDHC SD protol unit clock configuration
*/
void SDHC_SetSdClock(SDHC_Type * base, const sdhc_sd_clk_config_t* sdClkConfig);

/*!
* @brief Sends 80 clocks to the card to set it to be active state.
*
* When either of the PRSSTAT[CIHB] and PRSSTAT[CDIHB] bits are set, writing 1
* to this bit is ignored, that is, when command line or data lines are active, 
* write to this bit is not allowed. On the otherhand, when this bit is set, that is, 
* during intialization active period, it is allowed to issue command, and the 
* command bit stream will appear on the CMD pad after all 80 clock cycles are done. 
* So when this command ends, the driver can make sure the 80 clock cycles are sent out. 
* This is very useful when the driver needs send 80 cycles to the card and does
* not want to wait till this bit is self-cleared.
*
* @param base SDHC base address
* @param timeout timeout to initialize card
* @return 0 on success, else on error
*/
uint32_t SDHC_SetCardActive(SDHC_Type * base, uint32_t timeout);


/***************Transfer parameter setting related functions******************/
/*!
* @brief Sets the data transfer width.
*
* @param base SDHC base address
* @param dtw data transfer width
*/
static inline void SDHC_SetDataTransferWidth(SDHC_Type * base, sdhc_dtw_t dtw)
{
    SDHC_BWR_PROCTL_DTW(base, dtw);
}

/*!
* @brief Sets the ADMA address.
*
* @param base SDHC base address
* @param address the address for ADMA transfer
*/
static inline void SDHC_SetAdmaAddress(SDHC_Type * base, uint32_t address)
{
    /* When use ADMA, disable simple DMA*/
    SDHC_WR_DSADDR(base, 0);
    SDHC_WR_ADSADDR(base, address);
}

/*!
* @brief Sets the enablement command for SDIO card.
*
* Enables or disables corresponding command for SDIO card. These command is used 
* dynamically by the user for each transaction. 
*
* @param base SDHC base address
* @param command the enablement command.
* @param enable enable or disable the corresponding command.
*/
void SDHC_SetSdioEnableCmd(SDHC_Type * base, sdhc_sdio_enable_cmd_t command, bool enable);

/*!
* @brief Restarts a transaction which has stopped at the block gap for SDIO card.
*
* @param base SDHC base address
*/
static inline void SDHC_SetContinueRequest(SDHC_Type * base)
{
    SDHC_BWR_PROCTL_CREQ(base, 1);
}


/********************Transfer related functions*********************/
/*!
* @brief Sets command to be sent.
*
* This function fills command related argument/transfer type/data block register content.
*
* @param base SDHC base address
* @param cmdReq command request structure
*/
void SDHC_SetCommand(SDHC_Type * base, const sdhc_cmd_config_t* cmdReq);

/*!
* @brief Gets the command response.
*
* @param base SDHC base address
* @param index the index of response register, range from 0 to 3
* @return response register content
*/
uint32_t SDHC_GetResponse(SDHC_Type * base, uint32_t index);

/*!
* @brief Fills the the data port.
*
* This function manily used to implement the data transfer by data port
* instead of DMA.
*
* @param base SDHC base address
* @param data the data about to be sent
*/
static inline void SDHC_SetData(SDHC_Type * base, uint32_t data)
{
    SDHC_WR_DATPORT(base, data);
}

/*!
* @brief Retrieves the data from the data port.
*
* This function manily used to implement the data transfer by data port
* instead of DMA.
*
* @param base SDHC base address
* @return the data has been read
*/
static inline uint32_t SDHC_GetData(SDHC_Type * base)
{
    return SDHC_RD_DATPORT(base);
}

/***************interrupt flag/status setting/getting related functions*****************/
/*!
* @brief Gets present sdhc's state.
*
* Gets present sdhc's state which is the logic or of some bits mask defined 
* from SDHC_CMD_INHIBIT to SDHC_DATA7_LINE_LEVEL.
*
* @param base SDHC base address
* @return present sdhc's state
*/
static inline uint32_t SDHC_GetPresentState(SDHC_Type * base)
{
    return SDHC_RD_PRSSTAT(base);
}

/*!
* @brief Enables the specified interrupts.
*
* The corresponding status register bit will generate an interrupt when the 
* corresponding interrupt signal enable bit is set. This function can set multiple
* interrupt enable bits by using the bit mask defined from SDHC_CMD_COMPLETE_INT 
* to SDHC_DMA_ERR_INT.
*
* @param base SDHC base address
* @param enable enable or disable
* @param intMask the mask to specify interrupts to be enable
*/
void SDHC_SetIntSignal(SDHC_Type * base, bool enable, uint32_t intMask);

/*!
* @brief Enables the specified interrupt state.
*
* Setting the interrupt status register bit to 1 will enable the corresponding 
* interrupt status to be set by the specified event.This function can set multiple
* interrupt state enable bits by using the bit mask defined from SDHC_CMD_COMPLETE_INT 
* to SDHC_DMA_ERR_INT.
*
* @param base SDHC base address
* @param enable enable or disable
* @param intMask the mask to specify interrupts' state to be enabled
*/
void SDHC_SetIntState(SDHC_Type * base, bool enable, uint32_t intMask);

/*! 
* @brief Gets the current interrupt status.
*
* Gets current interrupt status which is the logic or of some bits mask 
* defined from SDHC_CMD_COMPLETE_INT to SDHC_DMA_ERR_INT.
*
* @param base SDHC base address
* @return current interrupt flags
*/
static inline uint32_t SDHC_GetIntFlags(SDHC_Type * base)
{
    return SDHC_RD_IRQSTAT(base);
}

/*!
* @brief Clears a specified interrupt status.
* 
* Clears specific interrupt according to the mask which is the logic or of 
* some bits mask defined from SDHC_CMD_COMPLETE_INT to SDHC_DMA_ERR_INT.
*
* @param base SDHC base address
* @param intMask the mask to specify interrupts' flags to be cleared
*/
static inline void SDHC_ClearIntFlags(SDHC_Type * base, uint32_t intMask)
{
    SDHC_WR_IRQSTAT(base, intMask);
}

/*!
* @brief Gets the status of auto CMD12 error.
*
* Gets the auto CMD12 error status which is the logic or of some bits 
* mask defined from SDHC_ACMD12_NOT_EXEC_ERR to SDHC_ACMD12_NOT_ISSUE_ERR.
*
* @param base SDHC base address
* @return auto CMD12 error status
*/
static inline uint32_t SDHC_GetAutoCmd12ErrStatus(SDHC_Type * base)
{
    return SDHC_RD_AC12ERR(base);
}

/*!
* @brief Gets the status of ADMA error.
* 
* Gets the ADMA error status which is the logic or of some bits mask 
* defined from SDHC_ADMA_STATE_ERR to SDHC_ADMA_DESCRIPTOR_ERR
*
* @param base SDHC base address
* @return ADMA error status
*/
static inline uint32_t SDHC_GetAdmaErrStatus(SDHC_Type * base)
{
    return SDHC_RD_ADMAES(base);
}

/*!
* @brief Sets the force events according to the given mask.
* 
* Sets the force events according to the mask which the logic or of some 
* bits mask defined from SDHC_ACMD12_NOT_EXEC_ERR_EVENT to SDHC_DMA_ERROR_EVENT.
* Then corresponding bit of interrupt status register can be set.
*
* @param base SDHC base address
* @param eventMask the event mask to specify the force events' flags to be set
*/
static inline void SDHC_SetForceEventFlags(SDHC_Type * base, uint32_t eventMask)
{
    SDHC_WR_FEVT(base, mask);
}

/*************************high level transaction api**********************************/
/*!
* @brief Initializes the host.
*
* Enables clock/interrupt and configures the host.
*
* @param base SDHC base address
* @param hostConfig the host configuration
* @return kStatus_SDHC_NoError if success
*/
sdhc_status_t SDHC_InitHost(SDHC_Type * base, const sdhc_config_t* hostConfig);

/*!
* @brief Deinitializes the host.
*
* Disables the sd bus clock/host clock/interrupt etc to shoudowns the communication 
* between the host and card.
*
* @param base SDHC base address
*/ 
sdhc_status_t SDHC_DeInitHost(SDHC_Type * base);


/*@} */
#if defined(__cplusplus)
}
#endif
/*! @} */

#endif  /* FSL_FEATURE_SOC_SDHC_COUNT */

#endif /* __SDHC_H__*/
/*************************************************************************************************
 * EOF
 ************************************************************************************************/

