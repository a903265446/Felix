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


#include "ksdk_common.h"


/*!
 * @addtogroup sdhc_driver
 * @{
 */

/******************************************************************************
 * Definitions.
 *****************************************************************************/

/*! @brief Host controller capabilities flag mask */
typedef enum _sdhc_capability_flag
{
    kSDHC_SupportAdma          = SDHC_HTCAPBLT_ADMAS_MASK,   /*!< Support ADMA */
    kSDHC_SupportHighSpeed     = SDHC_HTCAPBLT_HSS_MASK,     /*!< Support high speed */
    kSDHC_SupportDma           = SDHC_HTCAPBLT_DMAS_MASK,    /*!< Support DMA */
    kSDHC_SupportSuspendResume = SDHC_HTCAPBLT_SRS_MASK,     /*!< Support suspend/resume */
    kSDHC_SupportV330          = SDHC_HTCAPBLT_VS33_MASK     /*!< Support voltage 3.3V */
#if defined FSL_FEATURE_SDHC_HAS_V300_SUPPORT && FSL_FEATURE_SDHC_HAS_V300_SUPPORT
    kSDHC_SupportV300          = SDHC_HTCAPBLT_VS30_MASK,    /*!< Support voltage 3.0V */
#endif 
#if defined FSL_FEATURE_SDHC_HAS_V180_SUPPORT && FSL_FEATURE_SDHC_HAS_V180_SUPPORT
    kSDHC_SupportV180          = SDHC_HTCAPBLT_VS18_MASK,    /*!< Support voltage 1.8V */
#endif
} sdhc_capability_flag_t;

/*! @brief Wakeup event mask */
typedef enum _sdhc_wakeup_event
{
    kSDHC_WakeupOnCardInt       = SDHC_PROCTL_WECINT_MASK,   /*!< Wakeup on card interrupt */
    kSDHC_WakeupOnCardInsert    = SDHC_PROCTL_WECINS_MASK,   /*!< Wakeup on card insertion */
    kSDHC_WakeupOnCardRemove    = SDHC_PROCTL_WECRM_MASK     /*!< Wakeup on card removal */
} sdhc_wakeup_event_t;

/*! @brief Reset type mask */
typedef enum _sdhc_reset
{
    kSDHC_ResetAll      = SDHC_SYSCTL_RSTA_MASK,  /*!< Reset all except card detection */
    kSDHC_ResetCommand      = SDHC_SYSCTL_RSTC_MASK,  /*!< Reset command line */
    kSDHC_ResetData     = SDHC_SYSCTL_RSTD_MASK,  /*!< Reset data line */
} sdhc_reset_t;

/*! @brief Auto gated clock type mask */
typedef enum _sdhc_auto_gate_clock
{
    kSDHC_AutoGateClockPeriph    = SDHC_SYSCTL_PEREN_MASK,
    kSDHC_AutoGateClockSystem    = SDHC_SYSCTL_HCKEN_MASK,
    kSDHC_AutoGateClockIPG       = SDHC_SYSCTL_IPGEN_MASK,
} sdhc_auto_gate_clock_t;

/*! @brief Transfer flag mask */
typedef enum _sdhc_transfer_flag
{
    kSDHC_EnableDma            = SDHC_XFERTYP_DMAEN_MASK,  /*!< Enable DMA */ 

    kSDHC_CommandTypeSuspend       = (SDHC_XFERTYP_CMDTYP(1U)),/*!< Suspend command */
    kSDHC_CommandTypeResume        = (SDHC_XFERTYP_CMDTYP(2U)),/*!< Resume command */
    kSDHC_CommandTypeAbort         = (SDHC_XFERTYP_CMDTYP(3U)),/*!< Abort command */

    kSDHC_EnableBlockCount     = SDHC_XFERTYP_BCEN_MASK,   /*!< Enable block count */
    kSDHC_EnableAutoCMD12      = SDHC_XFERTYP_AC12EN_MASK, /*!< Enable auto CMD12 */
    kSDHC_EnableDataRead       = SDHC_XFERTYP_DTDSEL_MASK, /*!< Enable data read */
    kSDHC_MultipleBlock        = SDHC_XFERTYP_MSBSEL_MASK, /*!< Multiple block data read/write */

    kSDHC_ResponseLength136    = SDHC_XFERTYP_RSPTYP(1),   /*!< 136 bit response length */
    kSDHC_ResponseLength48     = SDHC_XFERTYP_RSPTYP(2),   /*!< 48 bit response length */
    kSDHC_ResponseLength48Busy = SDHC_XFERTYP_RSPTYP(3);   /*!< 48 bit response length with busy status */
} sdhc_transfer_flag_t;

/*! @brief Present state flag mask */
typedef enum _sdhc_present_state
{
    kSDHC_CommandInhibit                 = SDHC_PRSSTAT_CIHB_MASK,  /*!< Command inhibit */
    kSDHC_DataInhibit                = SDHC_PRSSTAT_CDIHB_MASK, /*!< Data inhibit */
    kSDHC_DatLineActive              = SDHC_PRSSTAT_DLA_MASK,   /*!< Data line active */
    kSDHC_SdClockStable              = SDHC_PRSSTAT_SDSTB_MASK, /*!< SD bus clock stable */
    kSDHC_WriteTransferActive        = SDHC_PRSSTAT_WTA_MASK,   /*!< Write transfer active */
    kSDHC_ReadTransferActive         = SDHC_PRSSTAT_RTA_MASK,   /*!< Read transfer active */
    kSDHC_BufferWriteEnable          = SDHC_PRSSTAT_BWEN_MASK,  /*!< Buffer write enable */
    kSDHC_BufferReadEnable           = SDHC_PRSSTAT_BREN_MASK,  /*!< Buffer read enable */
    kSDHC_CardInserted               = SDHC_PRSSTAT_CINS_MASK,  /*!< Card inserted */
    kSDHC_CommandLineLevel               = SDHC_PRSSTAT_CLSL_MASK,  /*!< Command line signal level */
    kSDHC_Data0LineLevel             = (1 << 24U),              /*!< Data0 line signal level */
    kSDHC_Data1LineLevel             = (1 << 25U),              /*!< Data1 line signal level */
    kSDHC_Data2LineLevel             = (1 << 26U),              /*!< Data2 line signal level */
    kSDHC_Data3LineLevel             = (1 << 27U),              /*!< Data3 line signal level */
    kSDHC_Data4LineLevel             = (1 << 28U),              /*!< Data4 line signal level */
    kSDHC_Data5LineLevel             = (1 << 29U),              /*!< Data5 line signal level */
    kSDHC_Data6LineLevel             = (1 << 30U),              /*!< Data6 line signal level */
    kSDHC_Data7LineLevel             = (1 << 31U),              /*!< Data7 line signal level */
} shdc_present_state_t;

/*! @brief Interrupt flag mask */
typedef enum _sdhc_irq_flag
{
    kSDHC_CommandComplete        = SDHC_IRQSTAT_CC_MASK,   /*!< Command complete */
    kSDHC_DataComplete       = SDHC_IRQSTAT_TC_MASK,   /*!< Data complete */
    kSDHC_BlockGapEvent      = SDHC_IRQSTAT_BGE_MASK,  /*!< Block gap event */
    kSDHC_DmaComplete        = SDHC_IRQSTAT_DINT_MASK, /*!< DMA interrupt */
    kSDHC_BuffWriteReady     = SDHC_IRQSTAT_BWR_MASK,  /*!< Buffer write ready */
    kSDHC_BuffReadReady      = SDHC_IRQSTAT_BRR_MASK,  /*!< Buffer read ready */
    kSDHC_CardInsertion      = SDHC_IRQSTAT_CINS_MASK, /*!< Card inserted */
    kSDHC_CardRemoval        = SDHC_IRQSTAT_CRM_MASK,  /*!< Card removed */
    kSDHC_CardInt            = SDHC_IRQSTAT_CINT_MASK, /*!< Card interrupt */
    kSDHC_CommandTimeout         = SDHC_IRQSTAT_CTOE_MASK, /*!< Command timeout error */
    kSDHC_CommandCrcErr          = SDHC_IRQSTAT_CCE_MASK,  /*!< Command CRC error */
    kSDHC_CommandEndBitErr       = SDHC_IRQSTAT_CEBE_MASK, /*!< Command end bit error */
    kSDHC_CommandIndexErr        = SDHC_IRQSTAT_CIE_MASK,  /*!< Command index error */
    kSDHC_DataTimeout        = SDHC_IRQSTAT_DTOE_MASK, /*!< Data timeout error */
    kSDHC_DataCrcErr         = SDHC_IRQSTAT_DCE_MASK,  /*!< Data CRC error */
    kSDHC_DataEndBitErr      = SDHC_IRQSTAT_DEBE_MASK, /*!< Data end bit error */ 
    kSDHC_AutoCMD12Err       = SDHC_IRQSTAT_AC12E_MASK,/*!< Auto CMD12 error */
    kSDHC_DmaErr             = SDHC_IRQSTAT_DMAE_MASK, /*!< DMA error */

    kSDHC_CommandErr             = (kSDHC_CommandTimeoutErr | \
                                kSDHC_CommandCrcErr | \
                                kSDHC_CommandEndBitErr | \
                                kSDHC_CommandIndexErr),    /*!< All command error interrupts */
    kSDHC_DataErr            = (kSDHC_DataTimeoutErr | \
                                kSDHC_DataCrcErr | \
                                kSDHC_DataEndBitErr),  /*!< All data error interrupts */
    kSDHC_Err                = (kSDHC_CommandErr | \
                                kSDHC_DataErr | \
                                kSDHC_AutoCMD12Err | \
                                kSDHC_DmaErr),         /*!< All SDHC error */
    kSDHC_DataInt            = (kSDHC_DataErr | \
                                kSDHC_DataComplete | \
                                kSDHC_BuffReadReady | \
                                kSDHC_BuffWriteReady | \
                                kSDHC_DmaErr | \
                                kSDHC_DmaComplete),    /*!<All data interrupts */ 
    kSDHC_CommandInt             = (kSDHC_CommandErr | \
                                kSDHC_CommandComplete | \
                                kSDHC_AutoCMD12Err),   /*!< All command interrupts */
    kSDHC_CardDetectInt      = (kSDHC_CardInsertion | \
                                kSDHC_CardRemoval),    /*!< All card detection interrups */
} sdhc_irq_flag_t;

/*! @brief Auto CMD12 error status flag mask */
typedef enum _sdhc_acommand12_err
{
    kSDHC_Acommand12NotExec          = SDHC_AC12ERR_AC12NE_MASK,   /*!< Not executed error */
    kSDHC_Acommand12Timeout          = SDHC_AC12ERR_AC12TOE_MASK,  /*!< Timeout error */
    kSDHC_Acommand12EndBitErr        = SDHC_AC12ERR_AC12EBE_MASK,  /*!< End bit error */
    kSDHC_Acommand12CrcErr           = SDHC_AC12ERR_AC12CE_MASK,   /*!< CRC error */
    kSDHC_Acommand12IndexErr         = SDHC_AC12ERR_AC12IE_MASK,   /*!< Index error */
    kSDHC_Acommand12NotIssued        = SDHC_AC12ERR_CNIBAC12E_MASK,/*!< Not issued error */
} sdhc_acommand12_err_t;

/*! @brief ADMA error status flag mask */
typedef enum _sdhc_amda_err
{
    kSDHC_AdmaState            = SDHC_ADMAES_ADMAES_MASK,    /*!< Error state */
    kSDHC_LenghMismatch        = SDHC_ADMAES_ADMALME_MASK,   /*!< Length mismatch error */
    kSDHC_DescriptorErr        = SDHC_ADMAES_ADMADCE_MASK,   /*!< Descriptor error */
} sdhc_adma_err_t;

/*!@brief SDHC status */
enum _sdhc_status {
    kStatus_SDHC_OutOfMemory                 = MAKE_STATUS(kStatusGroup_SDHC, 2),/*!< No enough memory */
    kStatus_SDHC_PrepareDmaDescriptorFailed  = MAKE_STATUS(kStatusGroup_SDHC, 3),/*!< Prepair DMA descritor failed */
    kStatus_SDHC_NoCardInsertedError         = MAKE_STATUS(kStatusGroup_SDHC, 4),/*!< No inserted card */
    kStatus_SDHC_CreateEventFailed           = MAKE_STATUS(kStatusGroup_SDHC, 7),/*!< Create event failed */
    kStatus_SDHC_WaitEventFailed             = MAKE_STATUS(kStatusGroup_SDHC, 8),/*!< Wait event failed */
    kStatus_SDHC_NotifyEventFailed           = MAKE_STATUS(kStatusGroup_SDHC, 9),/*!< Notify event failed */
    kStatus_SDHC_GetCurrentTimeFaild         = MAKE_STATUS(kStatusGroup_SDHC, 10),/*!< Get current time failed */
};

/*! 
 * @brief Adma error state 
 *
 * This state is the detail state when ADMA error has occurred.
 */
typedef enum _sdhc_adma_err_state
{
    kSDHC_AdmaErrStateStopDma,         /*!< Stop DMA */
    kSDHC_AdmaErrStateFetchDescriptor, /*!< Fetch descriptor */
    kSDHC_AdmaErrStateChangeAddress,   /*!< Change address */
    kSDHC_AdmaErrStateTransferData,    /*!< Transfer data */
} sdhc_adma_err_state_t;

/*! @brief Force event flag mask */
typedef enum _sdhc_force_event
{
    kSDHC_ForceAutoCMD12NotExec   = SDHC_FEVT_AC12NE_MASK,    /*!< Auto CMD12 not executed error */
    kSDHC_ForceAutoCMD12Timeout   = SDHC_FEVT_AC12TOE_MASK,   /*!< Auto CMD12 timeout error */
    kSDHC_ForceAutoCMD12CrcErr    = SDHC_FEVT_AC12CE_MASK,    /*!< Auto CMD12 CRC error */
    kSDHC_ForceEndBitErr          = SDHC_FEVT_AC12EBE_MASK,   /*!< Auto CMD12 end bit error */
    kSDHC_ForceAutoCMD12IndexErr  = SDHC_FEVT_AC12IE_MASK,    /*!< Auto CMD12 index error */
    kSDHC_ForceAutoCMD12NotIssued = SDHC_FEVT_CNIBAC12E_MASK, /*!< Auto CMD12 not issued error */
    kSDHC_ForceCommandTimeout         = SDHC_FEVT_CTOE_MASK,      /*!< Command timeout error */
    kSDHC_ForceCommandCrcErr          = SDHC_FEVT_CCE_MASK,       /*!< Command CRC error */
    kSDHC_ForceCommandEndBitErr       = SDHC_FEVT_CEBE_MASK,      /*!< Command end bit error */
    kSDHC_ForceCommandIndexErr        = SDHC_FEVT_CIE_MASK,       /*!< Command index error */
    kSDHC_ForceDataTimeout        = SDHC_FEVT_DTOE_MASK,      /*!< Data timeout error */
    kSDHC_ForceDataCrcErr         = SDHC_FEVT_DCE_MASK,       /*!< Data CRC error */
    kSDHC_ForceDataEndBitErr      = SDHC_FEVT_DEBE_MASK,      /*!< Data end bit error */
    kSDHC_ForceAutoCMD12Err       = SDHC_FEVT_AC12E_MASK,     /*!< Auto CMD12 error */
    kSDHC_ForceCardInt            = SDHC_FEVT_CINT_MASK,      /*!< Card interrupt */
    kSDHC_ForceDmaErr             = SDHC_FEVT_DMAE_MASK,      /*!< Dma error */
} sdhc_force_event_t;

/*! @brief Data transfer width */
typedef enum _sdhc_data_width 
{
    kSDHC_DataWidth1Bit = 0U,  /*!< 1-bit mode */
    kSDHC_DataWidth4Bit = 1U,  /*!< 4-bit mode */
    kSDHC_DataWidth8Bit = 2U,  /*!< 8-bit mode */
} sdhc_data_width_t;

/*! @brief Endian mode */
typedef enum _sdhc_endian_mode
{
    kSDHC_EndianModeBig            = 0U, /*!< Big endian mode */
    kSDHC_EndianModeHalfWordBig    = 1U, /*!< Half word big endian mode */
    kSDHC_EndianModeLittle         = 2U, /*!< Little endian mode */
} sdhc_endian_mode_t;

/*! @brief DMA mode */
typedef enum _sdhc_dma_mode 
{
    kSDHC_DmaModeNoOrSimple = 0U, /*!< No DMA or simple DMA is selected */
    kSDHC_DmaModeAdma1      = 1U, /*!< ADMA1 is selected */
    kSDHC_DmaModeAdma2      = 2U, /*!< ADMA2 is selected */
} sdhc_dma_mode_t;

/*! @brief SDIO control */
typedef enum _sdhc_sdio_control
{
    kSDHC_StopAtBlockGap      = 0x01,   /*!< Stop at block gap */
    kSDHC_ReadWaitControl     = 0x02,   /*!< Read wait control */
    kSDHC_IntAtBlockGap       = 0x04,   /*!< Interrupt at block gap */
    kSDHC_ExactBlockNumRead   = 0x08,   /*!< Exact block number read */
} sdhc_sdio_control_t;

/*! @brief MMC card boot mode */
typedef enum _sdhc_boot_mode
{
    kSDHC_BootModeNormal = 0U,  /*!< Normal boot */
    kSDHC_BootModeAlter  = 1U,  /*!< Alternative boot */ 
} sdhc_boot_mode_t;

/*! @brief Card detection way */
typedef enum _sdhc_card_detect 
{
    kSDHC_CardDetectGpio     = 1U,/*!< Use GPIO for card detection. */
    kSDHC_CardDetectDat3     = 2U,/*!< Use DAT3 for card detection. */
    kSDHC_CardDetectCD       = 3U,/*!< Use dedicate CD pin for card detection. */
} sdhc_card_detect_t;

/*! @brief Transfer mode */
typedef enum _sdhc_transfer_mode 
{
    kSDHC_TransferByDataPortPolling   = 1U,   /*!< Polling based Data Port */
    kSDHC_TransferByDataPortIRQ       = 2U,   /*!< Interrupt Data Port */
    kSDHC_TransferByAdma1Polling      = 3U,   /*!< Polling based ADMA1 */
    kSDHC_TransferByAdma1IRQ          = 4U,   /*!< Interrupt based ADMA1 */
    kSDHC_TransferByAdma2Polling      = 5U,   /*!< Polling based ADMA2 */
    kSDHC_TransferByAdma2IRQ          = 6U,   /*!< Interrupt based ADMA2 */
} sdhc_transfer_mode_t;

/*! 
 * @brief The command response type. 
 *
 * Defines the command response type from card to host controller.
 */
typedef enum _sdhc_response_type 
{
    kSDHC_ResponseTypeNone = 0U,         /*!< Response type: none */
    kSDHC_ResponseTypeR1   = 1U,         /*!< Response type: R1 */
    kSDHC_ResponseTypeR1b  = 2U,         /*!< Response type: R1b */
    kSDHC_ResponseTypeR2   = 3U,         /*!< Response type: R2 */
    kSDHC_ResponseTypeR3   = 4U,         /*!< Response type: R3 */
    kSDHC_ResponseTypeR4   = 5U,         /*!< Response type: R4 */
    kSDHC_ResponseTypeR5   = 6U,         /*!< Response type: R5 */
    kSDHC_ResponseTypeR5b  = 7U,         /*!< Response type: R5b */
    kSDHC_ResponseTypeR6   = 8U,         /*!< Response type: R6 */
    kSDHC_ResponseTypeR7   = 9U,         /*!< Response type: R7 */
} sdhc_response_type_t;

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
#define SDHC_ADMA1_DESC_TYPE_TRAN            (SDHC_ADMA1_DESC_ACT2_MASK | \
                                              SDHC_ADMA1_DESC_VALID_MASK)
#define SDHC_ADMA1_DESC_TYPE_LINK            (SDHC_ADMA1_DESC_ACT1_MASK | \
                                              SDHC_ADMA1_DESC_ACT2_MASK | \
                                              SDHC_ADMA1_DESC_VALID_MASK)
#define SDHC_ADMA1_DESC_TYPE_SET             (SDHC_ADMA1_DESC_ACT1_MASK | \
                                              SDHC_ADMA1_DESC_VALID_MASK)
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
typedef struct _sdhc_adma2_descriptor {
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
#define SDHC_ADMA2_DESC_TYPE_RCV             (SDHC_ADMA2_DESC_ACT1_MASK | \
                                              SDHC_ADMA2_DESC_VALID_MASK)
#define SDHC_ADMA2_DESC_TYPE_TRAN            (SDHC_ADMA2_DESC_ACT2_MASK | \
                                              SDHC_ADMA2_DESC_VALID_MASK)
#define SDHC_ADMA2_DESC_TYPE_LINK            (SDHC_ADMA2_DESC_ACT1_MASK | \
                                              SDHC_ADMA2_DESC_ACT2_MASK | \
                                              SDHC_ADMA2_DESC_VALID_MASK)
#define SDHC_ADMA2_DESC_LEN_SHIFT            (16U)
#define SDHC_ADMA2_DESC_LEN_MASK             (0xFFFFU)
#define SDHC_ADMA2_DESC_MAX_LEN_PER_ENTRY    (SDHC_ADMA2_DESC_LEN_MASK)

/*! @brief ADMA table configration. */
typedef struct _sdhc_adma_table_config
{
    sdhc_dma_mode_t dmaMode;         /*!< DMA mode */
    uint32_t *tableAddress;          /*!< ADMA table address */
    uint32_t maxTableEntries;        /*!< Max table entries allcated by user */

    uint32_t *dataBuffer;            /*!< Data buffer address */
    uint32_t dataLength;             /*!< Data length */    
} sdhc_adma_table_config_t;

/*! 
 * @brief SDHC capability information.  
 *
 * Defines structure to save the capability information of SDHC in the initialization phrase.
 *
 * @note The detail capability flag mask can be got according to sdhc_capability_flag_t.
 */
typedef struct _sdhc_capability
{
    uint32_t sourceClockFreq; /*!< Source clock frequency */
    uint32_t specVersion;     /*!< Specification version */
    uint32_t vendorVersion;   /*!< Verdor version */
    uint32_t maxBlockLength;  /*!< Max block length united as byte */
    uint32_t flags;           /*!< Capability flags to indicate the support information*/
} sdhc_capability_t;

/*! @brief SD bus clock(SD_CLK) configuration */
typedef struct _sdhc_sd_clock_config
{
    bool enableSdClock;      /*!< Enable or disable the SD_CLK */
    uint32_t baseClockFreq;  /*!< SDHC module clock which is base clock of SD_CLK */
    uint32_t sdClockFreq;    /*!< Dest SD_CLK clock frequence want to set */
} sdhc_sd_clock_config_t;

/*! @brief Card transfer configuration. 
 *
 * Defines structure to configure transfer related command index/argument/flags and data block 
 * size/data block numbers. This structure need to be filled when each time send command to the card. 
 *
 * @note The detail command flag mask can be got from sdhc_transfer_flag_t.
 */
typedef struct _sdhc_transfer_config
{
    uint32_t dataBlockSize;    /*!< Command associated data block size */
    uint32_t dataBlockCount;   /*!< Command associated data block count */
    uint32_t commandArgument;      /*!< Command argument */
    uint32_t commandIndex;         /*!< Command index */
    uint32_t flags;            /*!< Command flags */
} sdhc_transfer_config_t;

/*! @brief Data structure to configure the MMC boot feature */
typedef struct _sdhc_boot_config
{
    uint32_t ackTimeoutCount;      /*!< Timeout value for the boot Ack */
    sdhc_boot_mode_t bootMode;         /*!< Boot mode selection. */
    uint32_t blockCount;           /*!< Stop at block gap value of automatic mode */
    bool enableBootAck;            /*!< Enable or disable boot ACK */
    bool enableBoot;               /*!< Enable or disable fast boot */
    bool enableAutoStopAtBlockGap; /*!< Enable or disable auto stop at block gap fucntion in boot period */
} sdhc_boot_config_t;

/*! 
 * @brief Data structure to initialize the SDHC 
 */ 
typedef struct _sdhc_config
{
    sdhc_card_detect_t cardDetectMode; /*!< Card detection mode */
    bool enableAutoCMD12;              /*!< Enable or disable auto send CMD12 */
    sdhc_endian_mode_t endianMode; /*!< Endian mode */
    sdhc_dma_mode_t dmaMode;       /*!< DMA mode */
    uint32_t readWatermarkLevel;   /*!< Watermark level for DMA read operation */
    uint32_t writeWatermarkLevel;  /*!< Watermark level for DMA write opration */
} sdhc_config_t;

/*!
 * @brief Card data descriptor
 *
 * Defines structure to contain data related attribute.
 */
typedef struct _sdhc_data
{
    uint32_t blockSize;             /*!< Block size */
    uint32_t blockCount;            /*!< Block count */
    uint32_t bytesTransferred;      /*!< Transferred byte count */
    uint32_t *buffer;               /*!< Data buffer */

    bool isRead;                    /*!< Data direction */
} sdhc_data_t;

/*!
 * @brief Card command descriptor
 *
 * Defines card command related attribute.
 */
typedef struct _sdhc_command
{
    uint32_t index;                   /*!< Command index */
    uint32_t argument;                /*!< Command argument */
    bool isAbortCommand;              /* Request to abort transaction */ 

    sdhc_response_type_t responseType;/*!< Command response type */
    uint32_t response[4];             /*!< Response for this command */
} sdhc_command_t;

/*! 
 * @brief Host desciptor
 *
 * Defines the structure to save the host state information and callback function
 *
 * @note The memory and wait command/data function must be allocated by the user.
 */
typedef struct _sdhc_host
{
    /* Data */
    SDHC_Type *base;
    sdhc_config_t *sdhcConfig;
    
    /* Memory allocated by user */
    sdhc_capability_t *capability;     /*!< Capability information */
    uint32_t *admaTable;               /*!< ADMA table address */
    uint32_t admaTableMaxEntries;      /*!< Items count in ADMA table */    
    
    /* Transaction state */
    volatile sdhc_command_t *currentCommand;            /*!< Command is sending */
    volatile sdhc_data_t *currentData;          /*!< Data is transferring */
    volatile uint32_t lastIrqFlags;             /*!< Irq flags of last transaction */

    /* Wait command/data function */
    bool (*CreateCommandEvent)();                     /*!< Create command event */
    bool (*WaitCommandEvent)(uint32_t timeout);       /*!< Wait command event */
    bool (*NotifyCommandEvent)();                     /*!< Notify command event */
    bool (*DeleteCommandEvent)();                     /*!< Delete command event */
    bool (*CreateDataEvent)();                    /*!< Create data event */
    bool (*WaitDataEvent)(uint32_t timeout);      /*!< Wait data event */
    bool (*NotifyDataEvent)();                    /*!< Notify data event */ 
    bool (*DeleteDataEvent)();                    /*!< Delete data event */
    uint32_t (*getCurrentTimeMsec)();             /*!< Get current miliseconds time */
    uint32_t timeRangeMsec;                       /*!< Time range in miliseconds */

    /* Callback function */
    void (*cardIntCallback)(SDHC_Type *base);     /*!< Card interrupt occurs */
    void (*cardInsertedCallback)(SDHC_Type *base);/*!< Card inserted occurs */
    void (*cardRemovedCallback)(SDHC_Type *base); /*!< Card removed occurs */
    void (*blockGapCallback)(SDHC_Type *base);    /*!< Block gap occurs */

    /* Maps the actual transfer API. */
    status_t (*SendCommand)(SDHC_Type *base, sdhc_host_t *host, uint32_t timeoutInMs));
    status (*TransferData)(SDHC_Type *base, sdhc_host_t *host, uint32_t timeoutInMs);
} sdhc_host_t;

/* Check if host support switching the card to high speed mode */
#define DOES_SDHC_SUPPORT_HIGHSPEED(x)      (x->capability->flags & SDHC_SUPPORT_HIGHSPEED)

/* eSDHC on all kinetis boards will support 4 bit data bus. */
#define DOES_SDHC_SUPPORT_4BITS(x)          true
#define DOES_SDHC_SUPPORT_8BITS(x)          true


/*************************************************************************************************
 * API
 ************************************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @name SDHC Init, DeInit and Reset
 * @{
 */

/*!
 * @brief Gets driver version.
 *
 * This function gets the driver version of SDHC.
 *
 * @param version The version structure.
 */
void SDHC_GetVersion(version_t version);

/*!
 * @brief SDHC module intialization function.
 * 
 * Configures the SDHC according to the user configuration and gets the SDHC 
 * capability information.
 *
 * Example:
   @code 
   sdhc_config_t config;
   config.enableDat3AsCDPin = false;
   config.endianMode = kSDHC_EndianModeLittle;
   config.dmaMode = kSDHC_DmaModeAdma2;
   config.readWatermarkLevel = 512;
   config.writeWatermarkLevel = 512;
   sdhc_capability_t capability;
   SDHC_Init(SDHC, &config, 100000, &capability)
   @endcode
 *
 * @param base SDHC base point.
 * @param configPtr SDHC configuration information.
 * @param sourceClock SD bus frequency in HZ.
 * @param capabilityPtr The structure to save capability.
 */
void SDHC_Init(SDHC_Type *base, const sdhc_config_t *config, uint32_t sourceClock, 
       sdhc_capability_t *capability);

/*!
 * @brief Deinitializes the SDHC.
 * 
 * @param base SDHC base point.
 */
void SDHC_DeInit(SDHC_Type *base);

/*!
 * @brief Resets the SDHC.
 *
 * @param base SDHC base point.
 * @param mask The mask of reset type.
 * @param timeout Timeout for reset.
 * @return 0 on success, else on error.
 */
uint32_t SDHC_Reset(SDHC_Type *base, uint32_t mask, uint32_t timeout);


/*!
 * @name SDHC Dynamical setting
 * @{
 */

/*!
 * @brief Sets the auto gated off feature of SDHC internal clocks.
 *
 * @param base SDHC base point.
 * @param mask The auto gate clock type mask(sdhc_auto_gate_clock_t).
 */
void SDHC_SetClockAutoGated(SDHC_Type *base, uint32_t mask);

/*!
 * @brief Sets wakeup event in low power mode
 *
 * This function sets wakeup event according to the mask defined in sdhc_wakeup_event_t.
 * 
 * @param base SDHC base point.
 * @param mask Wakeup event mask(sdhc_wakeup_event_t).
 */
void SDHC_SetWakeupEvent(SDHC_Type *base, uint32_t mask);

/*!
 * @brief Sets the clock frequence of SD_CLK pin.
 *
 * This function defines working clock of the SD bus.   
 *
 * Example:
   @code
   sdhc_sd_clock_config_t sdClockConfig;
   sdClockConfig.enableSdClock = true;
   sdClockConfig.baseClockFreq = 180000000;
   sdClockConfig.sdClockFreq = 400000;
   SDHC_SetBusClockConfig(SDHC, &sdClockConfig);
   @endcode
 * 
 * @param base SDHC base point.
 * @param sdClockConfig SD bus clock configuration.
 */
void SDHC_SetSdClockConfig(SDHC_Type *base, const sdhc_sd_clock_config_t *config);

/*!
 * @brief Sends 80 clocks to the card to set it to be active state.
 *
 * @param base SDHC base point.
 * @param timeout Timeout to initialize card.
 * @return 0 on success, else on error.
 */
uint32_t SDHC_SetCardActive(SDHC_Type *base, uint32_t timeout);

/*!
 * @brief Sets the data transfer width.
 *
 * @param base SDHC base point.
 * @param dataWidth Data transfer width.
 */
void SDHC_SetDataWidth(SDHC_Type *base, sdhc_data_width_t dataWidth);

/*!
 * @brief Enables card detection level for test.
 *
 * @param base SDHC base point.
 */
static inline void SDHC_EnableCDForTest(SDHC_Type *base)
{
    base->PROCTL |= SDHC_PROCTL_CDSS_MASK; 
}

/*!
 * @brief Disables card detection level for test.
 *
 * @param base SDHC base point.
 */
static inline void SDHC_DisableCDForTest(SDHC_Type *base)
{
    base->PROCTL &= ~SDHC_PROCTL_CDSS_MASK;
}

/*!
 * @brief Sets card detection test level.
 * 
 * This function sets the card detection test level to indicate whether the card
 * is inserted into SDHC when DAT[3] or CD pin is selected as card detection pin.
 * This function can also assert the GPIO pin when CD pin is select as the card 
 * detection pin.
 *
 * @param base SDHC base point.
 * @param high True to set the card detect level to high.
 */
void SDHC_SetCDTestLevel(SDHC_Type *base, bool high);

/*! 
 * @brief Enables SDIO card control.
 *
 * @param base SDHC base point.
 * @param mask SDIO card control flag mask(sdhc_sdio_control_t).
 */
void SDHC_EnableSdioControl(SDHC_Type *base, uint32_t mask);

/*! 
 * @brief Disables SDIO card control.
 *
 * @param base SDHC base point.
 * @param mask SDIO card control flag mask(sdhc_sdio_control_t).
 */
void SDHC_DisableSdioControl(SDHC_Type *base, uint32_t mask);

/*!
 * @brief Restarts a transaction which has stopped at the block gap for SDIO card.
 *
 * @param base SDHC base point.
 */
static inline void SDHC_SetContinueRequest(SDHC_Type *base)
{
    base->PROCTL |= SDHC_PROCTL_CREQ_MASK;
}

/*!
 * @brief Configures the MMC boot feature.
 *
 * Example:
   @code
   sdhc_boot_config_t bootConfig;
   bootConfig.ackTimeoutCount = 4;
   bootConfig.bootMode = kSDHC_BootModeNormal;
   bootConfig.blockCount = 5;
   bootConfig.enableBootAck = true;
   bootConfig.enableBoot = true;
   enableBoot.enableAutoStopAtBlockGap = true;
   SDHC_SetMmcBootConfig(SDHC, &bootConfig);
   @endcode
 *
 * @param base SDHC base point.
 * @param config The MMC boot configuration information.
 */
void SDHC_SetMmcBootConfig(SDHC_Type *base, const sdhc_boot_config_t *config);

/*!
 * @name SDHC Command and Data read/write
 * @{ 
 */

/*!
 * @brief Sets card transfer related properties.
 *
 * This function fills card transfer related command argument/transfer flag/data size. Command and data
 * will be sent by SDHC after calling this function.
 *
 * Example:
   @code
   sdhc_transfer_config_t transferConfig;
   transferConfig.dataBlockSize = 512;
   transferConfig.dataBlockCount = 2;
   transferConfig.commandArgument = 0x01AA;
   transferConfig.commandIndex = 8;
   transferConfig.flags |= (kSDHC_EnableDma | kSDHC_EnableAutoCMD12 | kSDHC_MultipleBlock);
   SDHC_SetTransferConfig(SDHC, &transferConfig);
   @endcode
 *
 * @param base SDHC base point.
 * @param config Command configuration structure.
 */
void SDHC_SetTransferConfig(SDHC_Type *base, const sdhc_transfer_config_t *config);

/*!
 * @brief Gets the command response.
 *
 * @param base SDHC base point.
 * @param index The index of response register, range from 0 to 3.
 * @return Response register content.
 */
static inline uint32_t SDHC_GetCommandResponse(SDHC_Type *base, uint32_t index)
{
    assert(index < 4);

    return SDHC_RD_CMDRSP(base, index);
}

/*!
 * @brief Fills the the data port.
 *
 * This function manily used to implement the data transfer by Data Port
 * instead of DMA.
 *
 * @param base SDHC base point.
 * @param data The data about to be sent.
 */
static inline void SDHC_SetData(SDHC_Type *base, uint32_t data)
{
    base->DATAPORT = data;
}

/*!
 * @brief Retrieves the data from the data port.
 *
 * This function is manily used to implement the data transfer by Data Port
 * instead of DMA.
 *
 * @param base SDHC base point.
 * @return The data has been read.
 */
static inline uint32_t SDHC_GetData(SDHC_Type *base)
{
    return base->DATAPORT;
}

void SDHC_SetAdmaTableConfig(SDHC_Type *base, const sdhc_adma_table_config_t *config);

/*!
 * @name Interrupt and status
 * @{ 
 */

/*!
 * @brief Gets present sdhc's state.
 *
 * @param base SDHC base point.
 * @return Present sdhc's state.
 */
static inline uint32_t SDHC_GetPresentState(SDHC_Type *base)
{
    return base->PRSSTAT;
}

/*!
 * @brief Enables the specified interrupts.
 *
 * @param base SDHC base point.
 * @param mask The interrupt signal flag mask(sdhc_irq_flag_t).
 */
static inline void SDHC_EnableIntSignal(SDHC_Type *base, uint32_t mask)
{
    base->IRQSIGEN |= mask;
}

/*!
 * @brief Disables the specified interrupts.
 *
 * @param base SDHC base point.
 * @param mask The interrupt signal flag mask(sdhc_irq_flag_t).
 */
static inline void SDHC_DisableIntSignal(SDHC_Type *base, uint32_t mask)
{
    base->IRQSIGEN &= ~mask;
}

static inline void SDHC_GetEnabledIntStatus(SDHC_Type *base, uint32_t mask)
{
    return base->IRQSTATEN;
}

/*!
 * @brief Disables the interrupts state.
 *
 * @param base SDHC base point.
 * @param mask The interrupt status flag mask(sdhc_irq_flag_t).
 */
static inline void SDHC_DisableIntStatus(SDHC_Type *base, uint32_t mask)
{
    base->IRQSTATEN &= ~mask;
}

/*! 
 * @brief Gets the current interrupt status.
 *
 * @param base SDHC base point.
 * @return Current interrupt status flags mask(sdhc_irq_flag_t).
 */
static inline uint32_t SDHC_GetIntStatusFlags(SDHC_Type *base)
{
    return base->IRQSTAT;
}

/*!
 * @brief Clears a specified interrupt status.
 * 
 * Clears specific interrupt according to the mask which is the logic or of 
 * some bits mask defined from SDHC_CMD_COMPLETE_INT to SDHC_DMA_ERR_INT.
 *
 * @param base SDHC base point.
 * @param mask The interrupt status flag mask(sdhc_irq_flag_t).
 */
static inline void SDHC_ClearIntStatusFlags(SDHC_Type *base, uint32_t mask)
{
    base->IRQSTAT = mask;
}

/*!
 * @brief Gets the status of auto CMD12 error.
 *
 * @param base SDHC base point.
 * @return Auto CMD12 error status flag mask(sdhc_acommand12_err_t).
 */
static inline uint32_t SDHC_GetAcommand12ErrStatusFlags(SDHC_Type *base)
{
    return base->AC12ERR;
}

/*!
 * @brief Gets the status of ADMA error.
 *
 * @param base SDHC base point.
 * @return ADMA error status flag mask(sdhc_adma_err_t).
 */
static inline uint32_t SDHC_GetAdmaErrStatusFlags(SDHC_Type *base)
{
    return base->ADMAES;
}

/*!
 * @brief Sets the force events according to the given mask.
 *
 * @param base SDHC base point.
 * @param mask The force event flag mask(sdhc_force_event_t).
 */
static inline void SDHC_SetForceEvent(SDHC_Type *base, uint32_t mask)
{
    base->FEVT = mask;
}

/*!
 * @name Transaction API
 * @{ 
 */

/*!
 * @brief Regists a host descriptor.
 *
 * Regists state, callback and set card detection/data transfer way for the host controller
 * 
 * @param host The host descriptor.
 * @return kStatus_Success if no error.
 */
status_t SDHC_RegistHost(sdhc_host_t *host)

/*!
 * @brief Sends a command using polling.
 *
 * This function waits until the command response is got or encounter error by polling the 
 * status flag.
 *
 * @param host The host descriptor.
 * @param timeoutInMs The timeout time in miliseconds.
 * @return kStatus_Success if no error.
 */
status_t SDHC_SendCommandPolling(sdhc_host_t *host, uint32_t timeoutInMs);

/*!
 * @brief Sends a command using IRQ.
 *
 * This function waits until the command response is got or encounter error by waiting event
 * notification in the ISR.
 *
 * @param host The host descriptor.
 * @param timeoutInMs The timeout time in miliseconds.
 * @return kStatus_Success if no error.
 */
status_t SDHC_SendCommandIRQ(sdhc_host_t *host, uint32_t timeoutInMs);

/*!
 * @brief Transfers data until the data transfer complete.
 *
 * This function waits until the data transfer complete by polling the status flag.
 * 
 * @param host The host descriptor.
 * @param timeoutInMs The timeout time in miliseconds.
 * @return kStatus_Success if no error.
 */
status_t SDHC_TransferDataPortPolling(sdhc_host_t *host, uint32_t timeoutInMs);

/*!
 * @brief Transfers data until the data transfer complete.
 *
 * This function waits until the data transfer complete by waiting event notificatiion in ISR.
 * 
 * @param host The host descriptor.
 * @param timeoutInMs The timeout time in miliseconds.
 * @return kStatus_Success if no error.
 */
status_t SDHC_TransferDataPortIRQ(sdhc_host_t *host, uint32_t timeoutInMs);

status_t SDHC_TransferAdma1Polling(sdhc_host_t *host, uint32_t timeoutInMs);

status_t SDHC_TransferAdma1IRQ(sdhc_host_t *host, uint32_t timeoutInMs);

status_t SDHC_TransferAdma2Polling(sdhc_host_t *host, uint32_t timeoutInMs);

status_t SDHC_TransferAdma2IRQ(sdhc_host_t *host, uint32_t timeoutInMs);

/*!
 * @brief Detects if the card is inserted.
 *
 * @param host The host descriptor.
 * @return kStatus_Success if no error.
 */
status_t SDHC_DetectCard(sdhc_host_t *host);

/*!
 * @brief IRQ handler for SDHC
 *
 * This function deals with IRQs on the given host controller.
 *
 * @param host the host state inforamtion
 */
void SDHC_IRQHandler(sdhc_host_t* host);


/* @} */
#if defined(__cplusplus)
}
#endif
/*! @} */

#endif /* __SDHC_H__*/

