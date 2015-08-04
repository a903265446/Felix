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

#include "spec.h"
#include "sdhc.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*!
 * @addtogroup CARD TYPE
 * @{
 */

/*! 
 * @brief The command response type. 
 *
 * Defines the command response type from card to host controller.
 */
typedef enum _sdmmc_resp_type 
{
    kSdmmcRespTypeNone = 0U,         /*!< Response type: none */
    kSdmmcRespTypeR1   = 1U,         /*!< Response type: R1 */
    kSdmmcRespTypeR1b  = 2U,         /*!< Response type: R1b */
    kSdmmcRespTypeR2   = 3U,         /*!< Response type: R2 */
    kSdmmcRespTypeR3   = 4U,         /*!< Response type: R3 */
    kSdmmcRespTypeR4   = 5U,         /*!< Response type: R4 */
    kSdmmcRespTypeR5   = 6U,         /*!< Response type: R5 */
    kSdmmcRespTypeR5b  = 7U,         /*!< Response type: R5b */
    kSdmmcRespTypeR6   = 8U,         /*!< Response type: R6 */
    kSdmmcRespTypeR7   = 9U,         /*!< Response type: R7 */
} sdmmc_resp_type_t;

/*! @brief Defines the SD/MMC card API's running status. */
typedef enum _sdmmc_status
{   
    kStatus_SDMMC_NoError                   = 0U,/*!< Success */
    kStatus_SDMMC_Failed                    = 1U,/*!< Failed */  
    kStatus_SDMMC_TimeoutError              = 2U,/*!< Timeout */  
    kStatus_SDMMC_NotSupportYet             = 3U,/*!< Havn't supported */
    kStatus_SDMMC_OutOfMemory               = 4U,/*!< No enough memory */
    kStatus_SDMMC_BlockSizeHostNotSupport   = 5U,/*!< Block size isn't supported by host */
    kStatus_SDMMC_HostIsBusyError           = 6U,/*!< Host is busy */ 
    kStatus_SDMMC_PrepareHostDmaDataError   = 7U,/*!< DMA address error */
    kStatus_SDMMC_CardDetectNotSupportYet   = 8U,/*!< Card detection type isn't supported by host */
    kStatus_SDMMC_NoCardInsertedError       = 9U,/*!< No inserted card */
    kStatus_SDMMC_AllocHostAdmaTableFailed  = 10U,/*!< Allocate host ADMA table memory failed */
    kStatus_SDMMC_SendCardCmdFailed         = 11U,/*!< Send card command failed */
    kStatus_SDMMC_TransferCardDataFailed    = 12U,/*!< Transfer card data failed */
    kStatus_SDMMC_CardStatusError           = 13U,/*!< Card status is error */
    kStatus_SDMMC_SetCardBlockSizeFailed    = 14U,/* Set card block size failed */
    kStatus_SDMMC_SetCardBusClockFailed     = 15U,/*!< Set card bus clock failed */
    kStatus_SDMMC_CardNotSupport            = 16U,/*!< Card doesn't support */ 
    kStatus_SDMMC_InvalidIORange            = 17U,/*!< Invalid read/write address range */ 
    kStatus_SDMMC_AllSendCidCmdFailed       = 18U,/*!< AllSendCid command failed */
    kStatus_SDMMC_SendRcaCmdFailed          = 19U,/*!< SendRca command failed */
    kStatus_SDMMC_SendCsdCmdFailed          = 20U,/*!< SendCsd command failed */
    kStatus_SDMMC_SelectCardCmdFailed       = 21U,/*!< SelectCard command failed */
    kStatus_SDMMC_SendScrCmdFailed          = 22U,/*!< SendScr command failed */ 
    kStatus_SDMMC_SetBusWidthCmdFailed      = 23U,/*!< SetBusWidth command failed */
    kStatus_SDMMC_GoIdleCmdFailed           = 24U,/*!< GoIdle command failed */
    kStatus_SDMMC_SendOpCondCmdFailed       = 25U,/*!< SendOpCond command failed */
    kStatus_SDMMC_EraseCmdFailed            = 26U,/*!< Erase command failed */
    kStatus_SDMMC_SwitchFunctionCmdFailed   = 27U,/*!< SwitchFunction command failed */ 
    kStatus_SDMMC_StopTransmissionCmdFailed = 28U,/*!< StopTransmission command failed */
    kStatus_SDMMC_SendStatusCmdFailed       = 29U,/*!< SendStatus command failed */
    kStatus_SDMMC_CreateEventFailed         = 30U,/*!< Create event failed */
    kStatus_SDMMC_WaitEventFailed           = 31U,/*!< Wait event failed */
    kStatus_SDMMC_NotifyEventFailed         = 32U,/*!< Notify event failed */
    kStatus_SDMMC_GetCurrentTimeFaild       = 33U /*!< Get current time failed */
} sdmmc_status_t;

/*! @brief Defines host's transfer mode */
typedef enum _sdmmc_host_transfer_mode 
{
    kSdmmcHostTransModePio   = 1U,   /*!< Polling Data Port */
    kSdmmcHostTransModeSdma  = 2U,   /*!< Simple DMA */
    kSdmmcHostTransModeAdma1 = 3U,   /*!< ADMA1 */
    kSdmmcHostTransModeAdma2 = 4U,   /*!< ADMA2 */
} sdmmc_host_transfer_mode_t;

/*! @brief Defines host's card detection way */
typedef enum _sdmmc_host_card_detect 
{
    kSdmmcHostCardDetectGpio     = 1U,/*!< Use GPIO for card detection. */
    kSdmmcHostCardDetectDat3     = 2U,/*!< Use DAT3 for card detection. */
    kSdmmcHostCardDetectCdPin    = 3U,/*!< Use dedicate CD pin for card detection */
    kSdmmcHostCardDetectPollDat3 = 4U,/*!< Poll DAT3 for card detection. */
    kSdmmcHostCardDetectPollCd   = 5U,/*!< Poll dedicate CD pin for card detection. */
} sdmmc_host_card_detect_t;

/*!
 * @brief Card data structure
 *
 * Defines this structure to contain data related attribute, flag and error status.
 */
typedef struct CardData
{
    uint32_t blockSize;             /*!< Block size */
    uint32_t blockCount;            /*!< Block count */
    uint32_t bytesTransferred;      /*!< Transferred byte count */
    uint32_t *buffer;               /*!< Data buffer */
    uint32_t flags;                 /*!< Data flags */
#define CARD_DATA_FLAGS_DATA_READ      (1U << 0U)/*!< Data direction */
#define CARD_DATA_FLAGS_USE_DMA        (1U << 1U)/*!< DMA enable status */    
    uint32_t errors;                /*!< Error status */
#define CARD_DATA_ERR_DATA_TIMEOUT     (1U << 0U)/*!< Data timeout error */ 
#define CARD_DATA_ERR_DATA_CRC         (1U << 1U)/*!< Data CRC error */
#define CARD_DATA_ERR_DATA_END_BIT     (1U << 2U)/*!< Data end bit error */
#define CARD_DATA_ERR_DMA              (1U << 3U)/*!< DMA error */
#define CARD_DATA_ERR_AUTO_CMD12       (1U << 4U)/*!< Auto CMD12 error */
} card_data_t;

/*!
 * @brief Card command structure
 *
 * Defines card command related attribute, flags, error status.
 */
typedef struct CardCmd
{
    uint32_t cmdIndex;                /*!< Command index */
    uint32_t argument;                /*!< Command argument */
    uint32_t flags;                   /*!< Flags */
#define CARD_CMD_FLAGS_STOP_TRANS     (1U << 0U) /*!< Request to stop transmition */ 
    sdmmc_resp_type_t respType;       /*!< Command response type */
    uint32_t errors;                  /*!< Command error code */
#define CARD_CMD_ERR_HOST_BUSY        (1U << 0U) /*!< Host is busy */
#define CARD_CMD_ERR_SEND_CMD         (1U << 1U) /*!< Send command error */
#define CARD_CMD_ERR_CMD_CRC          (1U << 2U) /*!< Command CRC error */
#define CARD_CMD_ERR_CMD_INDEX        (1U << 3U) /*!< Command index error */
#define CARD_CMD_ERR_CMD_END_BIT      (1U << 4U) /*!< Command end bit error */
#define CARD_CMD_ERR_CMD_TIMEOUT      (1U << 5U) /*!< Command timeout error */
#define CARD_CMD_ERR_CARD_REMOVED     (1U << 6U) /*!< Card removed */
#define CARD_CMD_ERR_RSPBUSY_TIMEOUT  (1U << 7U) /*!< Response busy timeout error */
    uint32_t response[4];             /*!< Response for this command */
} card_cmd_t;

/* Host capability */
#define host_capability_t sdhc_capability_t

/*! @brief Host information and callback */
typedef struct Host
{
    /* Data */
    uint32_t instance;                       /*!< Host instance id */
    sdmmc_host_card_detect_t cardDetectMode; /*!< Card detection mode */
    sdmmc_host_transfer_mode_t transferMode; /*!< Data transfer mode */
    host_capability_t * capability;          /*!< Capability information */
    uint32_t flags;                          /*!< Flags */
#define HOST_FLAGS_CARD_PRESENTED    (1U << 0U) /* Detected card is inserted */
    card_cmd_t *currentCmd;                  /*!< Command is sending */
    card_data_t *currentData;                /*!< Data is transferring */
    uint32_t *admaTableAddress;              /*!< ADMA table address */
    uint32_t admaTableMaxEntries;            /*!< Items count in ADMA table */

    /* Callback */
    bool (*createCmdEvent)();                /*!< Create command event */
    bool (*waitCmdEvent)(uint32_t timeout);  /*!< Wait command event */
    bool (*notifyCmdEvent)();                /*!< Notify command event */
    bool (*deleteCmdEvent)();                /*!< Delete command event */
    bool (*createDataEvent)();               /*!< Create data event */
    bool (*waitDataEvent)(uint32_t timeout); /*!< Wait data event */
    bool (*notifyDataEvent)();               /*!< Notify data event */ 
    bool (*deleteDataEvent)();               /*!< Delete data event */
    void (*markStartTimeMsec)();             /*!< Mark start to count milisecond */
    uint32_t (*getElapsedTimeMsec)();        /*!< Get miliseconds elapsed */
    void (*delayTimeMsec)();                 /*!< Delay milisecond */
    void (*cardIntCallback)(uint32_t hostInstance); /*!< Card interrupt occurs */
    void (*cardDetectCallback)(uint32_t hostInstance, bool inserted);/*!< Card detect occurs */
    void (*blockGapCallback)(uint32_t hostInstance);/*!< Block gap occurs */
} host_t;

/* Defines host endian mode */
//#define FSL_HOST_USING_BIG_ENDIAN 
/* Defines if card driver support ADMA1 related operation. */
//#define FSL_CARD_DRIVER_ENABLE_ADMA1

/* Check if host support switching the card to high speed mode */
#define DOES_HOST_SUPPORT_HIGHSPEED(x)      (x->capability->supportMask & SDHC_SUPPORT_HIGHSPEED)
/* eSDHC on all kinetis boards will support 4 bit data bus. */
#define DOES_HOST_SUPPORT_4BITS(x)          true
/* Defines if use IRQ mode to send command and transfer data */
#define FSL_CARD_DRIVER_USING_IRQ
/* Enable auto CMD12 in host */
//#define FSL_CARD_DRIVER_ENABLE_HOST_AUTOCMD12

/* Continuously wait till the event be nodified */
#define FSL_HOST_WAIT_FOREVER               (1U << 32U - 1U)

/*!
 * @brief SD Card Structure
 *
 * Defines the card structure including the necessary fields to identify and
 * describe the card.
 */
typedef struct Sd 
{
    host_t * host;               /*!< Host state information */
    uint32_t rca;                /*!< Relative address of the card */
    uint32_t version;            /*!< Card version */
    uint32_t caps;               /*!< Capability */
#define SD_CARD_CAPS_HIGHCAPACITY    (1U << 1U)/*!< Card is high capacity */
#define SD_CARD_CAPS_BUSWIDTH_4BITS  (1U << 2U)/*!< Support 4-bit data width */
#define SD_CARD_CAPS_SDHC            (1U << 3U)/*!< Card is SDHC */
#define SD_CARD_CAPS_SDXC            (1U << 4U)/*!< Card is SDXC */
    uint32_t rawCid[4];          /*!< Raw CID content */
    uint32_t rawCsd[4];          /*!< Raw CSD content */
    uint32_t rawScr[2];          /*!< Raw CSD content */
    uint32_t ocr;                /*!< Raw OCR content */
    sd_cid_t cid;                /*!< CID */
    sd_csd_t csd;                /*!< CSD */
    sd_scr_t scr;                /*!< SCR */
    uint32_t blockCount;         /*!< Card total block number */
    uint32_t blockSize;          /*!< Card block size */
} sd_t;

/* Checks if card support high speed mode. */
#define IS_HIGHCAPACITY_CARD(x)             ((x)->caps & SD_CARD_CAPS_HIGHCAPACITY)
/* Checks if card support 4 bit width */
#define DOES_CARD_SUPPORT_4BITS(x)          ((x)->caps & SD_CARD_CAPS_BUSWIDTH_4BITS)
/* Maximum loop count to check the card operation voltage range */
#define FSL_CARD_MAX_VOLT_RETRIES           (1000U)
/* Default block size */
#define FSL_CARD_DEFAULT_BLOCK_SIZE         (512U) 
/* Card command maximum timeout value */     
#define FSL_CARD_COMMAND_TIMEOUT            (1000U)

/*! @name CARD  FUNCTION */
/*@{ */

/*************************************************************************************************
 * API
 ************************************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif

/*! @name CARD DRIVER FUNCTION */
/*@{ */

/*!
 * @brief Initializes the card on a specific host controller.
 *
 * This function initializes the card on a specific SDHC.
 *
 * @param host The pointer to store the host inforamtion.
 * @param card The pointer to store card related information.
 * @return kStatus_SD_NoError on success
 */
sdmmc_status_t SD_IndentifyCard(host_t *host, sd_t *card);

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
 * @return kStatus_SD_NoError on success
 */
sdmmc_status_t SD_ReadBlocks(sd_t *card, uint8_t *buffer, uint32_t startBlock, 
                    uint32_t blockCount);

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
 * @return kStatus_SD_NoError on success
 */
sdmmc_status_t SD_WriteBlocks(sd_t *card, uint8_t *buffer, uint32_t startBlock, 
                      uint32_t blockCount);

/*!
 * @brief Erases blocks of the specific card.
 *
 * This function erases blocks of a specific card, with default block size 
 * defined by the SDHC_CARD_DEFAULT_BLOCK_SIZE.
 *
 * @param card The handle of the card
 * @param startBlock The start block index
 * @param blockCount The number of blocks to erase
 * @return kStatus_SD_NoError on success
 */
sdmmc_status_t SD_EraseBlocks(sd_t *card, uint32_t startBlock, uint32_t blockCount);

/*!
 * @brief Checks whether the card is write-protected.
 *
 * This function checks if the card is write-protected via CSD register.
 *
 * @param card The specific card
 * @return kStatus_SD_NoError on success
 */
bool SD_CheckReadOnly(sd_t *card);

/*!
 * @brief Deinitializes the card.
 *
 * This function deinitializes the specific card.
 *
 * @param card The specific card
 */
void SD_Shutdown(sd_t *card);

/*!
 * @brief Checks whether the card is present on a specified host controller.
 *
 * This function checks if there's a card inserted in the SDHC. It is mainly 
 * used in the polling detection pin mode when DAT3 or dedicate CD pin is selected
 * as card detection pin.
 *
 * @param card The specific card
 * @return kStatus_SD_NoError on success
 */
sdmmc_status_t SD_DetectCard(sd_t *card);


/*!
 * @brief IRQ handler for SDHC
 *
 * This function deals with IRQs on the given host controller.
 *
 * @param host the host state inforamtion
 */
void SDMMC_IrqHandler(host_t* host);

/*!
 * @brief Sends a command using block way.
 *
 * This function will wait until the command response is got.
 *
 * @param host The host information.
 * @param timeoutInMs The timeout time in miliseconds.
 * @return kStatus_SDMMC_NoError if no error.
 */
sdmmc_status_t SDMMC_SendCmdBlocking(host_t *host, uint32_t timeoutInMs);

/*!
 * @brief Checks the card status in the R1 reponse.
 *
 * This function checks the card status to estimate if sending command success.
 *
 * @param cardCmd The card command inforamtion.
 * @return kStatus_SDMMC_NoError if no error.
 */
sdmmc_status_t SDMMC_CheckR1Response(card_cmd_t *cardCmd);

/*!
 * @brief Waits until the data transfer complete.
 *
 * @param host The host information.
 * @param timeoutInMs The timeout time in miliseconds.
 * @return kStatus_SDMMC_NoError if no error.
 */
sdmmc_status_t SDMMC_WaitDataTransferComplete(host_t *host, uint32_t timeoutInMs);

/*!
 * @brief Detects if the card is inserted.
 *
 * @param host The host information.
 * @return kStatus_SDMMC_NoError if no error.
 */
sdmmc_status_t SDMMC_DetectCard(host_t *host);

/*!
 * @brief Configures the SD bus clock frequence.
 *
 * @param host The host information.
 * @param Clock The target SD bus clock frequence. 
 * @return kStatus_SDMMC_NoError if no error.
 */
sdmmc_status_t SDMMC_ConfigClock(host_t *host, uint32_t targetClock);

/*!
 * @brief Initializes the host controller.
 *
 * @param host The host information.
 * @return kStatus_SDMMC_NoError if no error.
 */
sdmmc_status_t SDMMC_InitHost(host_t *host);

/*!
 * @brief Deinitializes the host controller.
 * 
 * @param host The host information.
 * @return kStatus_SDMMC_NoError if no error.
 */
sdmmc_status_t SDMMC_DeInitHost(host_t *host);

/*!
 * @brief Sets the bus width of the host controller.
 *
 * @param host The host information.
 * @param busWidth The data bus width.
 */
sdmmc_status_t SDMMC_SetHostBusWidth(host_t *host, sdhc_dtw_t busWidth);
/*@} */
#if defined(__cplusplus)
}
#endif
/*! @} */
#endif  /* __CARD_H__*/

/*************************************************************************************************
 * EOF
 ************************************************************************************************/
 