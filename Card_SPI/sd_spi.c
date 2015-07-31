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

/******************************************************************************
 * Enumerations.
 *****************************************************************************/

 /*! @addtogroup sdhc_carddrv_data_types */
/*! @{ */

/*! 
 * @brief The command response enumeration type. 
 *
 * Defines the command response enumeration type from card to host controller.
 */
typedef enum _sd_card_resp_type {
    kSdhcHostRespTypeNone = 0,          /*!< Response type: none */
    kSdhcHostRespTypeR1,                /*!< Response type: R1 */
    kSdhcHostRespTypeR1b,               /*!< Response type: R1b */
    kSdhcHostRespTypeR2,                /*!< Response type: R2 */
    kSdhcHostRespTypeR3,                /*!< Response type: R3 */
    kSdhcHostRespTypeR4,                /*!< Response type: R4 */
    kSdhcHostRespTypeR5,                /*!< Response type: R5 */
    kSdhcHostRespTypeR5b,               /*!< Response type: R5b */
    kSdhcHostRespTypeR6,                /*!< Response type: R6 */
    kSdhcHostRespTypeR7,                /*!< Response type: R7 */
} sdhc_host_resp_type_t;

/*! @brief Defines the sdhc card API's running status. */
typedef enum _sdcard_status
{    
    kStatus_SDCARD_NoError         = 0U, /*!< Success */
    kStatus_SDCARD_InvalidArgument = 1U, /*!< Invalid argument existed. */
    kStatus_SDCARD_Failed          = 2U  /*!< Failed. */      
} sdcard_status_t;

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*!
 * @brief SDHC request data structure
 *
 * Defines the request data structure from sdhc to card which includes the block 
 * size/count and flags.
 */
typedef struct SdhcHostReqData
{
    uint32_t blockSize;                             /*!< Block size */
    uint32_t blockCount;                            /*!< Block count */
    uint32_t bytesTransferred;                      /*!< Transferred buffer */
    uint32_t *buffer;                               /*!< Data buffer */
} sdhc_host_req_data_t;

/*!
 * @brief SDHC request structure
 *
 * Defines the request from sdhc to card which includes the command index, 
 * argument, flags, response, and data.
 */
typedef struct SdhcHostRequest
{
    uint32_t cmdIndex;                              /*!< Command index */
    uint32_t argument;                              /*!< Command argument */
    uint32_t flags;                                 /*!< Flags */
#define SDHC_HOST_REQ_FLAGS_DATA_READ      (1 << 0)  /*!< Request will read data */
#define SDHC_HOST_REQ_FLAGS_USE_DMA        (1 << 1)  /*!< Request will use DMA for data transferring */
#define SDHC_HOST_REQ_FLAGS_STOP_TRANS     (1 << 2)  /*!< Request to stop transmition */ 
    sdhc_host_resp_type_t respType;                  /*!< Response type */
    volatile uint32_t error;                        /*!< Command error code */
#define SDHC_HOST_REQ_ERR_HOST_BUSY        (1 << 0)  /*!< Host is busy */
#define SDHC_HOST_REQ_ERR_SEND_CMD         (1 << 1)  /*!< Send command error */
#define SDHC_HOST_REQ_ERR_CMD_CRC          (1 << 2)  /*!< Command CRC error */
#define SDHC_HOST_REQ_ERR_CMD_INDEX        (1 << 3)  /*!< Command index error */
#define SDHC_HOST_REQ_ERR_CMD_END_BIT      (1 << 4)  /*!< Command end bit error */
#define SDHC_HOST_REQ_ERR_CMD_TIMEOUT      (1 << 5)  /*!< Command timeout error */
#define SDHC_HOST_REQ_ERR_CARD_REMOVED     (1 << 6)  /*!< Card removed */
#define SDHC_HOST_REQ_ERR_RSPBUSY_TIMEOUT  (1 << 7)  /*!< Response busy timeout error */
#define SDHC_HOST_REQ_ERR_DAT_TIMEOUT      (1 << 8)  /*!< Data timeout error */
#define SDHC_HOST_REQ_ERR_DATA_CRC         (1 << 9)  /*!< Data CRC error */
#define SDHC_HOST_REQ_ERR_DATA_END_BIT     (1 << 10) /*!< Data end bit error */
#define SDHC_HOST_REQ_ERR_AUTO_CMD12       (1 << 11) /*!< Auto cmd12 error */
#define SDHC_HOST_REQ_ERR_DMA              (1 << 12) /*!< DMA error */
#define SDHC_HOST_REQ_ERR_TIMEOUT          (1 << 13) /*!< Request timeout error */
#define SDHC_HOST_REQ_ERR_DATA_PREPARE     (1 << 14) /*!< Data preparation error */
    uint32_t response[4];                           /*!< Response for this command */
    bool completeFlag;                              /*!< Request completion flag set in ISR */
    struct sdhc_host_req_data_t *data;              /*!< Data associated with request */
} sdhc_host_request_t;

/*! @brief state structure to save sdhc host information and callback */
typedef struct SdhcHost
{
    /* Data */
    uint32_t hostInstance;                          /*!< Host instance id */
    sdhc_capability_t* capability;                  /*!< Capability information */
    sdhc_host_request_t* currentReq;                /*!< Associated request */

    /* Callback */
    void (*cardIntCallback)(void);                  /*!< Callback function for card interrupt occurs */
    void (*cardDetectCallback)(bool inserted);      /*!< Callback function for card detect occurs */
    void (*blockGapCallback)(void);                 /*!< Callback function for block gap occurs */
} sdhc_host_t;

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