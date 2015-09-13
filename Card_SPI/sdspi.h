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

#ifndef __SDSPI_H__
#define __SDSPI_H__

#include <stdbool.h>

#include "fsl_device_registers.h"
#include "spec.h" 
#include "ksdk_common.h"
#include "fsl_dspi_master_driver.h"


/******************************************************************************
 * Definitions
 *****************************************************************************/

/*!
 * @addtogroup sdspi_driver
 * @{
 */

/*! @brief SDSPICard API status */
typedef enum _sdspi_status
{
    kStatus_SDSPI_TransferFailed  = MAKE_STATUS(kStatusGroup_SDSPI, 0),/*!< Transfer failed */
    kStatus_SDSPI_CardIsBusyError = MAKE_STATUS(kStatusGroup_SDSPI, 0),/*!< Card busy */
    kStatus_SDSPI_OutOfMemory     = MAKE_STATUS(kStatusGroup_SDSPI, 0),/*!< Out of memory */
    kStatus_SDSPI_TimeoutError    = MAKE_STATUS(kStatusGroup_SDSPI, 0),/*!< Time out */
    kStatus_SDSPI_WriteProtected  = MAKE_STATUS(kStatusGroup_SDSPI, 0),/*!< Write protected */
    kStatus_SDSPI_NotSupportYet   = MAKE_STATUS(kStatusGroup_SDSPI, 0),/*!< Not support */
} sdspi_status_t;

/*! @brief SDSPICard response type */
typedef enum _sdspi_response_type 
{
    kSDSPIRespTypeR1  = 0U,                 /*!< Response 1 */
    kSDSPIRespTypeR1b = 1U,                 /*!< Response 1 with busy */
    kSDSPIRespTypeR2  = 2U,                 /*!< Response 2 */
    kSDSPIRespTypeR3  = 3U,                 /*!< Response 3 */
    kSDSPIRespTypeR7  = 4U,                 /*!< Response 7 */
} sdspi_resp_type_t;

/*!@brief SDSPI command object */
typedef struct SDSPICmd 
{
    uint8_t cmdIndex;                                   /*!< Command index */
    uint32_t argument;                                  /*!< Command argument */
    uint8_t respType;                                   /*!< Response type */
    uint8_t response[5];                                /*!< Response */
} sdspi_cmd_t;

typedef struct SDSPIHost 
{
    SPI_Type *base;                   /*!< SPI master base address */
    uint32_t busBaudRate;             /*!< Bus baud rate */
    dspi_master_state_t* spiState;
    dspi_device_t* spiDevice;

    status_t (*setFrequency)(SPI_Type *base, uint32_t frequency); /*!< Set frequency of SPI */
    //sdspi_status_t (*sendByte)(SPI_Type *base, uint8_t byte);/*!< Send one byte and fetch return */     
    status_t (*exchange)(SPI_Type *base, const uint8_t *in, uint8_t *out, uint32_t size); /*!< Exchange data over SPI */
    uint32_t (*getCurrentTimeMsec)(void);/*!< Get current time in milliseconds */
    uint32_t timeRangeMsec;           /*!< Time range in miliseconds */
} sdspi_host_t;


/*!
 * @brief SD Card Structure
 *
 * Defines the card structure including the necessary fields to identify and
 * describe the card.
 */
typedef struct SDSPICard 
{
    sdspi_host_t * host;         /*!< Host state information */
    uint32_t rca;                /*!< Relative address of the card */
    uint32_t version;            /*!< Card version */
    uint32_t caps;               /*!< Capability */
#define SDSPI_CAPS_HIGH_CAPACITY    (1U << 1U)/*!< Card is high capacity */
#define SDSPI_CAPS_SDHC             (1U << 3U)/*!< Card is SDHC */
#define SDSPI_CAPS_SDXC             (1U << 4U)/*!< Card is SDXC */
#define SDSPI_CAPS_SDSC             (1U << 5U)/*!< Card is SDSC */
    uint32_t state;              /*!< SPI state */
#define SDSPI_STATE_WRITE_PROTECTED (1U << 0U)/*!< Card is write protected */
    uint8_t rawCid[16];          /*!< Raw CID content */
    uint8_t rawCsd[16];          /*!< Raw CSD content */
    uint8_t rawScr[8];          /*!< Raw CSD content */
    uint32_t ocr;                /*!< Raw OCR content */
    sd_cid_t cid;                /*!< CID */
    sd_csd_t csd;                /*!< CSD */
    sd_scr_t scr;                /*!< SCR */
    uint32_t blockCount;         /*!< Card total block number */
    uint32_t blockSize;          /*!< Card block size */
} sdspi_card_t;

#define IS_BLOCK_ACCESS(x)  ((x)->caps & SDSPI_CAPS_HIGH_CAPACITY)

/* common default attribute for all kinds of card. */

/* Default block size */
#define FSL_CARD_DEFAULT_BLOCK_SIZE         (512U) 
/* Card command maximum timeout value */     
#define FSL_SDSPI_TIMEOUT            (1000U)

/*************************************************************************************************
 * API
 ************************************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @name SDSPI CARD DRIVER FUNCTION 
 * @{ 
 */

/*!
 * @brief Initializes the card on a specific SPI instance.
 *
 * This function initializes the card on a specific SPI instance.
 *
 * @param card the place to store card related information
 * @return kStatus_SDSPI_NoError on success
 */
status_t SDSPI_Init(sdspi_card_t *card);

/*!
 * @brief Reads blocks from the specific card.
 *
 * This function reads blocks from specific card.
 *
 * @param card the handle of the card
 * @param buffer the buffer to hold the data read from card
 * @param startBlock the start block index
 * @param blockCount the number of blocks to read
 * @return kStatus_SDSPI_NoError on success
 */
status_t SDSPI_ReadBlocks(sdspi_card_t *card, uint8_t *buffer, uint32_t startBlock, uint32_t blockCount);


/*!
 * @brief Writes blocks of data to the specific card.
 *
 * This function writes blocks to specific card
 *
 * @param card the handle of the card
 * @param buffer the buffer holding the data to be written to the card
 * @param startBlock the start block index
 * @param blockCount the number of blocks to write
 * @return kStatus_SDSPI_NoError on success
 */
status_t SDSPI_WriteBlocks(sdspi_card_t *card, uint8_t *buffer, uint32_t startBlock, uint32_t blockCount);

/*!
 * @brief Checks whether the card is write-protected.
 *
 * This function checks if the card is write-protected via CSD register.
 *
 * @param card the specific card
 * @return kStatus_SDSPI_NoError on success
 */
bool SDSPI_CheckReadOnly(sdspi_card_t *card);

/*!
 * @brief Deinitializes the card.
 *
 * This function deinitializes the specific card.
 *
 * @param card the specific card
 */
void SDSPI_DeInit(sdspi_card_t *card);

/* @} */
#if defined(__cplusplus)
}
#endif
/*! @} */
#endif  /* __SD_SPI_H__*/

/*************************************************************************************************
 * EOF
 ************************************************************************************************/