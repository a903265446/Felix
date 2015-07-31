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

#ifndef __SD_SPI_H__
#define __SD_SPI_H__

/******************************************************************************
 * Enumerations.
 *****************************************************************************/

 /*! @addtogroup sdspi_carddrv_data_types */
/*! @{ */

/* SPI mode related */
#define SD_SPI_R1_IN_IDLE_STATE   (1 << 0)
#define SD_SPI_R1_ERASE_RESET     (1 << 1)
#define SD_SPI_R1_ILLEGAL_CMD     (1 << 2)
#define SD_SPI_R1_COM_CRC_ERR     (1 << 3)
#define SD_SPI_R1_ERASE_SEQ_ERR   (1 << 4)
#define SD_SPI_R1_ADDRESS_ERR     (1 << 5)
#define SD_SPI_R1_PARAMETER_ERR   (1 << 6)

#define SD_SPI_R2_CARD_LOCKED     (1 << 0)
#define SD_SPI_R2_WP_LOCK_FAILED  (1 << 1)
#define SD_SPI_R2_ERR             (1 << 2)
#define SD_SPI_R2_CC_ERR          (1 << 3)
#define SD_SPI_R2_CARD_ECC_FAILED (1 << 4)
#define SD_SPI_R2_WP_VIOLATION    (1 << 5)
#define SD_SPI_R2_ERASE_PARAM     (1 << 6)
#define SD_SPI_R2_OUT_OF_RANGE    (1 << 7)
#define SD_SPI_R2_CSD_OVERWRITE   (1 << 7)

#define SD_SPI_R7_VERSION_SHIFT   (28)
#define SD_SPI_R7_VERSION_MASK    (0xF)
#define SD_SPI_R7_VOLTAGE_SHIFT   (8)
#define SD_SPI_R7_VOLTAGE_MASK    (0xF)
#define SD_SPI_R7_VOLTAGE_27_36   ((uint32_t) 0x1 << SD_SPI_R7_VOLTAGE_SHIFT)
#define SD_SPI_R7_ECHO_SHIFT      (0)
#define SD_SPI_R7_ECHO_MASK       ((uint32_t) 0xFF)

/* Data Error Token */
#define SD_SPI_DET_MASK           (0xF)
#define SD_SPI_DET_ERROR          (1 << 0)       /*!< Data error */
#define SD_SPI_DET_CC_ERROR       (1 << 1)       /*!< CC error */
#define SD_SPI_DET_ECC_FAILED     (1 << 2)       /*!< Card ecc error */
#define SD_SPI_DET_OUT_OF_RANGE   (1 << 3)       /*!< Out of range */

/* Data Token */
#define SD_SPI_DT_START_SINGLE_BLK (0xFEU)       /*!< First byte of block, single block */
#define SD_SPI_DT_START_MULTI_BLK  (0xFCU)       /*!< First byte of block, multi-block */
#define SD_SPI_DT_STOP_TRANSFER    (0xFDU)       /*!< Stop transmission */

/* Data Response */
#define SD_SPI_DR_MASK             (0x1F)        /*!< Mask for data response bits */
#define SD_SPI_DR_ACCEPTED         (0x05)        /*!< Data accepted */
#define SD_SPI_DR_CRC_ERROR        (0x0B)        /*!< Data rejected due to CRC error */
#define SD_SPI_DR_WRITE_ERROR      (0x0D)        /*!< Data rejected due to write error */



/*************************************************************************************************
 * API
 ************************************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif

/*! @name SDSPI CARD DRIVER FUNCTION */
/*@{ */


/*@} */
#if defined(__cplusplus)
}
#endif
/*! @} */
#endif  /* __SD_SPI_H__*/

/*************************************************************************************************
 * EOF
 ************************************************************************************************/