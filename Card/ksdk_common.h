/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
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
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#if !defined(__KSDK_COMMON_H__)
#define __KSDK_COMMON_H__

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include "fsl_device_registers.h"

/*!
 * @addtogroup ksdk_common
 * @{
 */

/*******************************************************************************
 * Definitions
 ******************************************************************************/

//! @brief Construct a status code value from a group and code number.
#define MAKE_STATUS(group, code) ((((group) * 100) + (code)))

//! @brief Status group numbers.
enum _status_groups
{
    kStatusGroup_Generic = 0,           //!< Group number for generic status codes.
    kStatusGroup_LPI2C = 1,
    kStatusGroup_SDHC = 2,
    kStatusGroup_SDMMC = 3,
    kStatusGroup_SDSPI = 4,
    kStatusGroup_ApplicationRangeStart = 100,    //!< Starting number for application groups.
};

//! @brief Generic status return codes.
enum _generic_status
{
    kStatus_Success         = MAKE_STATUS(kStatusGroup_Generic, 0),
    kStatus_Fail            = MAKE_STATUS(kStatusGroup_Generic, 1),
    kStatus_InvalidArgument = MAKE_STATUS(kStatusGroup_Generic, 2),
    kStatus_Timeout         = MAKE_STATUS(kStatusGroup_Generic, 3)
};

//! @brief Type used for all status and error return values.
typedef int32_t status_t;

//! @name Min/max macros
//@{
#if !defined(MIN)
    #define MIN(a, b)   ((a) < (b) ? (a) : (b))
#endif

#if !defined(MAX)
    #define MAX(a, b)   ((a) > (b) ? (a) : (b))
#endif
//@}

//! @brief Computes the number of elements in an array.
#define ARRAY_SIZE(x)    (sizeof(x) / sizeof((x)[0]))

// #if !defined(NULL)
// #define NULL (0)
// #endif

/*******************************************************************************
 * API
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @brief
 */
static inline void EnableIRQ(IRQn_Type interrupt)
{
#if (FSL_FEATURE_SOC_INTMUX_COUNT > 0)
    if (interrupt < FSL_FEATURE_INTMUX_IRQ_START_INDEX)
#endif
    {
        NVIC_EnableIRQ(interrupt);
    }
}

/*!
 * @brief
 */
static inline void DisableIRQ(IRQn_Type interrupt)
{
#if (FSL_FEATURE_SOC_INTMUX_COUNT > 0)
    if (interrupt < FSL_FEATURE_INTMUX_IRQ_START_INDEX)
#endif
    {
        NVIC_DisableIRQ(interrupt);
    }
}

#if defined(__cplusplus)
}
#endif

/*! @} */

#endif /* __KSDK_COMMON_H__*/
/*******************************************************************************
 * EOF
 ******************************************************************************/


