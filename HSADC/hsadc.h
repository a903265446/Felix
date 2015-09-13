

#ifndef __HSADC_H__
#define __HSADC_H__

#include <stdbool.h>

#include "fsl_device_registers.h"

/******************************************************************************
 * Definitions
 *****************************************************************************/

/*!
 * @addtogroup hsadc_driver
 * @{
 */


/*! @breif Converter mask */
#define HSADC_CONVA      (1U << 0U)
#define HSADC_CONVB      (1U << 1U)
#define HSADC_CONV_BOTH  (HSADC_CONVA | HSADC_CONVB)

/*! @brief Interrupt */
#define HSADC_INT_CONVA_END_OF_SCAN      (1U << 0U)
#define HSADC_INT_CONVB_END_OF_SCAN      (1U << 1U)
#define HSADC_INT_ZERO_CROSS             (1U << 2U)
#define HSADC_INT_LOW_LIMIT              (1U << 3U)
#define HSADC_INT_HIGH_LIMIT             (1U << 4U)
#define HSADC_INT_CONVA_END_OF_CALIB     (1U << 5U)
#define HSADC_INT_CONVB_END_OF_CALIB     (1U << 6U)

#define HSADC_INT_ALL         (HSADC_CONVA_END_OF_SCAN_INT
                             | HSADC_CONVB_END_OF_SCAN_INT
                             | HSADC_ZERO_CROSS_INT
                             | HSADC_LOW_LIMIT_INT
                             | HSADC_HIGH_LIMIT_INT) 

/*! @brief Status flag */
#define HSADC_FLAG_CONVA_CONV_IN_PROGRESS        (1U << 0U)
#define HSADC_FLAG_CONVB_CONV_IN_PROGRESS        (1U << 1U)
#define HSADC_FLAG_CONVA_END_OF_SCAN             (1U << 2U)
#define HSADC_FLAG_CONVB_END_OF_SCAN             (1U << 3U)
#define HSADC_FLGA_ZERO_CROSS                    (1U << 4U)
#define HSADC_FLAG_LOW_LIMIT                     (1U << 5U)
#define HSADC_FLAG_HIGH_LIMIT                    (1U << 6U)
#define HSADC_FLAG_CONVA_END_OF_CALIB            (1U << 7U)
#define HSADC_FLAG_CONVB_END_OF_CALIB            (1U << 8U)
#define HSADC_FLAG_CONVA_DUMMY_CONV_RUNNING      (1U << 9U)
#define HSADC_FLAG_CONVB_DUMMY_CONV_RUNNING      (1U << 10U)
#define HSADC_FLAG_CONVA_CALIB_RUNNING           (1U << 11U)
#define HSADC_FLAG_CONVB_CALIB_RUNNING           (1U << 12U)
#define HSADC_FLAG_CONVA_POWERED_DOWN            (1U << 13U)
#define HSADC_FLAG_CONVB_POWERED_DOWN            (1U << 14U)

#define HSADC_FLAG_ALL           (HSADC_FLAG_CONVA_CONV_IN_PROGRESS
                                | HSADC_FLAG_CONVB_CONV_IN_PROGRESS
                                | HSADC_FLAG_CONVA_END_OF_SCAN_INT
                                | HSADC_FLAG_CONVB_END_OF_SCAN_INT
                                | HSADC_FLGA_ZERO_CROSS_INT
                                | HSADC_FLAG_LOW_LIMIT_INT
                                | HSADC_FLAG_HIGH_LIMIT_INT
                                | HSADC_FLAG_CONVA_END_OF_CALIB_INT
                                | HSADC_FLAG_CONVB_END_OF_CALIB_INT
                                | HSADC_FLAG_CONVA_DUMMY_CONV_RUNNING
                                | HSADC_FLAG_CONVB_DUMMY_CONV_RUNNING
                                | HSADC_FLAG_CONVA_CALIB_RUNNING
                                | HSADC_FLAG_CONVB_CALIB_RUNNING
                                | HSADC_FLAG_CONVA_POWERED_DOWN
                                | HSADC_FLAG_CONVB_POWERED_DOWN)



/*! @brief Converter's scan mode */
typedef enum _hsadc_scan_mode
{
    kHSADCScanModeOnceSeq           = 0U,
    kHSADCScanModeOnceParallel      = 1U,
    kHSADCScanModeLoopSeq           = 2U,
    kHSADCScanModeLoopParallel      = 3U,
    kHSADCScanModeTriggeredSeq      = 4U,
    kHSADCScanModeTriggeredParallel = 5U
} hsadc_scan_mode_t;

/*! @brief Zero crossing mode */
typedef enum _hsadc_zero_crossing_mode
{
    kHSADCZeroCrossingDisable         = 0U,
    kHSADCZeroCrossingEnableFalling   = 1U,
    kHSADCZeroCrossingEnableRising    = 2U,
    kHSADCZeroCrossingEnableBoth      = 3U
} hsadc_zero_crossing_mode_t;

/*!
 * @brief Conversion resolution
 */
typedef enum _hsadc_conv_resolution
{
    kHSADCConvResolution6Bit        = 0U,
    kHSADCConvResolution8Bit        = 1U,
    kHSADCConvResolution10Bit       = 2U,
    kHSADCConvResolution12Bit       = 3U
} hsadc_conv_resolution_t;

/*!
 * @brief DMA trigger source
 */
typedef enum _hsadc_dma_trigger_src
{
    kHSADCDmaTriggerByEndOfScanInt    = 0U,
    kHSADCDmaTriggerByResultReady     = 1U
} hsadc_dma_trigger_src_t;

/*!
 * @brief Slot flag
 */
typedef enum _hsadc_slot_flag
{
    kHSADCSlotFlagOfResultReady       = 0U,
    kHSADCSlotStatusLowLimit          = 1U,
    kHSADCSlotStatusHighLimit         = 2U,
    kHSADCSlotStatusZeroCrossing      = 3U
} hsadc_slot_status_t;

/*!
 * @brief power mode
 */
typedef enum _hsadc_idle_work_mode
{
    kHSADCIdleKeeepNormal       = 0U,
    kHSADCIdleAutoStandby       = 1U,
    kHSADCIdleAutoPowerdown     = 2U
} hsadc_idle_work_mode_t;

/*!
 * @brief HSADC user configuration
 */
typedef struct HSADCUserConfig
{
    hsadc_scan_mode_t dualConvScanMode;
    bool simultaneousEnable;
    uint32_t powerUpDelay;
    hsadc_dma_trigger_src_t dmaTriggerSrc;
    hsadc_idle_work_mode_t idleWorkMode;
} hsadc_user_config_t;

/*!
 * @brief Converter configuration
 */
typedef struct HSADCConvConfig
{
    bool dmaEnable;
    bool syncEnable;
    uint32_t clockDividor;
    uint32_t samplingTime;
    bool calibBypassEnable;
} hsadc_conv_config_t;

/*!
 * @brief Slot configuration
 */
typedef struct HSADCSlotConfig
{
    /* Conversion channel setting. */
    uint32_t chnNum;
    bool diffConvEnable;

    /* Result setting */
    hsadc_zero_crossing_mode_t zeroCrossingMode;
    uint32_t lowLimitValue;
    uint32_t highLimitValue;
    uint32_t offsetValue;
    bool slotSyncEnable;
    bool slotReadyIntEnable;
} hsadc_slot_config_t;

/*! @brief Slot mask */
#define HSADC_SLOT(slotIndex) (1 << slotIndex)

/*! @brief Calibration mode mask */
#define HSADC_CALIB_SE             (1U << 0U)
#define HSADC_CALIB_DIFF           (1U << 1U)
#define HSADC_CALIB_BOTH           (HSADC_CALIB_SE | HSADC_CALIB_DIFF)

/*! @brief Calibration value mask */
#define HSADC_CONVA_SE_CALIB_VAL_MASK  0x00FF0000U
#define HSADC_CONVA_SE_CALIB_VAL_SHIFT 16U
#define HSADC_CONVA_DIFF_CALIB_VAL_MASK  0xFF000000U
#define HSADC_CONVA_DIFF_CALIB_VAL_SHIFT 24U
#define HSADC_CONVB_SE_CALIB_VAL_MASK  0x000000FFU
#define HSADC_CONVB_SE_CALIB_VAL_SHIFT 0U
#define HSADC_CONVB_DIFF_CALIB_VAL_MASK  0x0000FF00U
#define HSADC_CONVB_DIFF_CALIB_VAL_SHIFT 8U


/*!
 * @brief Resets the module.
 *
 * @param base HSADC base address
 */
void HSADC_Reset(HSADC_Type *base);

/*!
 * @brief Initializes the module.
 *
 * @param base      HSADC base address
 * @param configPtr HSADC user configuration
 */
void HSADC_Init(HSADC_Type *base, const hsadc_user_config_t *configPtr);

/*!
 * @brief Deinitializes the module.
 *
 * @param base HSADC base address
 */
void HSADC_DeInit(HSADC_Type *base);

/*!
 * @brief Configures the converter.
 *
 * @param base      HSADC base address
 * @param convMask  Converter mask
 * @param configPtr Conveter configuration
 */
void HSADC_ConfigConv(HSADC_Type *base, uint32_t convMask, const hsadc_conv_config_t *configPtr);

/*!
 * @brief Enables or disables the sync input 
 *
 * @param base     HSADC base address
 * @param convMask Converter mask
 * @param enable   True to enable the sync input 
 */
void HSADC_SetConvSyncCmd(HSADC_Type *base, uint32_t convMask, bool enable);

/*!
 * @brief Sets the channel 6/7 mux selector for the converter.
 *
 * @param base       HSADC base address
 * @param convMask   Converter mask
 * @param muxSeletor Channel 6/7 mux selector index
 */
void HSADC_SetConvChn67Mux(HSADC_Type *base, uint32_t convMask, uint32_t muxSeletor);

/*!
 * @brief Enables or disables the conversion to stop.
 *
 * @param base     HSADC base address
 * @param convMask Converter mask
 * @param enable   True to enable the converter to enter stop mode
 */
void HSADC_SetConvStopModeCmd(HSADC_Type *base, uint32_t convMask, bool enable);

/*!
 * @brief Enables or disables the conversion to start(software trigger).
 *
 * @param base     HSADC base address
 * @param convMask Converter mask
 */
void HSADC_StartConv(HSADC_Type *base, uint32_t convMask);

/*!
 * @brief Powers down the converter 
 *
 * @param base     HSADC base address
 * @param convMask Converter mask
 * @param enable   True to power down the converter
 */
void HSADC_SetConvPowerDownCmd(HSADC_Type *base, uint32_t convMask, bool enable);

/*!
 * @brief Enables or disables the converter interrupt 
 *
 * @param base    HSADC base address
 * @param intMask Interrupt mask
 * @param enable  True to enable the interrupt
 */
void HSADC_SetConvIntCmd(HSADC_Type *base, uint32_t intMask, bool enable);

/*!
 * @brief Gets the converter flag status
 *
 * @param  base HSADC base address
 * @param  flag Converter flag
 *
 * @return      The converter flag status
 */
uint32_t HSADC_GetConvFlag(HSADC_Type *base);

/*!
 * @brief Clears the converter flag
 *
 * @param base HSADC base address
 * @param flag Converter falg
 */
void HSADC_ClearConvFlag(HSADC_Type *base, uint32_t flagMask);

/*!
 * @brief Disables all slos
 *
 * @param base HSADC base address
 */
void HSADC_ClearSeq(HSADC_Type *base);

/*!
 * @brief Configures the slot
 *
 * @param base      HSADC base address
 * @param slotIndex Slot index
 * @param configPtr 
 */
void HSADC_ConfigSeqSlot(HSADC_Type *base, uint32_t slotIndex, const hsadc_slot_config_t *configPtr);

/*!
 * @brief Gets the slot flag status
 *
 * @param  base     HSADC base address
 * @param  slotMask Slot mask
 * @param  flagType The flag type
 *
 * @return          Slot flag status
 */
uint32_t HSADC_GetSeqFlag(HSADC_Type *base, uint32_t slotMask, hsadc_slot_status_t flagType);

/*!
 * @brief Clears the slot flag.
 *
 * @param base     HSADC base address
 * @param slotMask Slot mask
 * @param flagType Slot flag type
 */
void HSADC_ClearSeqFlag(HSADC_Type *base, uint32_t slotMask, hsadc_slot_status_t flagType);

/*!
 * @brief
 *
 * @param  base  [description]
 * @param  index [description]
 *
 * @return       [description]
 */
uint32_t HSADC_GetSeqSlotConvResult(HSADC_Type *base, uint32_t slotIndex);


/*!
 * @brief Enables or disables the converter calibration
 *
 * This function sets the converter to start single-ended calibration or differential calibration 
 * as the mask as the definition from HSADC_SE_CALIB_MASK to 
 * HSADC_CONVB_DIFF_CALLIBRATION_VAL_SHIFT
 *
 * @param base                HSADC base address
 * @param convMask            Converter mask
 * @param calibrationModeMask Calibration mode mask
 * @param enable              True to enable the converter calibration
 */
void HSADC_SetConvCalibCmd(HSADC_Type *base, uint32_t convMask, uint32_t calibModeMask, bool enable);

/*!
 * @brief Gets the converter calibration value
 *
 * This function gets the converter single-ended calibration value or differential calibration value
 * as the mask as the definition from HSADC_SE_CALIB_MASK to 
 * HSADC_CONVB_DIFF_CALLIBRATION_VAL_SHIFT
 *
 * @param  base                HSADC base address
 * @param  convMask            Converter mask
 * @param  calibrationModeMask Calibration mode mask
 *
 * @return                     The calibration value
 */
uint32_t HSADC_GetConvCalibValue(HSADC_Type *base, uint32_t convMask, uint32_t calibModeMask);

uint32_t HSADC_CalibAtPowerUp(HSADC_Type *base, uint32_t convMask, uint32_t calibModeMask);
uint32_t HSADC_CalibAfterPowerup(HSADC_Type *base, uint32_t convMask, uint32_t calibModeMask);

#endif /* __HSADC_H__ */


