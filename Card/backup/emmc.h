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

 #include "sdhc.h"
 #include <assert.h>

/*******************************************************************************
 * Definitions
 ******************************************************************************/
 
#define EMMC_CLK_26MHZ           (26000000U)
#define EMMC_CLK_52MHZ           (52000000U)

typedef enum _emmc_cmd_t {           /*   type     argument        response */
    kEmmcSetRelativeAddr = 3,        /*!< ac       [31:16] RCA     R1 */
    kEmmcSleepAwake = 5,             /*!< ac       [31:16] RCA     R1b */
                                     /*!<          [15] flag */
    kEmmcSwitch = 6,                 /*!< ac       [31:16] RCA     R1b */
    kEmmcSendExtCsd = 8,             /*!< adtc                     R1 */
    kEmmcReadDataUntilStop = 11,     /*!< adtc     [31:0] data     R1 */
                                     /*!<          address */
    kEmmcBusTestRead = 14,           /*!< adtc                     R1 */
    kEmmcWriteDataUntilStop = 20,    /*!< ac       [31:0] data    R1 */
                                     /*!<          address */
    kEmmcProgramCid = 26,            /*!< adtc                     R1 */
    kEmmcEraseGroupStart = 35,       /*!< ac       [31:0] data     R1 */
                                     /*!<          address */
    kEmmcEraseGroupEnd = 36,         /*!< ac       [31:0] data     R1 */
                                     /*!<          address */
    kEmmcFastIo = 39,                /*!< ac                       R4 */
    kEmmcGoIrqState = 40,            /*!< bcr                      R5 */
} emmc_cmd_t;

/*
 * These macros enables features of EMMC driver:
 *
 * EMMC_SUP_PRE_DEF_BLK_CNT
 *  - Enables PRE-DEFINED block count freature when read/write. 
 *
 */
//#define EMMC_SUP_PRE_DEF_BLK_CNT /* EMMC card support pre-defined block count*/

/*
 * These macros define attribute of EMMC driver:
 * EMMC_BLK_SIZE_POW_ON
 *  - EMMC data block size after power on.
 */
#define EMMC_BLK_SIZE_POW_ON      512

/*!
 * @brief EMMC card classified as voltage range.
 */
typedef enum _emmc_type_as_vol_t
{
    kEmmcHighVolCard,            /*!< High voltage MMC card */
    kEmmcDualVolCard,            /*!< Dual voltage MMC card */
} emmc_type_as_vol_t;

/*!
 * @brief EMMC card classified as density level.
 */
typedef enum _emmc_type_as_density_t {
    /* EMMC card density is less than or equal 2GB,which is access as bytes */ 
    kEmmcWithin2GB,      
    /* EMMC card density is higher than 2GB, which is access as sector(512bytes) */
    kEmmcHigher2GB,       
} emmc_type_as_density_t;  

/*!
 * @brief Register OCR const and read/write function
 *
 */


/*!
 * @brief EMMC OCR register fields.
 */
#define EMMC_OCR_V170TO195_POS               7U           
#define EMMC_OCR_V170TO195_MASK              0x00000080U  /*!< 1.70¨C1.95V */
#define EMMC_OCR_V200TO260_POS               8U
#define EMMC_OCR_V200TO260_MASK              0x00007F00U  /*!< 2.0¨C2.6V */
#define EMMC_OCR_V270TO360_POS               15U
#define EMMC_OCR_V270TO360_MASK              0x00FF8000U  /*!< 2.7¨C3.6V */
#define EMMC_OCR_ACCESS_MODE_POS             29U
#define EMMC_OCR_ACCESS_MODE_MASK            0x60000000U  /*!< Access mode */ 
#define EMMC_OCR_BUSY_POS                    31U
#define EMMC_OCR_BUSY_MASK                   0x80000000U  /*!< card power up status bit (busy) */

/*!
 * @brief EMMC card access mode.
 */
typedef enum _emmc_acc_mode
{
    kEmmcAccessAsByte = 0,         /*!< The card should be accessed as byte */
    kEmmcAccessAsSector = 2,       /*!< The card should be accessed as sector */
} emmc_acc_mode_t;

/*!
 * @brief EMMC card voltage range.
 */
typedef enum _emmc_vol_range
{
    kEmmcVol170to195 = 1,          /*!< Voltage range is 1.70V to 1.95V */
    kEmmcVol270to360 = 511,         /*!< Voltage range is 2.70V to 3.60V */
} emmc_vol_range_t;

/*!
 * @brief Register OCR read/write function
 *
 */

/*!
 * @brief Register CID const
 *
 */
#define EMMC_PRO_NAME_LEN 6     /*!< MMC product name length*/

/*!
 * @brief EMMC card CID register fields.
 */
typedef struct _emmc_cid
{
    uint8_t mid;         /*!< Manufacturer ID */
    uint16_t oid;        /*!< OEM/Application ID */
    uint8_t pnm[EMMC_PRO_NAME_LEN];  /*!< Product name */
    uint8_t prv;        /*!< Product revision */
    uint32_t psn;        /*!< Product serial number */
    uint8_t mdt;       /*!< Manufacturing date */
} emmc_cid_t;

/*!
 * @brief Register CID read/write function
 *
 */


/*!
 * @brief EMMC card CSD register fields.
 */
typedef struct _emmc_csd
{
    uint8_t csdStructVer;         /*!< CSD structure [127:126]*/
    uint8_t sysSpecVer;           /*!< System specification version */
    uint8_t taac;                 /*!< Data read access-time 1 */
    uint8_t nsac;                 /*!< Data read access-time 2 in CLK cycles (NSAC*100) */
    uint8_t tranSpeed;            /*!< Max. bus clock frequency */
    uint16_t ccc;                 /*!< ard command classes */
    uint8_t readBlkLen;           /*!< Max. read data block length */
/*!< Partial blocks for read allowed [79:79]*/
#define EMMC_CSD_READ_BL_PARTIAL           (1<<0)   
/*!< Write block misalignment [78:78]*/
#define EMMC_CSD_WRITE_BLK_MISALIGN        (1<<1) 
/*!< Read block misalignment [77:77]*/
#define EMMC_CSD_READ_BLK_MISALIGN         (1<<2)  
/*!< DSR implemented [76:76] */
#define EMMC_CSD_DSR_IMP                   (1<<3)  
/*!< Write protect group enabled [31:31] */
#define EMMC_CSD_WP_GRP_ENABLED            (1<<4)  
/*!< Partial blocks for write allowed [21:21]*/
#define EMMC_CSD_WRITE_BL_PARTIAL          (1<<5) 
/*!< Content protection application [16:16]*/
#define EMMC_CSD_CONTENT_PROT_APP          (1<<6)   
/*!< File format group [15:15]*/
#define EMMC_CSD_FILE_FORMAT_GROUP         (1<<7)    
/*!< Copy flag [14:14]*/
#define EMMC_CSD_COPY                      (1<<8)   
/*!< Permanent write protection [13:13]*/
#define EMMC_CSD_PERM_WRITE_PROTECT        (1<<9)
/*!< Temporary write protection [12:12]*/
#define EMMC_CSD_TMP_WRITE_PROTECT         (1<<10)
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
} emmc_csd_t;

/*!
 * @brief Register CSD const
 *
 */

/*!
 * @brief CSD structure version.
 */
typedef enum _emmc_csd_struc_ver
{
    kEmmcCsdStrucVer10,         /*!< CSD version No. 1.0 */
    kEmmcCsdStrucVer11,         /*!< CSD version No. 1.1 */
    kEmmcCsdStrucVer12,         /*!< CSD version No. 1.2 */
    /*!< Version is coded in the CSD_STRUCTURE byte in the EXT_CSD register */
    kEmmcCsdStrucVerInExtcsd,   
} emmc_csd_struc_ver_t;

/*!
 * @brief EMMC card specification version.
 */
typedef enum _emmc_spec_ver
{
    kEmmcSpecVer0,         /*!< Allocated by MMCA */
    kEmmcSpecVer1,         /*!< Allocated by MMCA */
    kEmmcSpecVer2,         /*!< Allocated by MMCA */
    kEmmcSpecVer3,         /*!< Allocated by MMCA */
    kEmmcSpecVer414243,    /*!< Version 4.1/4.2/4.3 */
} emmc_spec_ver_t;

/*!< The mult in TRAN-SPEED field */
#define EMMC_TRAN_SPEED_FREQ_UNIT_POS   0U   /*!< Frequency unit */
#define EMMC_TRAN_SPEED_FREQ_UNIT_MASK  0x07U
#define EMMC_TRAN_SPEED_MULT_POS        3U   /*!< Multiplier factor */
#define EMMC_TRAN_SPEED_MULT_MASK       0x78U

/*!
 * @brief Register CSD read/write function
 *
 */
/*!< Read the value of frequence unit in TRAN-SPEED field */
#define RD_EMMC_CSD_TRAN_SPEED_FREQ_UNIT(CSD)  \
(((CSD.tranSpeed) & EMMC_TRAN_SPEED_FREQ_UNIT_MASK) >> EMMC_TRAN_SPEED_FREQ_UNIT_POS)
#define RD_EMMC_CSD_TRAN_SPPED_MULT(CSD) \
(((CSD.tranSpeed) & EMMC_TRAN_SPEED_MULT_MASK) >> EMMC_TRAN_SPEED_MULT_POS)  

/*!
* @brief EMMC card EXT_CSD register fields(unit:byte).
 */
typedef struct _emmc_ext_csd
{
    uint8_t supportedCmdSet;      /*!< Supported Command Sets, [504]*/
    /* Following fields only has effective value in EMMC spec V4.3 in MMC mode 
    (card spec version can be read from sysSpecVer field in CSD.): 
    from bootInfo to cardType */
    uint8_t bootInfo;             /*!< Boot information, [228] */
    uint8_t bootSizeMult;         /*!< Boot partition size, [226] */
    uint8_t accessSize;           /*!< Access size, [225] */
    uint8_t HC_ERASE_GRP_SIZE;    /*!< High-capacity erase unit size, [224] */
    uint8_t ERASE_TIMEOUT_MULT;   /*!< High-capacity erase timeout, [223] */
    uint8_t reliableWrSecCnt;     /*!< Reliable write sector count, [222] */
    uint8_t hc_wp_grp_size;       /*!< High-capacity write protect group size, [221] */
    uint8_t sleepCurrentVCC;      /*!< Sleep current (VCC), [220] */
    uint8_t sleepCurrentVCCQ;     /*!< Sleep current (VCCQ), [219] */    
    uint8_t slpAwkTimeout;        /*!< Sleep/awake timeout, [217] */
    uint32_t sectorCount;         /*!< Sector Count, [215:212] */
    /* Following fields only has effective value in MMC mode:
    from MIN_PERF_W_8_52 to PWR_CL_52_195, powerCls, HS_TIMING, busWidth */
    uint8_t MIN_PERF_W_8_52;      /*!< Minimum Write Performance for 8bit @52MHz, [210] */
    uint8_t MIN_PERF_R_8_52;      /*!< Minimum Read Performance for 8bit @52MHz, [209] */
    uint8_t MIN_PERF_W_8_26_4_52; /*! Minimum Write Performance for 8bit @26MHz/ 4bit @52MHz, [208]*/
    uint8_t MIN_PERF_R_8_26_4_52; /*!< Minimum read Performance for 8bit @26MHz/ 4bit @52MHz, [207] */
    uint8_t MIN_PERF_W_4_26;      /*!< Minimum Write Performance for 4bit @26MHz, [206] */
    uint8_t MIN_PERF_R_4_26;      /*!< Minimum Read Performance for 4bit @26MHz, [205] */
    uint8_t PWR_CL_26_360;        /*!< Power Class for 26MHz @ 3.6V, [203] */
    uint8_t PWR_CL_52_360;        /*!< Power Class for 52MHz @ 3.6V, [202] */
    uint8_t PWR_CL_26_195;        /*!< Power Class for 26MHz @ 1.95V, [201] */
    uint8_t PWR_CL_52_195;        /*< Power Class for 52MHz @ 1.95V, [200] */
    uint8_t cardType;             /*!< Card Type, [196] */
    uint8_t csdStrucVer;          /*!< CSD structure version, [194] */
    uint8_t extCsdVer;            /*!< Extended CSD revision, [192] */
    uint8_t cmdSet;               /*!< Command set, [191] */
    uint8_t cmdSetRev;            /*!< Command set revision, [189] */
    uint8_t powerCls;             /*!< Power class, [187] */
    uint8_t HS_TIMING;            /*!< High-speed interface timing, [185] */
    uint8_t busWidth;             /*!< Bus width mode, [183] */
    uint8_t erasedMemCnt;         /*!< Erased memory content, [181] */
    /* Following fields only has effective value in EMMV V4.3 in MMC mode: 
    from bootConfig to ERASE_GROUP_DEF */    
    uint8_t bootConfig;           /*!< Boot configuration, [179] */
    uint8_t bootBusWidth;         /*!< Boot bus width, [177] */
    uint8_t ERASE_GROUP_DEF;      /*!< High-density erase group definition, [175] */
} emmc_ext_csd_t;

/*! 
 * @brief Register EXT_CSD const
 *
 */
/*!
 * @brief EMMC card EXT_CSD version.
 */
typedef enum _emmc_ext_csd_ver
{
    kEmmcExtCsdVer10,    /*!< Revision 1.0 */
    kEmmcExtCsdVer11,    /*!< Revision 1.1 */
    kEmmcExtCsdVer12,    /*!< Revision 1.2 */
    kEmmcExtCsdVer13,    /*!< Revision 1.3 */
} emmc_ext_csd_ver_t;

/*!
 * @brief EXT_CSD register access mode.
 */
typedef enum _emmc_ext_csd_access_mode
{
    /*!< The command set is changed according to the Cmd Setfield of the argument  */
    kEmmcExtCsdCommandSet,    
    /*!< The bits in the pointed byte are set, according to the bits in the Value field   */
    kEmmcExtCsdSetBits,   
    /*!< The bits in the pointed byte are cleared, according to the bits in the Value field */
    kEmmcExtCsdClearBits,  
    /*!< The Value field is written into the pointed byte */
    kEmmcExtCsdWriteBits,             
} emmc_ext_csd_access_mode_t;

/*!
 * @brief EMMC card command set.
 */
typedef enum _emmc_cmd_set_t
{
    kEmmcStandardMmc,        /*!< Standard MMC */
    kEmmcCmdSet1,           
    kEmmcCmdSet2,           
    kEmmcCmdSet3,           
    kEmmcCmdSet4,   
} emmc_cmd_set_t;

typedef enum _emmc_alt_boot_t
{
    kEmmcNotSupAltBoot,  /*!< Device does not support alternate boot method */
    kEmmcSupAltBoot,     /*!< Device supports alternate boot method. */
} emmc_alt_boot_t;
/*!
 * @brief EMMC card power class used in PWR_CL_ff_vvv in EXT_CSD.
 */
typedef enum _emmc_pow_cls_t
{
    kEmmcPowClsLev0,     /*!< power class level1*/
    kEmmcPowClsLev1,     /*!< power class level2*/
    kEmmcPowClsLev2,
    kEmmcPowClsLev3,
    kEmmcPowClsLev4,
    kEmmcPowClsLev5,
    kEmmcPowClsLev6,
    kEmmcPowClsLev7,
    kEmmcPowClsLev8,
    kEmmcPowClsLev9,
    kEmmcPowClsLev10,    /*!< power class level11*/
} emmc_pow_cls_t;

/*!
 * @brief EMMC card frequence when in high speed mode.
 */
typedef enum _emmc_hs_freq_t
{
    kEmmcHSAt26MHZ = 1,  /*!< High-Speed MultiMediaCard @ 26MHz */
    kEmmcHSAt52MHZ = 3,  /*!< High-Speed MultiMediaCard @ 52MHz */
} emmc_hs_freq_t;

#define EMMC_BUSWIDTH_TYPE_NUM 3 /* The number of bus width type */
/*!
 * @brief EMMC card bus width.
 */
typedef enum _emmc_bus_width_t
{
    kEmmcBusWidth1b = 1,             /* EMMC bus width is 1 bit */
    kEmmcBusWidth4b = 4,             /* EMMC bus width is 4 bits */
    kEmmcBusWidth8b = 8,             /* EMMC bus width is 8 bits */
} emmc_bus_width_t;

/*!
 * @brief EMMC card bus timing.
 */
typedef enum _emmc_bus_timing_t
{
    kEmmcNoneHSTiming,         /*!< EMMC card using none high speed timing */
    kEmmcHSTiming,             /*!< EMMC card using high speed timing */
} emmc_bus_timing_t;

/*!
 * @brief EMMC card boot partition enablement.
 */
typedef enum _emmc_boot_part_en_t
{
    kEmmcBootNotEnabled,      /*!< No boot acknowledge sent (default)*/
    kEmmcBootPart1,           /*!< Boot partition 1 enabled for boot */
    kEmmcBootPart2,           /*!< Boot partition 2 enabled for boot */
    kEmmcBootUserAera = 7,    /*!< User area enabled for boot */ 
} emmc_boot_part_en_t;

/*!
 * @breif EMMC card boot partition to be accessed.
 */
typedef enum _emmc_access_part_t
{
    kEmmcAccessBootPartNot,   /*!< No access to boot partition (default), normal partition */
    kEmmcAccessBootPart1,     /*!< R/W boot partition 1*/
    kEmmcAccessBootPart2,     /*!< R/W boot partition 2*/
} emmc_access_part_t;
  
/*!< The power class value bit mask when bus in 4 bit mode */
#define EMMC_EXT_CSD_PWRCLFFVV_4BUS_MASK        (0x0FU)
/*!< The power class value bit mask when bus in 8 bit mode */
#define EMMC_EXT_CSD_PWRCLFFVV_8BUS_MASK        (0xF0U)


#define EMMC_CARD_TYPE_HIGH_SPEED_26MHZ_POS      0U    /*!< High-Speed MultiMediaCard @ 26MHz */
#define EMMC_CARD_TYPE_HIGH_SPEED_26MHZ_MASK     0x01U 
#define EMMC_CARD_TYPE_HIGH_SPEED_52MHZ_POS      1U    /*!< High-Speed MultiMediaCard @ 52MHz */
#define EMMC_CARD_TYPE_HIGH_SPEED_52MHZ_MASK     0x02U 

/*!
 * @brief EMMC card boot configuration definition.
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
#define EMMC_BOOT_CONFIG_BOOT_PARTITION_ACCESS_POS     0U
#define EMMC_BOOT_CONFIG_BOOT_PARTITION_ACCESS_MASK    0x00000007U
#define EMMC_BOOT_CONFIG_BOOT_PARTITION_ENABLE_POS     3U
#define EMMC_BOOT_CONFIG_BOOT_PARTITION_ENABLE_MASK    0x00000038U
#define EMMC_BOOT_CONFIG_BOOT_ACK_POS                  6U
#define EMMC_BOOT_CONFIG_BOOT_ACK_MASK                 0x00000040U
#define EMMC_BOOT_CONFIG_BOOT_BUS_WIDTH_POS            8U
#define EMMC_BOOT_CONFIG_BOOT_BUS_WIDTH_MASK           0x00000300U
#define EMMC_BOOT_CONFIG_RESET_BOOT_BUS_WIDTH_POS      10U
#define EMMC_BOOT_CONFIG_RESET_BOOT_BUS_WIDTH_MASK     0x00000400U    
    
/*!< The byte index of field in EXT_CSD */
#define EMMC_EXT_CSD_POWER_CLASS_INDEX      187   /*!< The index of POWER_CLASS */
#define EMMC_EXT_CSD__HS_TIMING_INDEX        185   /*!< The index of HS_TIMING */
#define EMMC_EXT_CSD_BUS_WIDTH_INDEX        183
#define EMMC_EXT_CSD_ERASE_GRP_DEF_INDEX    175
#define EMMC_EXT_CSD_BOOT_CONFIG_INDEX      179
#define EMMC_EXT_CSD_BOOT_BUS_WIDTH_INDEX   177

/*!
 * @brief EMMC card operation.
 */
typedef struct _emmc_ext_csd_op
{
    emmc_cmd_set_t cmdSet;
    uint8_t value;
    uint8_t indexOfByte;
    emmc_ext_csd_access_mode_t accessMode;
#define EMMC_SWITCH_PARAM_CMD_SET_POS     0U               /* Command set bit position in SWITCH command parameter */
#define EMMC_SWITCH_PARAM_CMD_SET_MASK    0x00000007U      
#define EMMC_SWITCH_PARAM_VALUE_POS       8U         /*!< The index of the byte which will be operated, 
                                                   The Index field can contain any value from 0-255,
                                                   but only values 0-191 are valid values */
#define EMMC_SWITCH_PARAM_VALUE_MASK      0x0000FF00U
#define EMMC_SWITCH_PARAM_INDEX_OF_BYTE_POS  16U
#define EMMC_SWITCH_PARAM_INDEX_OF_BYTE_MASK 0x00FF0000U
#define EMMC_SWITCH_PARAM_ACCESS_MODE_POS  24U 
#define EMMC_SWTICH_PARAM_ACCESS_MODE_MASK 0x03000000U
} emmc_ext_csd_op_t;

/*!< The length of CID, CSD, EXT_CSD register, unit of length is word(128), byte(512)) */
#define EMMC_EXT_CSD_LEN_AS_WORD                  (128U)  
#define EMMC_EXT_CSD_LEN_AS_BYTE                  (512U)

/*!
 * @brief Register EXT_CSD read/write function
 *
 */
/*!< Read the high speed frequence 26MHZ bit value */
#define RD_CARD_TYPE_HSFREQ_26MHZ(EXTCSD)  \
(((EXTCSD.cardType) & EMMC_CARD_TYPE_HIGH_SPEED_26MHZ_MASK) >> EMMC_CARD_TYPE_HIGH_SPEED_26MHZ_POS)
/*!< Read the high speed frequence 52MHZ bit value */
#define RD_CARD_TYPE_HSFREQ_52MHZ(EXTCSD) \
(((EXTCSD.cardType) & EMMC_CARD_TYPE_HIGH_SPEED_52MHZ_MASK) >> EMMC_CARD_TYPE_HIGH_SPEED_52MHZ_POS)

/*!
 * @brief The timeout value of sending and receive the response of the command.
 */
#define FSL_EMMC_REQUEST_TIMEOUT        1000

/*!< The Minimum RSA value can be assigned to the card */
#define EMMC_MINIMUM_RSA                     (2U)     
/*!< MMC card default RSA */    
#define EMMC_DEFAULT_RSA   EMMC_MINIMUM_RSA   

/*! @brief Bus test pattern when bus is at 8 bit width mode */
#define EMMC_TEST_PATTERN_8BIT_BUS     (0x0000AA55)
/*! @brief The XOR result of test pattern when bus is at 8 bit width mode */
#define EMMC_PATTERN_XOR_RES_8BIT_BUS  (0x0000FFFF)
/*!@brief Bus test pattern when bus is at 4 bit width mode */
#define EMMC_TEST_PATTERN_4BIT_BUS     (0x0000005A)
/*!@brief The XOR result of test pattern when bus is at 4 bit width mode */
#define EMMC_PATTERN_XOR_RES_4BIT_BUS  (0x000000FF) 
/*!@brief Bus test pattern when bus is at 1 bit width mode */
#define EMMC_TEST_PATTERN_1BIT_BUS     (0x80)
#define EMMC_PATTERN_XOR_RES_1BIT_BUS  (0x000000C0)









 /*!
* @brief Issues the request on a specific host controller and returns immediately.
*
* This function sents the command to the card on a specific SDHC.
* The command is sent and host will not wait the command response from the card.
* Command response and read/write data operation will be done in ISR instead of
* in this function.
*
* @param base SDHC base address
* @param host the host state inforamtion
* @return kStatus_SDHC_NoError on success
*/
sdcard_status_t SDCARD_IssueRequestNonBlocking(SDHC_Type * base, sdhc_host_t* host);

