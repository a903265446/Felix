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

/*******************************************************************************
 * Definitons
 ******************************************************************************/
/* Max block count can be set */
#define SDHC_MAX_BLOCK_COUNT        ((1U << SDHC_BLKATTR_BLKCNT_WIDTH) - 1U)

/* Clock setting */
#define SDHC_MAX_DVS                (16U)      /* Max SD clock divisor from base clock */
#define SDHC_INITIAL_DVS            (1U)       /* Initial value of SD clock divisor */
#define SDHC_INITIAL_CLKFS          (2U)       /* Initial value of SD clock frequency selector */
#define SDHC_NEXT_DVS(x)            do { ((x) += 1U); } while(0)
#define SDHC_PREV_DVS(x)            do { ((x) -= 1U); } while(0)
#define SDHC_MAX_CLKFS              (256U)
#define SDHC_NEXT_CLKFS(x)          do { ((x) <<= 1U); } while(0)
#define SDHC_PREV_CLKFS(x)          do { ((x) >>= 1U); } while(0)

/* Continuously wait till the event be nodified */
#define SDHC_WAIT_EVENT_FOREVER         (1U << 32U - 1U)

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

static void SDHC_SetAdmaTable(SDHC_Type *base, uint32_t *tableAddress, uint32_t * dataBuffer, 
                      uint32_t dataLength);

/*******************************************************************************
 * Variables
 ******************************************************************************/
const version_t sdhcDriverVersion = {
    .major = 2,                /*major version: 2*/
    .minor = 0,                /*minor version: 0*/
    .bugfix = 0                /*bugfix version: 0*/
};

/*******************************************************************************
 * Code
 ******************************************************************************/

/*FUNCTION****************************************************************
 *
 * Function Name: SDHC_GetVersion
 * Description: Gets the driver version.
 *
 *END*********************************************************************/
void SDHC_GetVersion(version_t *version)
{
    assert(version);

    *version = sdhcDriverVersion;
}

/*FUNCTION****************************************************************
 *
 * Function Name: SDHC_Init
 * Description: Initializes SDHC according to the configuration structure.
 *
 *END*********************************************************************/
void SDHC_Init(SDHC_Type *base, const sdhc_config_t *config, uint32_t sourceClock, 
       sdhc_capability_t *capability);
{
    uint32_t proctl, wml;
    uint32_t htCapability, hostVer;
    uint8_t maxBlockLength;
    assert(config);
    assert(capability);

    /* Enables clock and NVIC IRQ. */
    switch ((uint32_t)base)
    {
#if defined(SDHC)
        case (uint32_t)SDHC:
        CLOCK_EnableClock(kCLOCK_Sdhc0);
        NVIC_EnableIRQ(SDHC_IRQS[0]);
#endif
        default:
            break;
    }

    /* Resets to default state. */
    SDHC_Reset(base, kSDHC_ResetAll, 100);

    proctl = base->PROCTL;
    wml = base->WML;
    hostVer = base->HOSTVER;
    htCapability = base->HTCAPBLT;

    if (config->cardDetectMode == kSDHC_CardDetectDat3)
    {
    	proctl |= SDHC_PROCTL_D3CD_MASK;
    }
    proctl &= ~SDHC_PROCTL_EMODE_MASK;
    proctl |= SDHC_PROCTL_EMODE(config->endianMode);
    proctl &= ~SDHC_PROCTL_DMAS_MASK;
    proctl |= SDHC_PROCTL_DMAS(config->dmaMode);
    
    wml &= ~SDHC_WML_RDWML_MASK;
    wml &= ~SDHC_WML_WRWML_MASK;
    wml |= SDHC_WML_RDWML(config->readWatermarkLevel);
    wml |= SDHC_WML_WRWML(config->writeWatermarkLevel);  
    
    /* Gets the capability of SDHC. */
    capability->sourceClockFreq = sourceClock;

    capability->specVersion = ((hostVer & SDHC_HOSTVER_SVN_MASK) >> SDHC_HOSTVER_SVN_SHIFT);
    capability->vendorVersion = ((hostVer & SDHC_HOSTVER_VVN_MASK) >> SDHC_HOSTVER_VVN_SHIFT);

    maxBlockLength = ((htCapability & SDHC_HTCAPBLT_MBL_MASK) >> SDHC_HTCAPBLT_MBL_SHIFT);
    switch (maxBlockLength)
    {
        case 0:
            capability->maxBlockLength = 512U;
            break;
        case 1:
            capability->maxBlockLength = 1024U;
            break;
        case 2:
            capability->maxBlockLength = 2048U;
            break;
        case 3:
            capability->maxBlockLength = 4096U;
            break;
        default: 
            break;
    }

    capability->flags = (htCapability & (kSDHC_SupportAdma
                                       | kSDHC_SupportHighSpeed
                                       | kSDHC_SupportDma
                                       | kSDHC_SupportSuspendResume
                                       | kSDHC_SupportV330));
#if defined FSL_FEATURE_SDHC_HAS_V300_SUPPORT && FSL_FEATURE_SDHC_HAS_V300_SUPPORT
    capability->flags |= (htCapability & kSDHC_SupportV300);
#endif
#if defined FSL_FEATURE_SDHC_HAS_V180_SUPPORT && FSL_FEATURE_SDHC_HAS_V180_SUPPORT
    capability->flags |= (htCapability & kSDHC_SupportV180);
#endif 
     
    SDHC_WR_WML(base, wml);
    SDHC_WR_PROCTL(base, proctl);  

}

/*FUNCTION****************************************************************
 *
 * Function Name: SDHC_Reset
 * Description: Perform different kinds of reset.
 *
 *END*********************************************************************/
void SDHC_DeInit(SDHC_Type *base)
{
    SDHC_Reset(base, SDHC_SYSCTL_RSTA_MASK, 100);
    
    switch ((uint32_t)base)
    {
#if defined(SDHC):
        case (uint32_t)SDHC:
            CLOCK_DisableClock(kCLOCK_Sdhc0);
            NVIC_DisableIRQ(SDHC_IRQS[0]);
#endif 
        default:
            break;
    }
}

/*FUNCTION****************************************************************
 *
 * Function Name: SDHC_Reset
 * Description: Performs different kinds of reset.
 *
 *END*********************************************************************/
uint32_t SDHC_Reset(SDHC_Type *base, uint32_t mask, uint32_t timeout)
{
    uint32_t mask;
    assert(timeout);

    mask = (mask & (SDHC_SYSCTL_RSTA_MASK | SDHC_SYSCTL_RSTC_MASK | SDHC_SYSCTL_RSTD_MASK));

    base->SYSCTL |= mask;
    while ((base->SYSCTL & mask))
    {
        if (!timeout)
        {
            break;
        }
        timeout--;
    }
    return (!timeout);
}

/*FUNCTION****************************************************************
 *
 * Function Name: SDHC_SetAutoGatedClock
 * Description: Sets the auto gated off feature of SDHC internal clocks.
 *
 *END*********************************************************************/
void SDHC_SetClockAutoGated(SDHC_Type *base, uint32_t mask)
{
    uint32_t mask, sysctl;

    mask = (mask & (SDHC_SYSCTL_PEREN_MASK | SDHC_SYSCTL_HCKEN_MASK | SDHC_SYSCTL_IPGEN_MASK));

    sysctl = base->SYSCTL;
    sysctl &= ~(SDHC_SYSCTL_PEREN_MASK | SDHC_SYSCTL_HCKEN_MASK | SDHC_SYSCTL_IPGEN_MASK);

    sysctl |= mask;

    base->SYSCTL = sysctl;
}

/*FUNCTION****************************************************************
 *
 * Function Name: SDHC_SetAutoGatedClock
 * Description: Sets the auto gated off feature of SDHC internal clocks.
 *
 *END*********************************************************************/
void SDHC_SetWakeupEvent(SDHC_Type *base, uint32_t mask)
{
    uint32_t mask, sysctl;

    mask = (mask & (SDHC_PROCTL_WECRM_MASK | SDHC_PROCTL_WECINS_MASK | SDHC_PROCTL_WECINT_MASK));

    sysctl = base->SYSCTL;
    sysctl &= ~(SDHC_PROCTL_WECRM_MASK | SDHC_PROCTL_WECINS_MASK | SDHC_PROCTL_WECINT_MASK);

    sysctl |= mask;

    base->SYSCTL = sysctl;
}

/*FUNCTION****************************************************************
 *
 * Function Name: SDHC_SetSdClockConfig
 * Description: Sets the clock frequence of SD_CLK pin.
 *
 *END*********************************************************************/
void SDHC_SetSdClockConfig(SDHC_Type *base, sdhc_sd_clock_config_t *config)
{
    uint32_t divisor, freq, sysCtlReg;
    assert(config);
    assert((config->sdClockFreq) && (config->sdClockFreq < config->baseClockFreq));

    divisor = SDHC_INITIAL_DVS;
    freq = SDHC_INITIAL_CLKFS;

    /* Disables SD clock and enable all clock's to be automatically gated off state. */
    base->SYSCTL &= ~(SDHC_SYSCTL_IPGEN_MASK | SDHC_SYSCTL_HCKEN_MASK | SDHC_SYSCTL_PEREN_MASK 
                    | SDHC_SYSCTL_SDCLKEN_MASK);

    /* If user want to disable the clock , directly return. */
    if(!(config->enableSdClock))
    {
        return;
    }

    if (config->sdClockFreq > 0)
    {
        while((config->baseClockFreq / freq / SDHC_MAX_DVS > config->sdClockFreq) &&
                (freq < SDHC_MAX_CLKFS))
        {
            SDHC_NEXT_CLKFS(freq);
        }
        while((config->baseClockFreq / freq / divisor > config->sdClockFreq) &&
                (divisor < SDHC_MAX_DVS))
        {
            SDHC_NEXT_DVS(divisor);
        }

        config->sdClockFreq = config->baseClockFreq / freq / divisor;
        SDHC_PREV_CLKFS(freq);
        SDHC_PREV_DVS(divisor);
        sysCtlReg = SDHC_RD_SYSCTL(base);
        /* Sets the SD clock frequency divisor. */
        sysCtlReg &= (~SDHC_SYSCTL_DVS_MASK);
        sysCtlReg |= (SDHC_SYSCTL_DVS(divisor));
        /* Sets the SD clock frequency select. */
        sysCtlReg &= (~SDHC_SYSCTL_SDCLKFS_MASK);
        sysCtlReg |= (SDHC_SYSCTL_SDCLKFS(freq));
        /* Sets the data timeout counter value. */
        sysCtlReg &= (~SDHC_SYSCTL_DTOCV_MASK);
        sysCtlReg |= (SDHC_SYSCTL_DTOCV(0xE));
        /* Enables the IPG clock and no automatic clock gating off. 
         Enables the system clock and no automatic clock gating off. 
         Enables the peripheral clock and no automatic clock gating off. */
        sysCtlReg |= (SDHC_SYSCTL_IPGEN_MASK | SDHC_SYSCTL_HCKEN_MASK | SDHC_SYSCTL_PEREN_MASK);

        base->SYSCTL = sysCtlReg;

        /* Checks whether the SD clock is stable or not. */
        while(!(base->PRSSTAT & SDHC_PRSSTAT_SDSTB_MASK)) {}

        /* Enables the SD clock. It should be disabled before changing the SD clock frequency. */
        base->SYSCTL |= SDHC_SYSCTL_SDCLKEN_MASK;
    }

}

/*FUNCTION****************************************************************
 *
 * Function Name: SDHC_SetCardActive
 * Description: Sends 80 clocks to the card to set it to be active state.
 *
 *END*********************************************************************/
uint32_t SDHC_SetCardActive(SDHC_Type *base, uint32_t timeout)
{
    assert(timeout);

    base->SYSCTL |= SDHC_SYSCTL_INITA_MASK;

    while(!(base->SYSCTL & SDHC_SYSCTL_INITA_MASK))
    {
        if (!timeout)
        {
            break;
        }
        timeout--;
    }
    return (!timeout);
}

/*FUNCTION****************************************************************
 *
 * Function Name: SDHC_SetDataWidth
 * Description: Sends 80 clocks to the card to set it to be active state.
 *
 *END*********************************************************************/
void SDHC_SetDataWidth(SDHC_Type *base, sdhc_data_width_t dataWidth)
{
    base->PROCTL &= ~SDHC_PROCTL_DTW_MASK;
    base->PROCTL |= SDHC_PROCTL_DTW(dataWidth);
}

/*FUNCTION****************************************************************
 *
 * Function Name: SDHC_EnableSdioControl
 * Description: Enables SDIO card control.
 *
 *END*********************************************************************/
void SDHC_EnableSdioControl(SDHC_Type *base, uint32_t mask)
{
    proctl = base->PROCTL;

    if (mask & kSDHC_StopAtBlockGap)
    {
        proctl |= SDHC_PROCTL_SABGREQ_MASK;
    }

    if (mask & kSDHC_ReadWaitControl)
    {
        proctl |= SDHC_PROCTL_RWCTL_MASK;
    }

    if (mask & kSDHC_IntAtBlockGap)
    {
        proctl |= SDHC_PROCTL_IABG_MASK;
    }

    if (mask & kSDHC_ExactBlockNumRead)
    {
        proctl |= SDHC_VENDOR_EXBLKNU_MASK;
    }

    base->PROCTL = proctl;
}

/*FUNCTION****************************************************************
 *
 * Function Name: SDHC_EnableSdioControl
 * Description: Disables SDIO card control.
 *
 *END*********************************************************************/
void SDHC_DisableSdioControl(SDHC_Type *base, uint32_t mask)
{
    proctl = base->PROCTL;

    if (mask & kSDHC_StopAtBlockGap)
    {
        proctl &= ~SDHC_PROCTL_SABGREQ_MASK;
    }

    if (mask & kSDHC_ReadWaitControl)
    {
        proctl &= ~SDHC_PROCTL_RWCTL_MASK;
    }

    if (mask & kSDHC_IntAtBlockGap)
    {
        proctl &= ~SDHC_PROCTL_IABG_MASK;
    }

    if (mask & kSDHC_ExactBlockNumRead)
    {
        proctl &= ~SDHC_VENDOR_EXBLKNU_MASK;
    }

    base->PROCTL = proctl;
}

/*FUNCTION****************************************************************
 *
 * Function Name: SDHC_SetCDTestLevel
 * Description: Sets the card detect test level.
 *
 *END*********************************************************************/
void SDHC_SetCDTestLevel(SDHC_Type *base, bool high)
{
    if (high)
    {
        base->PROCTL |= SDHC_PROCTL_CDTL_MASK;
    }
    else
    {
        base->PROCTL &= ~SDHC_PROCTL_CDTL_MASK;
    }
}

/*FUNCTION****************************************************************
 *
 * Function Name: SDHC_SetMmcBootConfig
 * Description: Sets MMC boot related configuration.
 *
 *END*********************************************************************/
void SDHC_SetMmcBootConfig(SDHC_Type *base, sdhc_boot_config_t *config)
{
    uint32_t mmcboot;
    assert(config);

    /* Sets boot parameter if boot configuration structure is not NULL. */
    mmcboot  = base->MMCBOOT;

    mmcboot |= SDHC_MMCBOOT_DTOCVACK(config->ackTimeoutCount);
    mmcboot |= SDHC_MMCBOOT_BOOTMODE(config->mode);
    mmcboot |= SDHC_MMCBOOT_BOOTBLKCNT(config->blockCount);
    if (config->bootAckEnable)
    {
        mmcboot |= SDHC_MMCBOOT_BOOTACK_MASK;
    }
    if (config->bootEnable)
    {
        mmcboot |= SDHC_MMCBOOT_BOOTEN_MASK;
    }
    if (config->autoStopAtBlockGapEnable)
    {
        mmcboot |= SDHC_MMCBOOT_AUTOSABGEN_MASK;
    }

    base->MMCBOOT = mmcboot;
}

/*FUNCTION****************************************************************
 *
 * Function Name: SDHC_SetTransferConfig
 * Description: Sets transfer command attribute and data size.
 *
 *END*********************************************************************/
static void SDHC_SetTransferConfig(SDHC_Type *base, const sdhc_transfer_config_t *config)
{
    uint32_t blkattr;
    assert(config);

    blkattr = base->BLKATTR;

    blkattr &= ~(SDHC_BLKATTR_BLKSIZE_MASK | SDHC_BLKATTR_BLKCNT_MASK);
    blkattr |= SDHC_BLKATTR_BLKSIZE(config->dataBlockSize);
    blkattr |= SDHC_BLKATTR_BLKCNT(config->dataBlockCount);
    base->BLKATTR = blkattr;

    base->CMDARG |= config->commandArgument;

    base->XFERTYP = ((config->commandIndex << SDHC_XFERTYP_CMDINX_SHIFT) & SDHC_XFERTYP_CMDINX_MASK)
                   | (config->flags & (SDHC_XFERTYP_DMAEN_MASK | SDHC_XFERTYP_MSBSEL_MASK 
                    | SDHC_XFERTYP_DPSEL_MASK | SDHC_XFERTYP_CMDTYP_MASK | SDHC_XFERTYP_BCEN_MASK 
                    | SDHC_XFERTYP_CICEN_MASK | SDHC_XFERTYP_CCCEN_MASK | SDHC_XFERTYP_RSPTYP_MASK 
                    | SDHC_XFERTYP_DTDSEL_MASK | SDHC_XFERTYP_AC12EN_MASK))
}


/*! @brief ADMA table configration. */
typedef struct _sdhc_adma_table_config
{
    sdhc_dma_mode_t dmaMode;         /*!< DMA mode */
    uint32_t *tableAddress;          /*!< ADMA table address */
    uint32_t maxTableEntries;        /*!< Max table entries allcated by user */

    uint32_t *dataBuffer;            /*!< Data buffer address */
    uint32_t dataLength;             /*!< Data length */    
} sdhc_adma_table_config_t;

/*FUNCTION****************************************************************
 *
 * Function Name: SDHC_SetAdmaTableConfig
 * Description: Sets each item in ADMA table and sets the ADMA table address.
 *
 *END*********************************************************************/
static void SDHC_SetAdmaTableConfig(SDHC_Type *base, sdhc_adma_table_config_t *config)
{
    sdhc_dma_mode_t dmaMode;
    uint32_t *tableAddress;
    uint32_t maxTableEntries;
    uint32_t *dataBuffer;
    uint32_t dataLength;
    uint32_t * startAddress;
    uint32_t entries, i = 0;
    sdhc_adma1_descriptor_t * adma1EntryAddress = NULL;
    sdhc_adma2_descriptor_t * adma2EntryAddress = NULL;

    dmaMode = config->dmaMode;
    tableAddress = config->tableAddress;
    maxTableEntries = config->maxTableEntries;
    dataBuffer = config->dataBuffer;
    dataLength = config->dataLength;

    startAddress = config->dataBuffer;

    switch (dmaMode)
    {
        case kSDHCDmaAdma1:
            entries = (dataLength / SDHC_ADMA1_DESC_MAX_LEN_PER_ENTRY) + 1;
            /* ADMA1 needs two descritors to finish a transfer */
            entries *= 2;
            if (entries > maxTableEntries)
            {
                return kStatus_SDHC_OutOfMemory;
            }

            adma1EntryAddress = (sdhc_adma1_descriptor_t *)(tableAddress);

            for (i = 0; i < entries; i += 2)
            {
                /* Each descriptor for ADMA1 is 64-bit in length */
                if ((dataLength - sizeof(uint32_t) * (startAddress - dataBuffer)) 
                    <= SDHC_ADMA1_DESC_MAX_LEN_PER_ENTRY)
                {
                    /* The last piece of data, setting end flag in descriptor */
                    adma1EntryAddress[i] = ((uint32_t)(dataLength - sizeof(uint32_t) * (startAddress - dataBuffer)) 
                                           << SDHC_ADMA1_DESC_LEN_SHIFT);
                    adma1EntryAddress[i] |= SDHC_ADMA1_DESC_TYPE_SET;
                    adma1EntryAddress[i+1] = ((uint32_t)(startAddress) << SDHC_ADMA1_DESC_ADDRESS_SHIFT);
                    adma1EntryAddress[i+1] |= (SDHC_ADMA1_DESC_TYPE_TRAN | SDHC_ADMA1_DESC_END_MASK);
                }
                else
                {
                    adma1EntryAddress[i] = (uint32_t)SDHC_ADMA1_DESC_MAX_LEN_PER_ENTRY << SDHC_ADMA1_DESC_LEN_SHIFT;
                    adma1EntryAddress[i] |= SDHC_ADMA1_DESC_TYPE_SET;
                    adma1EntryAddress[i+1] = (uint32_t)(startAddress) << SDHC_ADMA1_DESC_ADDRESS_SHIFT;
                    adma1EntryAddress[i+1] |= SDHC_ADMA1_DESC_TYPE_TRAN;
                    startAddress += SDHC_ADMA1_DESC_MAX_LEN_PER_ENTRY/sizeof(uint32_t);
                }
            }
            break;
        case kSDHCDmaAdma2:
            entries = ((dataLength / SDHC_ADMA2_DESC_MAX_LEN_PER_ENTRY) + 1);
            if (entries > maxTableEntries)
            {
                return kStatus_SDHC_OutOfMemory;
            }

            adma2EntryAddress = (sdhc_adma2_descriptor_t *)(tableAddress);
            for (i = 0; i < entries; i++)
            {
                /* Each descriptor for ADMA2 is 64-bit in length */
                if ((dataLength - sizeof(uint32_t) * (startAddress - dataBuffer)) <= SDHC_ADMA2_DESC_MAX_LEN_PER_ENTRY)
                {
                    /* The last piece of data, setting end flag in descriptor */
                    adma2EntryAddress[i].address = startAddress;
                    adma2EntryAddress[i].attribute = ((SDHC_ADMA2_DESC_LEN_MASK & (dataLength - sizeof(uint32_t) * (startAddress - dataBuffer))) 
                                                     << SDHC_ADMA2_DESC_LEN_SHIFT);
                    adma2EntryAddress[i].attribute |= (SDHC_ADMA2_DESC_TYPE_TRAN | SDHC_ADMA2_DESC_END_MASK);
                }
                else
                {
                    adma2EntryAddress[i].address = startAddress;
                    adma2EntryAddress[i].attribute = ((SDHC_ADMA2_DESC_LEN_MASK & SDHC_ADMA2_DESC_MAX_LEN_PER_ENTRY) 
                                                     << SDHC_ADMA2_DESC_LEN_SHIFT);
                    adma2EntryAddress[i].attribute |= SDHC_ADMA2_DESC_TYPE_TRAN;
                    startAddress += SDHC_ADMA2_DESC_MAX_LEN_PER_ENTRY/sizeof(uint32_t);
                }
            }
            break;
        default:
            break;
    }

    /* When use ADMA, disable simple DMA */
    base->DSADDR = 0;
    base->ADSADDR = (uint32_t)(config->tableAddress);
}

/*FUNCTION****************************************************************
 *
 * Function Name: SDHC_RegistHost
 * Description: Regists a host controller descriptor.
 *
 *END*********************************************************************/
status_t SDHC_RegistHost(sdhc_host_t *host)
{
    SDHC_Type *base;
    sdhc_config_t sdhcConfig = {0};
    uint32_t commandIrqEnabled = 0, dataIrqEnabled = 0; /* The IRQ status flags to enable */
    bool commandUsingIrq = false, dataUsingIRQ = false; /* If enable IRQ signal */
    assert(host);
    assert(host->sdhcConfig);
    assert(host->capability);
    assert(host->admaTableAddress);
    assert(host->CreateCommandEvent);
    assert(host->WaitCommandEvent);
    assert(host->NotifyCommandEvent);
    assert(host->DeleteCommandEvent);
    assert(host->CreateDataEvent);
    assert(host->WaitDataEvent);
    assert(host->NotifyDataEvent);
    assert(host->DeleteDataEvent);
    assert(host->SendCommand);
    assert(host->TransferData);

    base = host->base;

    if (host->SendCommand == SDHC_SendCommandPolling)
    {
        commandUsingIrq = false;
    }
    else if (host->SendCommand == SDHC_SendCommandIRQ)
    {
        commandUsingIrq = true;
    }
    else
    {
        return kStatus_InvalidArgument;
    }

    if ((host->TransferData == SDHC_TransferDataPortPolling) || (host->TransferData == SDHC_TransferAdma1Polling)
        || (host->TransferData == SDHC_TransferAdma2Polling))
    {
        dataUsingIRQ = false;
    }
    else if ((host->TransferData == SDHC_TransferDataPortIRQ) || (host->TransferData == SDHC_TransferAdma1IRQ)
        || (host->TransferData == SDHC_TransferAdma2IRQ))
    {
        dataUsingIRQ = true;
    }
    else
    {
        return kStatus_InvalidArgument;
    }

    SDHC_Init(base, host->sdhcConfig, CLOCK_GetFreq(kCLOCK_Sdhc0), config->capability);
    
    /* Enables all interrupts */
    SDHC_DisableIntStatus(base, (uint32_t)-1);
    SDHC_DisableIntSignal(base, (uint32_t)-1);

    commandIrqEnabled = (kSDHC_CommandIndexErr | kSDHC_CommandCrcErr | kSDHC_CommandEndBitErr 
                | kSDHC_CommandTimeout | kSDHC_CommandComplete);
    dataIrqEnabled = (kSDHC_DataTimeout | kSDHC_DataCrcErr | kSDHC_DataEndBitErr | kSDHC_DataComplete);

    switch (config->dmaMode)
    {
        case kSDHC_DmaModeAdma1:
        case kSDHC_DmaModeAdma2:
            dataIrqEnabled |= (kSDHC_DmaErr | kSDHC_DmaComplete);
            break;
        case kSDHC_DmaModeNoOrSimple:
            dataIrqEnabled |= (kSDHC_BuffReadReady | kSDHC_BuffWriteReady);
            break;
        default:
            break;
    }

    if (config->enableAutoCMD12)
    {
        irqEnabled |= kSDHC_AutoCommand12Err;
    }

    switch (config->cardDetectMode)
    {
        case kSDHCCardDetectGpio:
            break;
        case kSDHCCardDetectCD:
        case kSDHCCardDetectDat3:
            commandIrqEnabled |= (kSDHC_CardInsertion | kSDHC_CardRemoval);
            dataIrqEnabled |= (kSDHC_CardInsertion | kSDHC_CardRemoval);
            break;
        default:
            break;
    }

    SDHC_EnableIntStatus(base, commandIrqEnabled);
    SDHC_EnableIntStatus(base, dataIrqEnabled);

    if (commandUsingIrq)
    {
        SDHC_EnableIntSignal(base, commandIrqEnabled);
    }    
    if (dataUsingIRQ)
    {
        SDHC_EnableIntSignal(base, dataIrqEnabled);
    }
    
    return kStatus_Success;
}

/*FUNCTION****************************************************************
 *
 * Function Name: SDHC_PrepareDmaDescriptor
 * Description: Prepares DMA data for transferring
 *
 *END*********************************************************************/
static status_t SDHC_PrepareDmaDescriptor(sdhc_host_t *host) 
{
    uint32_t totalSize, entries;
    SDHC_Type *base;
    assert(host);
    assert(host->currentData)
    assert(host->currentData->buffer);
    assert(host->currentData->blockCount);
    assert(host->currentData->blockSize);
    assert(host->sdhcConfig);

    base = host->base;
    sdhc_data_t *data = host->currentData;

    totalSize = (data->blockSize * data->blockCount); 

    sdhc_adma_table_config_t admaTableConfig;

    admaTableConfig.dmaMode = host->sdhcConfig->dmaMode;
    admaTableConfig.tableAddress = host->admaTableAddress;
    admaTableConfig.maxTableEntries = host->admaTableMaxEntries;
    admaTableConfig.dataBuffer = data->buffer;
    admaTableConfig.dataLength = totalSize;
    return SDHC_SetAdmaTableConfig(base, &admaTableConfig);
}

/*FUNCTION****************************************************************
 *
 * Function Name: SDHC_ConfigAndStartTransfer
 * Description: Configures command parameter and data size then start transfer
 *
 *END*********************************************************************/
static void SDHC_ConfigAndStartTransfer(sdhc_host_t *host)
{
    SDHC_Type *base;
    uint32_t flags;
    sdhc_transfer_config_t sdhcTransferConfig;        
    sdhc_command_t *command;
    sdhc_data_t *data;
    sdhc_config_t sdhcConfig;
    assert(host);
    assert(host->current);
    assert(host->currentData);
    assert(host->sdhcConfig);

    base = host->base;
    flags = 0;
    command = host->currentCommand;
    data = host->currentData;
    sdhcConfig = host->sdhcConfig;

    /* Defines the flag corresponding to each response type. */
    switch (command->responseType)
    {
        case kSDHC_ResponseTypeNone:
            break;
        case kSDHC_ResponseTypeR1:/* Response 1 */
            flags |= (kSDHC_ResponseLength48 | SDHC_ENABLE_CRC_CHECK | SDHC_ENABLE_INDEX_CHECK);
            break;
        case kSDHC_ResponseTypeR1b:/* Response 1 with busy */            
            flags |= (kSDHC_ResponseLength48Busy | SDHC_ENABLE_CRC_CHECK | SDHC_ENABLE_INDEX_CHECK);
            break;
        case kSDHC_ResponseTypeR2:/* Response 2 */
            flags |= (kSDHC_ResponseLength136 | SDHC_ENABLE_CRC_CHECK);     
            break;
        case kSDHC_ResponseTypeR3:/* Response 3 */
            flags |= (kSDHC_ResponseLength48);
            break;
        case kSDHC_ResponseTypeR4:/* Response 4 */
            flags |= (kSDHC_ResponseLength48);
            break;
        case kSDHC_ResponseTypeR5:/* Response 5 */
            flags |= (kSDHC_ResponseLength48 | SDHC_ENABLE_CRC_CHECK);
            break;
        case kSDHC_ResponseTypeR5b:/* Response 5 with busy */
            flags |= (kSDHC_ResponseLength48Busy | SDHC_ENABLE_CRC_CHECK | SDHC_ENABLE_INDEX_CHECK);  
            break;
        case kSDHC_ResponseTypeR6:/* Response 6 */
            flags |= (kSDHC_ResponseLength48 | SDHC_ENABLE_CRC_CHECK | SDHC_ENABLE_INDEX_CHECK);    
            break;
        case kSDHC_ResponseTypeR7:/* Response 7 */
            flags |= (kSDHC_ResponseLength48 | SDHC_ENABLE_CRC_CHECK | SDHC_ENABLE_INDEX_CHECK);     
            break;
        default:
            break;
    }

    if(command->isAbortCommand)
    {
        flags |= kSDHC_CommandTypeAbort;
    }

    // while((SDHC_GetPresentState(base) & kSDHC_CommandInhibit)) {}
    
    // if ((data) || (command->responseType == kSDHC_ResponseTypeR1b) || (command->responseType == kSDHC_ResponseTypeR5b))
    // {
    //     while((SDHC_GetPresentState(base) & kSDHC_DatInhibit)) {}
    // }

    if (data)
    {
        flags |= SDHC_DATA_PRESENT;    

        if (sdhcConfig->dmaMode != kSDHC_DmaModeNoOrSimple)
        {
            flags |= kSDHC_EnableDma;
        }

        if (data->isRead)
        {
            flags |= kSDHC_EnableDataRead;
        }

        if (data->blockCount > 1)
        {
            flags |= kSDHC_MultipleBlock;
            flags |= kSDHC_EnableBlockCount;
            if (sdhcConfig->enableAutoCMD12)
            {
                /* Enable Auto CMD12 */
                flags |= kSDHC_enableAutoCMD12;
            }
            
        }

        if (data->blockCount > SDHC_MAX_BLOCK_COUNT)
        {
            sdhcTransferConfig.dataBlockSize  = data->blockSize;
            sdhcTransferConfig.dataBlockCount = SDHC_MAX_BLOCK_COUNT;
            flags &= ~kSDHC_EnableBlockCount;
        }
        else
        {
            sdhcTransferConfig.dataBlockSize  = data->blockSize;
            sdhcTransferConfig.dataBlockCount = data->blockCount;
        }
    }
    else
    {
        sdhcTransferConfig.dataBlockSize  = 0;
        sdhcTransferConfig.dataBlockCount = 0;
    }

    sdhcTransferConfig.cmdArgument  = command->argument;
    sdhcTransferConfig.cmdIndex = command->index;
    sdhcTransferConfig.flags = flags;
    SDHC_SetTransferConfig(base, &sdhcTransferConfig);

    /* Waits until command line is not busy. */
    while((SDHC_GetPresentState(base) & kSDHC_CommandInhibit)) {}
    
    /* If response type is R1b/R5b, the card may still busy after receving the response. Waits until
    data line is not busy. */
    if ((command->responseType == kSDHC_ResponseTypeR1b) || (command->responseType == kSDHC_ResponseTypeR5b))
    {
        while((SDHC_GetPresentState(base) & kSDHC_DatInhibit)) {}
    }
}



/*FUNCTION****************************************************************
 *
 * Function Name: SDHC_WaitInt
 * Description: Waits for specific interrupts
 *
 *END*********************************************************************/
static status_t SDHC_WaitInt(sdhc_host_t *host, uint32_t mask, uint32_t *irq, uint32_t timeoutInMs)
{
    SDHC_Type *base;
    status_t status = kStatus_Success;
    uint32_t startTime, currentTime, elapsedTime = 0;
    assert(host);
    assert(timeoutInMs <= SDHC_WAIT_EVENT_FOREVER);

    base = host->base;
    startTime = host->getCurrentTimeMsec();
    do
    {
        host->lastIrqFlags = SDHC_GetIntStatusFlags(base);
        *irq = ((host->lastIrqFlags) & mask);
        if (*irq)
        {
            break;
        }
        currentTime = host->getCurrentTimeMsec();
        if (currentTime < startTime)
        {
            currentTime += host->timeRangeMsec;
        }
        elapsedTime = (currentTime - startTime);
    }
    while (elapsedTime < timeoutInMs);

    if (!(*irq))
    {
        status = kStatus_Fail;
    }

    return status;
}

/*FUNCTION****************************************************************
 *
 * Function Name: SDHC_ReadBlockByDataPort
 * Description: Read a block using PIO
 *
 *END*********************************************************************/
static void SDHC_ReadBlockByDataPort(SDHC_Type *base, sdhc_data_t *data)
{
    uint32_t blockSize, blockCount;
    assert(data);    

    blockCount = data->blockCount;
    //while (SDHC_GetPresentState(base) & kSDHC_BufferReadEnable)
    //{
        blockSize = data->blockSize;
        while (blockSize)
        {
            data->buffer[data->bytesTransferred >> 2] = SDHC_GetData(base);
            data->bytesTransferred += 4;
            blockSize -= 4;
        }
        blockCount--;
        //data->blockCount--;
        if (!blockCount)
        //if (!data->blockCount)
        {
         //   break;
        }
   // }
}

/*FUNCTION****************************************************************
 *
 * Function Name: SDHC_WriteBlockByDataPort
 * Description: Writes a block using PIO
 *
 *END*********************************************************************/
static void SDHC_WriteBlockByDataPort(SDHC_Type *base, sdhc_data_t *data)
{
    uint32_t blockSize, blockCount;
    assert(data);

    blockCount = data->blockCount;
    while (SDHC_GetPresentState(base) & kSDHC_BufferWriteEnable)
    {
        blockSize = data->blockSize;
        while (blockSize)
        {
            SDHC_SetData(base, data->buffer[data->bytesTransferred >> 2]);
            data->bytesTransferred += 4;
            blockSize -= 4;
        }
        blockCount--;
        //data->blockCount--;
        if (!blockCount)
        //if (!data->blockCount)
        {
            break;
        }
    }
}

/*FUNCTION****************************************************************
 *
 * Function Name: SDHC_CardDetectIrq
 * Description: Card detection interrupt handler
 *
 *END*********************************************************************/
static void SDHC_CardDetectIrq(sdhc_host_t *host, uint32_t irq)
{
    SDHC_Type *base;
    assert(host);
    assert(irq & kSDHC_CardDetectInt);
    assert(host->cardInsertedCallback);
    assert(host->cardRemovedCallback)

    base = host->base;

    if ((irq & kSDHC_CardDetectInt) == kSDHC_CardInsertion)
    {
        if (host->cardInsertedCallback)
        {
            host->cardInsertedCallback(base);
        }
    }
    else
    {
        if (host->cardRemovedCallback)
        {
            host->cardRemovedCallback(base);
        }
    }
}

static void SDHC_ReceiveAndRealignResponse(SDHC_Type *base, sdhc_command_t *command)
{
    assert(command);

    if (command->responseType != kSDHC_ResponseTypeNone)
    {
        command->response[0] = SDHC_GetResponse(base, 0);

        if (command->responseType == kSDHC_ResponseTypeR2)
        {
            command->response[1] = SDHC_GetResponse(base, 1);
            command->response[2] = SDHC_GetResponse(base, 2);
            command->response[3] = SDHC_GetResponse(base, 3);
            i = 4;
            /* R3-R2-R1-R0[lowest 8 bit is invalid bit] has the same format as 
            spec R2 format after removed internal CRC7 and end bit. */
            do {
                command->response[i - 1] <<= 8;
                if (i > 1)
                {
                    command->response[i - 1] |= 
                        ((command->response[i-2] & 0xFF000000U) >> 24);
                }
            } while(i--);
        }
    }
}

/*FUNCTION****************************************************************
 *
 * Function Name: SDHC_CommandIrq
 * Description: Handles command related irqs
 *
 *END*********************************************************************/
static void SDHC_CommandIrq(sdhc_host_t *host, uint32_t irq)
{
    SDHC_Type *base;
    uint32_t i;
    sdhc_command_t *command;
    assert(host);
    assert(host->currentCommand);

    base = host->base;
    command = host->currentCommand;

    if (irq & kSDHC_CommandErr)
    {
        host->NotifyCommandEvent();
        return;
    }
    else if (irq & kSDHC_CommandComplete)
    {
        SDHC_ReceiveAndRealignResponse(base, host->currentCommand);
    }
    
    host->NotifyCommandEvent();
}

/*FUNCTION****************************************************************
 *
 * Function Name: SDHC_DataIrq
 * Description: Handles data related irqs
 *
 *END*********************************************************************/
static void SDHC_DataIrq(sdhc_host_t *host, uint32_t irq)
{  
    SDHC_Type *base;
    sdhc_data_t *data;
    assert(host);
    assert(host->currentData);
    assert(host->currentData->buffer);
    assert(irq & kSDHC_DataInt);

    base = host->base;
    data = host->currentData;

    if (irq & (kSDHC_DataErr | kSDHC_DmaErr))
    {
        host->NotifyDataEvent();
        return;
    }

    if (irq & kSDHC_BuffReadReady)
    {
        SDHC_ReadBlockByDataPort(base, host->currentData);
    }
    else if (irq & kSDHC_BuffWriteReady)
    {
        SDHC_WriteBlockByDataPort(base, host->currentData);
    }
    else if (irq & kSDHC_DataComplete)
    {
        host->NotifyDataEvent();
    }
    else if (irq & kSDHC_DmaComplete)
    {
        if (host->transferMode != kSDHCTransModeSdma)
        {
            return;
        }
    }
}



/*FUNCTION****************************************************************
 *
 * Function Name: SDHC_BlockGapIrq
 * Description: Block gap interrupt handler
 *
 *END*********************************************************************/
static void SDHC_BlockGapIrq(sdhc_host_t *host)
{    
    SDHC_Type *base;
    assert(host);
    assert(host->blockGapCallback);

    base = host->base;

    host->blockGapCallback(base);
}

/*FUNCTION****************************************************************
 *
 * Function Name: SDHC_CardIntIrq
 * Description: Card interrupt handler
 *
 *END*********************************************************************/
static void SDHC_CardIntIrq(sdhc_host_t *host)
{
    SDHC_Type *base;
    assert(host);
    assert(host->cardIntCallback);

    base = host->base;

    host->cardIntCallback(base);
}

/*FUNCTION****************************************************************
 *
 * Function Name: SDHC_IrqHandler
 * Description: IRQ handler
 *
 *END*********************************************************************/
void SDHC_IRQHandler(sdhc_host_t* host);
{
    SDHC_Type *base;
    uint32_t irq;
    assert(host);

    base = host->base;
    irq = SDHC_GetIntStatusFlagss(base);
    host->lastIrqFlags = irq;

    if (irq & kSDHC_CardDetectInt)
    {
        SDHC_CardDetectIrq(host, (irq & kSDHC_CardDetectInt));
    }
    if (irq & kSDHC_CommandInt)
    {
        SDHC_CommandIrq(host, (irq & kSDHC_CommandInt));
    }
    if (irq & kSDHC_DataInt)
    {
        SDHC_DataIrq(host, (irq & kSDHC_DataInt));
    }
    if (irq & kSDHC_CardInt)
    {
        SDHC_CardIntIrq(host);
    }
    if (irq & kSDHC_BlockGapEvent)
    {
        SDHC_BlockGapIrq(host);
    }

    SDHC_ClearIntStatusFlags(base, irq);
}

status_t SDHC_SendCommandPolling(sdhc_host_t *host, uint32_t timeoutInMs)
{
    uint32_t mask = 0, irqFlags = 0, i, totalSize;
    SDHC_Type *base;
    assert(host);
    assert(host->currentCommand);
    assert(host->currentData);

    base = host->base;
    data = host->currentData;

    /* DATA-PORT is 32-bit align, ADMA1 4 bytes align, ADMA2 is 4096 bytes align */
    if ((data) && (data->blockSize % 4))
    {
        return kStatus_SDHC_BlockSizeNotSupport;
    }

    if (data && (host->sdhcConfig->dmaMode != kSDHC_DmaModeNoOrSimple))
    {
        if (kStatus_Success != SDHC_PrepareDmaDescriptor(host))
        {
            return kStatus_SDHC_PrepareDmaDescriptorFailed;
        }
    }

    SDHC_ConfigAndStartTransfer(host);

    mask = (kSDHC_CommandComplete | kSDHC_CommandErr);
    if (kStatus_Success != SDHC_WaitInt(host, mask, &irqFlags, timeoutInMs))
    {
        
        host->currentCommand = 0;
        host->currentData = 0;
        return kStatus_Timeout;
    }

    if (irqFlags != kSDHC_CommandComplete)
    {
        SDHC_ClearIntStatusFlags(base, mask);
        host->currentCommand = 0;
        host->currentData = 0;
        return kStatus_Fail;
    }

    /* Command complete successfully */
    SDHC_ClearIntStatusFlags(base, kSDHC_CommandComplete);
    SDHC_ReceiveAndRealignResponse(base, host->currentCommand);

    host->currentCommand = 0;
    return kStatus_Success;
}

status_t SDHC_SendCommandIRQ(sdhc_host_t *host, uint32_t timeoutInMs)
{
    SDHC_Type *base;
    bool eventStatus; 
    sdhc_data_t data;
    assert(host);
    assert(host->currentCommand);
    assert(host->currentData);

    base = host->base;
    data = host->currentData;

    /* DATA-PORT is 32-bit align, ADMA1 4 bytes align, ADMA2 is 4096 bytes align */
    if ((data) && (data->blockSize % 4))
    {
        return kStatus_SDHC_BlockSizeNotSupport;
    }

    if (data && (host->sdhcConfig->dmaMode != kSDHC_DmaModeNoOrSimple))
    {
        if (kStatus_Success != SDHC_PrepareDmaDescriptor(host))
        {
            return kStatus_SDHC_PrepareDmaDescriptorFailed;
        }
    }

    if ( false == host->CreateCommandEvent())
    {
        return kStatus_SDHC_CreateEventFailed;
    }
    if (data)
    {
        /* Allocates semaphore for command data if command has data. */
        if ( false == host->CreateDataEvent())
        {
            return kStatus_SDHC_CreateEventFailed;
        }
    }

    SDHC_ConfigAndStartTransfer(host);

    if (!timeoutInMs)
    {
        eventStatus = host->WaitCommandEvent(SDHC_WAIT_EVENT_FOREVER);
    }
    else
    {
        eventStatus = host->WaitCommandEvent(timeoutInMs);
    }

    if (false == eventStatus)
    {
        /* Command data will not be received. */
        host->DeleteCommandEvent();
        if (data)
        {
            host->DeleteDataEvent();
        }
        host->currentCommand = 0;
        host->currentData = 0;
        return kStatus_Timeout;
    }

    host->DeleteCommandEvent();
    host->currentCommand = 0;

    if (host->lastIrqFlags & kSDHC_CommandErr)
    {
        host->currentData = 0; 
        return kStatus_Fail;
    }

    return kStatus_Success;
}

status_t SDHC_TransferDataPortPolling(sdhc_host_t *host, uint32_t timeoutInMs)
{
    SDHC_Type *base;
    uint32_t opMask, mask, i, irqFlags;
    status_t status;
    sdhc_data_t *data;
    assert(host);
    assert(host->currentData);

    base = host->base;
    data = host->currentData;

    mask = (kSDHC_DataComplete | kSDHC_DataErr);
    
    if (data->isRead)
    {
        opMask = kSDHC_BuffReadReady;
    }
    else
    {
        opMask = kSDHC_BuffWriteReady;
    }

    for (i = 0; i < data->blockCount; i++)
    {
        status = SDHC_WaitInt(host, (mask | opMask), &irqFlags, timeoutInMs);
        if (status != kStatus_Success)
        {
            host->currentData = 0;
            return kStatus_Timeout;
        }
        if (irqFlags & kSDHC_DataErr)
        {
            SDHC_ClearIntStatusFlags(base, mask);
            host->currentData = 0;
            return kStatus_Fail;
        }

        if (irqFlags & opMask)
        {
            if (data->isRead)
            {
                SDHC_ReadBlockByDataPort(base, host->currentData);
            }
            else
            {
                SDHC_WriteBlockByDataPort(base, host->currentData);
            }
            
            SDHC_ClearIntStatusFlags(base, opMask);
        }
    }

    do
    {
        status = SDHC_WaitInt(host, mask, &irqFlags, timeoutInMs);
        if (status != kStatus_Success)
        {
            host->currentData = 0;
            return kStatus_Timeout;
        }
        if (irqFlags & kSDHC_DataErr)
        {
            SDHC_ClearIntStatusFlags(base, mask);
            host->currentData = 0;
            return kStatus_Fail;
        }
    } while (!(irqFlags & kSDHC_DataComplete));

    SDHC_ClearIntStatusFlags(base, mask);

    /* Waits until data line is not busy. */
    while((SDHC_GetPresentState(base) & kSDHC_DatInhibit)) {}
    
    host->currentData = 0;
    return kStatus_Success;
}

status SDHC_TransferDataPortIRQ(sdhc_host_t *host, uint32_t timeoutInMs)
{
    SDHC_Type *base;
    sdhc_data_t *data;
    bool status;
    assert(host);
    assert(host->currentData);  

    base = host->base;
    data = host->currentData;

    if (!timeoutInMs)
    {
        status = host->WaitDataEvent(SDHC_WAIT_EVENT_FOREVER);
    }
    else
    {
        status = host->WaitDataEvent(timeoutInMs);
    }

    if (false == status)
    {
         host->DeleteDataEvent();
         return kStatus_Timeout;
    }

    host->DeleteDataEvent();

    if (host->lastIrqFlags & kSDHC_DataErr)
    {
        ret = kStatus_Fail;
    }

    /* Waits until data line is not busy. */
    while((SDHC_GetPresentState(base) & kSDHC_DatInhibit)) {}

    host->currentData = 0;
    return kStatus_Success;
}

/*FUNCTION****************************************************************
 *
 * Function Name: SDHC_TransferAdma2Polling
 * Description: Transfers data using DMA mode
 *
 *END*********************************************************************/
status SDHC_TransferAdma2Polling(sdhc_host_t *host, uint32_t timeoutInMs)
{
    SDHC_Type *base;
    uint32_t mask, irqFlags;
    status_t status;
    assert(host);
    assert(host->currentData);

    base = host->base;
    mask = (kSDHC_DataComplete | kSDHC_DmaErr);

    do
    {
        status = SDHC_WaitInt(host, mask, &irqFlags, timeoutInMs);
        if (status != kStatus_Success)
        {
            host->currentData = 0;
            return kStatus_Timeout;
        }

        if (irqFlags & kSDHC_DmaErr)
        {
            host->currentData = 0;
            return kStatus_Fail;
        }
    } while (!(irqFlags & kSDHC_DataComplete));

    SDHC_ClearIntStatusFlags(base, mask);

    /* Waits until data line is not busy. */
    while((SDHC_GetPresentState(base) & kSDHC_DatInhibit)) {}

    host->currentData = 0;
    return kStatus_Success;
}


status SDHC_TransferAdma2IRQ(sdhc_host_t *host, uint32_t timeoutInMs)
{
    SDHC_Type *base;
    sdhc_data_t *data;
    bool status;
    status_t ret = kStatus_Success;
    assert(host);
    assert(host->currentData);  

    base = host->base;
    data = host->currentData;

    if (!timeoutInMs)
    {
        status = host->WaitDataEvent(SDHC_WAIT_EVENT_FOREVER);
    }
    else
    {
        status = host->WaitDataEvent(timeoutInMs);
    }

    if (false == status)
    {
         host->DeleteDataEvent();
         return kStatus_Timeout;
    }

    host->DeleteDataEvent();

    if (host->lastIrqFlags & kSDHC_DmaErr)
    {
        ret = kStatus_Fail;
    }

    /* Waits until data line is not busy. */
    while((SDHC_GetPresentState(base) & kSDHC_DatInhibit)) {}

    host->currentData = 0;
    return ret;
}

#define SDHC_TransferAdma1Polling(host, timeoutInMs) SDHC_TransferAdma2Polling(host, timeoutInMs)

#define SDHC_TransferAdma1IRQ(host, timeoutInMs) SDHC_TransferAdma2IRQ(host, timeoutInMs);

/*FUNCTION****************************************************************
 *
 * Function Name: SDHC_DetectCard
 * Description: Checks whether the card is present on specified host
 *      controller.
 *
 *END*********************************************************************/
status_t SDHC_DetectCard(sdhc_host_t *host)
{
    SDHC_Type *base;
    assert(host);

    base = host->base;
    
    if (host->cardDetectMode == kSDHCCardDetectGpio)
    {
        return kStatus_InvalidArgument;
    }

    if (!(SDHC_GetPresentState(base) & kSDHC_CardInsertion))
    {
        return kStatus_SDHC_NoCardInsertedError;
    }
    SDHC_SetCardActive(base, 100);
    return kStatus_Success;
}
