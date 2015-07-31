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

#include <assert.h>
#include "fsl_clock_manager.h"
#include "fsl_interrupt_manager.h"
#include "sdhc.h"

#if FSL_FEATURE_SOC_SDHC_COUNT

/*FUNCTION****************************************************************
 *
 * Function Name: SDHC_InitMmcBoot
 * Description: Initializes boot configuration for MMC card.
 *
 *END*********************************************************************/

/*FUNCTION****************************************************************
 *
 * Function Name: SDHC_Init
 * Description: Initializes SDHC according to the configuration structure.
 *
 *END*********************************************************************/
void SDHC_Init(SDHC_Type * base, const sdhc_config_t* configPtr)
{
    assert(configPtr);
    
    uint32_t proctl, wml;

    proctl  = SDHC_RD_PROCTL(base);
    if (configPtr->dat3AsCardDetectPinEnable)
    {
    	proctl |= SDHC_PROCTL_D3CD_MASK;
    }
    proctl &= ~SDHC_PROCTL_EMODE_MASK;
    proctl |= SDHC_PROCTL_EMODE(configPtr->endianMode);
    proctl &= ~SDHC_PROCTL_DMAS_MASK;
    proctl |= SDHC_PROCTL_DMAS(configPtr->dmaMode);
    SDHC_WR_PROCTL(base, proctl);

    wml = SDHC_RD_WML(base);
    wml &= ~SDHC_WML_RDWML_MASK;
    wml &= ~SDHC_WML_WRWML_MASK;
    wml |= SDHC_WML_RDWML(configPtr->readWatermarkLevel);
    wml |= SDHC_WML_WRWML(configPtr->writeWatermarkLevel);
    SDHC_WR_WML(base, wml);
}

/*FUNCTION****************************************************************
 *
 * Function Name: SDHC_Reset
 * Description: Perform different kinds of reset.
 *
 *END*********************************************************************/
uint32_t SDHC_Reset(SDHC_Type * base, uint32_t resetTypeMask, uint32_t timeout)
{
    uint32_t mask;
    assert(timeout);
    mask = (resetTypeMask & (SDHC_SYSCTL_RSTA_MASK
                           | SDHC_SYSCTL_RSTC_MASK
                           | SDHC_SYSCTL_RSTD_MASK));
    SDHC_SET_SYSCTL(base, mask);
    while ((SDHC_RD_SYSCTL(base) & mask))
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
 * Function Name: SDHC_GetCapability
 * Description: Gets the capability information of SDHC.
 *
 *END*********************************************************************/
void SDHC_GetCapability(SDHC_Type * base, sdhc_capability_t * capabilityPtr)
{
    uint32_t htCapability, hostVer;
    uint8_t maxBlockLength;
    assert(capabilityPtr);

    htCapability = SDHC_RD_HTCAPBLT(base);
    hostVer = SDHC_RD_HOSTVER(base);

    capabilityPtr->specVersion = ((hostVer & SDHC_HOSTVER_SVN_MASK) >> SDHC_HOSTVER_SVN_SHIFT);
    capabilityPtr->vendorVersion = ((hostVer & SDHC_HOSTVER_VVN_MASK) >> SDHC_HOSTVER_VVN_SHIFT);
    maxBlockLength = ((htCapability & SDHC_HTCAPBLT_MBL_MASK) >> SDHC_HTCAPBLT_MBL_SHIFT);
    switch (maxBlockLength)
    {
        case 0:
            capabilityPtr->maxBlockLength = 512U;
            break;
        case 1:
            capabilityPtr->maxBlockLength = 1024U;
            break;
        case 2:
            capabilityPtr->maxBlockLength = 2048U;
            break;
        case 3:
            capabilityPtr->maxBlockLength = 4096U;
            break;
        default: 
            break;
    }
    capabilityPtr->supportMask = (htCapability & (SDHC_SUPPORT_ADMA
                                               | SDHC_SUPPORT_HIGHSPEED
                                               | SDHC_SUPPORT_DMA
                                               | SDHC_SUPPORT_SUSPEND_RESUME
                                               | SDHC_SUPPORT_V330));
#if FSL_FEATURE_SDHC_HAS_V300_SUPPORT
    capabilityPtr->supportMask |= (htCapability & SDHC_SUPPORT_V300);
#endif
#if FSL_FEATURE_SDHC_HAS_V180_SUPPORT
    capabilityPtr->supportMask |= (htCapability & SDHC_SUPPORT_V180);
#endif 
}

/*FUNCTION****************************************************************
 *
 * Function Name: SDHC_SetSdClock
 * Description: Sets the clock frequence of SD_CLK pin.
 *
 *END*********************************************************************/
void SDHC_SetSdClock(SDHC_Type * base, sdhc_sd_clock_config_t* configPtr)
{
    uint32_t divisor, freq, sysCtlReg;
    assert(configPtr);
    divisor = SDHC_INITIAL_DVS;
    freq = SDHC_INITIAL_CLKFS;
    /* Disables SD clock and enable all clock's to be automatically gated off state. */
    SDHC_CLR_SYSCTL(base, (SDHC_SYSCTL_IPGEN_MASK | SDHC_SYSCTL_HCKEN_MASK | \
      SDHC_SYSCTL_PEREN_MASK | SDHC_SYSCTL_SDCLKEN_MASK));
    /* If user want to disable the clock , directly return. */
    if(!(configPtr->sdClockEnable))
    {
        return;
    }
    if (configPtr->sdClockFreq > 0)
    {
        while((configPtr->baseClockFreq / freq / SDHC_MAX_DVS > configPtr->sdClockFreq) &&
                (freq < SDHC_MAX_CLKFS))
        {
            SDHC_NEXT_CLKFS(freq);
        }
        while((configPtr->baseClockFreq / freq / divisor > configPtr->sdClockFreq) &&
                (divisor < SDHC_MAX_DVS))
        {
            SDHC_NEXT_DVS(divisor);
        }

        configPtr->sdClockFreq = configPtr->baseClockFreq / freq / divisor;
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
        SDHC_WR_SYSCTL(base, sysCtlReg);
        /* Checks whether the SD clock is stable or not. */
        while(!SDHC_BRD_PRSSTAT_SDSTB(base)) {}
        /* nables the SD clock. It should be disabled before changing the SD clock frequency. */
        SDHC_SET_SYSCTL(base, SDHC_SYSCTL_SDCLKEN_MASK);
    }

}

/*FUNCTION****************************************************************
 *
 * Function Name: SDHC_SetCardActive
 * Description: Sends 80 clocks to the card to set it to be active state.
 *
 *END*********************************************************************/
uint32_t SDHC_SetCardActive(SDHC_Type * base, uint32_t timeout)
{
    assert(timeout);
    SDHC_BWR_SYSCTL_INITA(base, 1);
    while((!SDHC_BRD_SYSCTL_INITA(base)))
    {
        if (!timeout)
        {
            break;
        }
        timeout--;
    }
    return (!timeout);
}
uint32_t SDHC_GetAdmaEntryNumber(SDHC_Type * base, uint32_t dataLength)
{
    sdhc_dma_mode_t dmaMode;
    uint32_t entries = 0;

    dmaMode = (sdhc_dma_mode_t)SDHC_BRD_PROCTL_DMAS(base);
    switch (dmaMode)
    {
        case kSdhcDmaAdma1:
            /* Check data length alignment */
            /*if (dataLength % SDHC_ADMA1_LEN_ALIGN)
            {
                return 0;
            }*/
            entries = (dataLength / SDHC_ADMA1_DESC_MAX_LEN_PER_ENTRY) + 1;
            /* ADMA1 needs two descritors to finish a transfer */
            entries *= 2;
            break;
        case kSdhcDmaAdma2:
            /* Check data length alignment */
            /*if (dataLength % SDHC_ADMA2_LEN_ALIGN)
            {
                return 0;
            }*/
            entries = ((dataLength / SDHC_ADMA2_DESC_MAX_LEN_PER_ENTRY) + 1);
            break;
        default:
            break;
    }
    return entries;
}
void SDHC_SetAdmaTable(SDHC_Type * base, uint32_t* tableAddress, uint32_t * dataBuffer, uint32_t dataLength)
{
    sdhc_dma_mode_t dmaMode;
    uint32_t * startAddress;
    uint32_t entries, i = 0;
    sdhc_adma1_descriptor_t * adma1EntryAddress = NULL;
    sdhc_adma2_descriptor_t * adma2EntryAddress = NULL;
    dmaMode = (sdhc_dma_mode_t)SDHC_BRD_PROCTL_DMAS(base);
    startAddress = dataBuffer;
    switch (dmaMode)
    {
        case kSdhcDmaAdma1:
            entries = (dataLength / SDHC_ADMA1_DESC_MAX_LEN_PER_ENTRY) + 1;
            /* ADMA1 needs two descritors to finish a transfer */
            entries *= 2;
            adma1EntryAddress = (sdhc_adma1_descriptor_t *)tableAddress;
            //adma1EntryAddress = (sdhc_adma1_descriptor_t *);
            for (i = 0; i < entries; i += 2)
            {
                /* Each descriptor for ADMA1 is 64-bit in length */
                if ((dataLength - sizeof(uint32_t) * (startAddress - dataBuffer)) < SDHC_ADMA1_DESC_MAX_LEN_PER_ENTRY)
                {
                    /* The last piece of data, setting end flag in descriptor */
                    adma1EntryAddress[i] = ((uint32_t)(dataLength - sizeof(uint32_t) * (startAddress - dataBuffer)) << SDHC_ADMA1_DESC_LEN_SHIFT);
                    adma1EntryAddress[i] |= SDHC_ADMA1_DESC_TYPE_SET;
                    adma1EntryAddress[i+1] = (uint32_t)(startAddress) << SDHC_ADMA1_DESC_ADDRESS_SHIFT;
                    adma1EntryAddress[i+1] |= SDHC_ADMA1_DESC_TYPE_TRAN | SDHC_ADMA1_DESC_END_MASK;
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
        case kSdhcDmaAdma2:
            entries = ((dataLength / SDHC_ADMA2_DESC_MAX_LEN_PER_ENTRY) + 1);
            //adma2EntryAddress = (sdhc_adma2_descriptor_t *)SDHC_RD_ADSADDR(base);
            adma2EntryAddress = (sdhc_adma2_descriptor_t *)tableAddress;
            for (i = 0; i < entries; i++)
            {
                /* Each descriptor for ADMA2 is 64-bit in length */
                if ((dataLength - sizeof(uint32_t) * (startAddress - dataBuffer)) <= SDHC_ADMA2_DESC_MAX_LEN_PER_ENTRY)
                {
                    /* The last piece of data, setting end flag in descriptor */
                    adma2EntryAddress[i].address = startAddress;
                    adma2EntryAddress[i].attribute = ((SDHC_ADMA2_DESC_LEN_MASK 
                                                   & (dataLength - sizeof(uint32_t) * (startAddress - dataBuffer))) 
                                                   << SDHC_ADMA2_DESC_LEN_SHIFT);
                    adma2EntryAddress[i].attribute |= (SDHC_ADMA2_DESC_TYPE_TRAN | SDHC_ADMA2_DESC_END_MASK);
                }
                else
                {
                    adma2EntryAddress[i].address = startAddress;
                    adma2EntryAddress[i].attribute = ((SDHC_ADMA2_DESC_LEN_MASK & SDHC_ADMA2_DESC_MAX_LEN_PER_ENTRY) << SDHC_ADMA2_DESC_LEN_SHIFT);
                    adma2EntryAddress[i].attribute |= SDHC_ADMA2_DESC_TYPE_TRAN;
                    startAddress += SDHC_ADMA2_DESC_MAX_LEN_PER_ENTRY/sizeof(uint32_t);
                    //adma2EntryAddress++;
                }
            }
            break;
        default:
            break;
    }
    /* When use ADMA, disable simple DMA */
    SDHC_WR_DSADDR(base, 0);
    SDHC_WR_ADSADDR(base, (uint32_t)tableAddress);
}

/*FUNCTION****************************************************************
 *
 * Function Name: SDHC_SetEnableCmd
 * Description: Sets the enablement command for SDHC.
 *
 *END*********************************************************************/
void SDHC_SetEnableCmd(SDHC_Type * base, sdhc_enable_cmd_t command, bool enable)
{
    if (enable)
    {
        switch (command)
        {
            case kSdhcSetCardDetectLevelForTest:
				SDHC_BWR_PROCTL_CDSS(base, 1);
				break;
			case kSdhcSetCardDetectTestLevel:
				SDHC_BWR_PROCTL_CDTL(base, 1);
				break;
            case kSdhcStopAtBlockGapEnable:
                SDHC_BWR_PROCTL_SABGREQ(base, 1);
                break;
            case kSdhcReadWaitControlEnable:
                SDHC_BWR_PROCTL_RWCTL(base, 1);
                break;
            case kSdhcIntAtBlockGap:
                SDHC_BWR_PROCTL_IABG(base, 1);
                break;
            case kSdhcExactBlockNumRead:
                SDHC_BWR_VENDOR_EXBLKNU(base, 1);
                break;
            default:
                break;
        }
    }
    else
    {
        switch (command)
        {
            case kSdhcSetCardDetectLevelForTest:
				SDHC_BWR_PROCTL_CDSS(base, 0);
				break;
			case kSdhcSetCardDetectTestLevel:
				SDHC_BWR_PROCTL_CDTL(base, 0);
				break;
            case kSdhcStopAtBlockGapEnable:
                SDHC_BWR_PROCTL_SABGREQ(base, 0);
                break;
            case kSdhcReadWaitControlEnable:
                SDHC_BWR_PROCTL_RWCTL(base, 0);
                break;
            case kSdhcIntAtBlockGap:
                SDHC_BWR_PROCTL_IABG(base, 0);
                break;
            case kSdhcExactBlockNumRead:
                SDHC_BWR_VENDOR_EXBLKNU(base, 0);
                break;
            default:
                break;
        }
    }
    
}

/*FUNCTION****************************************************************
 *
 * Function Name: SDHC_SetCardCommand
 * Description: Sets card command related properties.
 *
 *END*********************************************************************/
void SDHC_SetCardCommand(SDHC_Type * base, const sdhc_card_cmd_config_t* configPtr)
{
    assert(configPtr);

    SDHC_BWR_BLKATTR_BLKSIZE(base, configPtr->dataBlockSize);
    SDHC_BWR_BLKATTR_BLKCNT(base, configPtr->dataBlockCount);
    SDHC_WR_CMDARG(base, configPtr->argument);
    SDHC_WR_XFERTYP(base, ((configPtr->cmdIndex << SDHC_XFERTYP_CMDINX_SHIFT) & SDHC_XFERTYP_CMDINX_MASK)
            | (configPtr->cmdFlags & (SDHC_XFERTYP_DMAEN_MASK | SDHC_XFERTYP_MSBSEL_MASK | SDHC_XFERTYP_DPSEL_MASK
                | SDHC_XFERTYP_CMDTYP_MASK | SDHC_XFERTYP_BCEN_MASK | SDHC_XFERTYP_CICEN_MASK
                | SDHC_XFERTYP_CCCEN_MASK | SDHC_XFERTYP_RSPTYP_MASK | SDHC_XFERTYP_DTDSEL_MASK
                | SDHC_XFERTYP_AC12EN_MASK)));
}

/*!
 * @brief Enables the specified interrupts.
 *
 * This function can set multiple interrupt enable bits by using the bit 
 * mask defined from SDHC_CMD_COMPLETE_INT to SDHC_DMA_ERR_INT.
 *
 * @param base SDHC base address.
 * @param enable Enable or disable interrupt.
 * @param intMask The mask to specify interrupts to be enable.
 */
void SDHC_SetIntSignal(SDHC_Type * base, bool enable, uint32_t intMask)
{
    if (enable)
    {
        SDHC_SET_IRQSIGEN(base, intMask);
    }
    else
    {
        SDHC_CLR_IRQSIGEN(base, intMask);
    }
}

void SDHC_SetIntState(SDHC_Type * base, bool enable, uint32_t intMask)
{
    if (enable)
    {
        SDHC_SET_IRQSTATEN(base, intMask);
    }
    else
    {
        SDHC_CLR_IRQSTATEN(base, intMask);
    }
}

/*FUNCTION****************************************************************
 *
 * Function Name: SDHC_SetIntCmd
 * Description: Enables the specified interrupts.
 *
 *END*********************************************************************/
void SDHC_SetPowerState(SDHC_Type * base, const sdhc_power_config_t * configPtr)
{
    uint32_t wakeupEventMask;
    assert(configPtr);

    /* Sets the clock auto gated off feature. */
	if (configPtr->powerSavingEnable)
	{
	    SDHC_BWR_SYSCTL_IPGEN(base, 1);
    	SDHC_BWR_SYSCTL_HCKEN(base, 1);
    	SDHC_BWR_SYSCTL_PEREN(base, 1);
	}
	else
	{
	    SDHC_BWR_SYSCTL_IPGEN(base, 0);
    	SDHC_BWR_SYSCTL_HCKEN(base, 0);
    	SDHC_BWR_SYSCTL_PEREN(base, 0);

		/* Sets the wakeup events in low power mode. */
		wakeupEventMask = (configPtr->lowPowerWakeupEventMask & (SDHC_WAKEUP_ON_CARD_INT
		                                                       | SDHC_WAKEUP_ON_CARD_INSERT
		                                                       | SDHC_WAKEUP_ON_CARD_REMOVE));
		SDHC_SET_PROCTL(base, wakeupEventMask);
	}
}

void SDHC_SetMmcBoot(SDHC_Type * base, sdhc_boot_config_t* configPtr)
{
    uint32_t mmcboot;
    assert(configPtr);
    /* Sets boot parameter if boot configuration structure is not NULL. */
    mmcboot  = SDHC_RD_MMCBOOT(base);
    mmcboot |= SDHC_MMCBOOT_DTOCVACK(base);
    mmcboot |= SDHC_MMCBOOT_BOOTMODE(base);
    mmcboot |= SDHC_MMCBOOT_BOOTBLKCNT(base);
    if (configPtr->bootAckEnable)
    {
        mmcboot |= SDHC_MMCBOOT_BOOTACK_MASK;
    }
    if (configPtr->bootEnable)
    {
        mmcboot |= SDHC_MMCBOOT_BOOTEN_MASK;
    }
    if (configPtr->autoStopAtBlockGapEnable)
    {
        mmcboot |= SDHC_MMCBOOT_AUTOSABGEN_MASK;
    }
}

sdhc_status_t SDHC_InitHost(uint32_t instance, const sdhc_config_t* hostConfig)
{
    SDHC_Type *base = &SDHC[instance];
    CLOCK_SYS_EnableSdhcClock(instance);
    SDHC_Init(base, hostConfig);
    INT_SYS_EnableIRQ(SDHC_IRQn);
    return kStatus_SDHC_NoError;
}

sdhc_status_t SDHC_DeInitHost(uint32_t instance)
{
    SDHC_Type *base = &SDHC[instance];
    CLOCK_SYS_DisableSdhcClock(instance);
    INT_SYS_DisableIRQ(SDHC_IRQn);
    SDHC_Reset(base, SDHC_RST_TYPE_ALL, 100);
    return kStatus_SDHC_NoError;
}
#endif /* #if FSL_FEATURE_SOC_SDHC_COUNT */