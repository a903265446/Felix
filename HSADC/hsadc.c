

#include "hsadc.h"


void HSADC_Reset(HSADC_Type *base);
{
    uint32_t i;

    HSADC_WR_CTRL1(base, 0x5005U);
    HSADC_WR_CTRL2(base, 0x5044U);
    HSADC_WR_ZXCTRL1(base, 0x0U);
    HSADC_WR_ZXCTRL2(base, 0x0U);
    HSADC_WR_CLIST1(base, 0x3210U);
    HSADC_WR_CLIST2(base, 0x7654U);
    HSADC_WR_CLIST3(base, 0xBA98U);
    HSADC_WR_CLIST4(base, 0xFEDC);
    HSADC_WR_SDIS(base, 0xF0F0);
    HSADC_WR_STAT(base, 0x1830);
    HSADC_WR_LOLIMSTAT(base, 0xFFFF);
    HSADC_WR_HILIMSTAT(base, 0xFFFF);
    HSADC_WR_ZXSTAT(base, 0xFFFF);
    for(i = 0; i < 16; i++)
    {
        HSADC_WR_RSLT(base, i, 0x0U);
        HSADC_WR_LOLIM(base, i, 0x0U);
        HSADC_WR_HILIM(base, i, 0xFFFFU);
        HSADC_WR_OFFST(base, i, 0x0U);
    }
    HSADC_WR_PWR(base, 0x0123U);
    HSADC_WR_SCTRL(base, 0x0U);
    HSADC_WR_PWR2(base, 0x0400U);
    HSADC_WR_CTRL3(base, 0x0300U);
    HSADC_WR_SCINTEN(base, 0x0U);
    HSADC_WR_SAMPTIM(base, 0x0U);
    HSADC_WR_CALIB(base, 0x0U);
    HSADC_WR_CALVAL_A(base, 0x0U);
    HSADC_WR_CALVAL_B(base, 0x0U);
    HSADC_WR_MUX67_SEL(base, 0x0U);    
}

void HSADC_Init(HSADC_Type *base, const hsadc_user_config_t *configPtr)
{
    assert(configPtr);

    HSADC_WR_CTRL1_SMODE(base, configPtr->dualConvScanMode);
    HSADC_WR_CTRL2_SIMULT(base, (configPtr->simultaneousEnable) ? 1 : 0);
    HSADC_WR_PWR_PUDELAY(base, configPtr->powerUpDelay);
    HSADC_WR_CTRL3_DMASRC(base, configPtr->dmaTriggerSrc);
    switch (configPtr->idleWorkMode)
    {
        case kHSADCIdleKeeepNormal:
            HSADC_WR_PWR_ASB(base, 0);
            HSADC_WR_PWR_APD(base, 0);
            break;
        case kHSADCIdleAutoStandby:
            HSADC_WR_PWR_ASB(base, 1);
            HSADC_WR_PWR_APD(base, 0);
            break;
        case kHSADCIdleAutoPowerdown:
            HSADC_WR_PWR_ASB(base, 0);
            HSADC_WR_PWR_APD(base, 1);
            break;
        default:
            break;
    }
}

void HSADC_DeInit(HSADC_Type *base)
{
    HSADC_Reset(base);
}

void HSADC_ConfigConv(HSADC_Type *base, uint32_t convMask, const hsadc_conv_config_t *configPtr)
{
    uint32_t ctrl1, ctrl2, samptim, calib;

    ctrl1 = HSADC_RD_CTRL1(base);
    ctrl2 = HSADC_RD_CTRL2(base);
    samptim = HSADC_RD_SAMPTIM(base);
    pwr2 = HSADC_RD_PWR2(base);
    calib = HSADC_RD_CALIB(base);
    assert(configPtr);
     
    if (convMask & HSADC_CONVA)
    {
        ctrl1 &= ~(HSADC_CTRL1_DMAENA_MASK);
        if (configPtr->dmaEnable)
        {
            ctrl1 |= HSADC_CTRL1_DMAENA_MASK;
        }

        ctrl1 &= ~(HSADC_CTRL1_SYNCA_MASK);
        if (configPtr->syncEnable)
        {
            ctrl1 |= HSADC_CTRL1_SYNCA_MASK;
        }

        ctrl2 &= ~(HSADC_CTRL2_DIVA_MASK);
        ctrl2 |= (configPtr->clockDividor) << HSADC_CTRL2_DIVA_SHIFT;

        samptim &= ~(HSADC_SAMPTIM_SAMPT_A_MASK);
        samptim |= (configPtr->samplingTime) << HSADC_SAMPTIM_SAMPT_A_SHIFT;

        calib &= ~(HSADC_CALIB_BYPA_MASK);
        if (configPtr->calibBypassEnable)
        {
            calib |= HSADC_CALIB_BYPA_MASK;
        }
        
    }
    
    if (convMask & HSADC_CONVB)
    {
        ctrl2 &= ~(HSADC_CTRL2_DMAENB_MASK);
        if (configPtr->dmaEnable)
        {
            ctrl2 |= HSADC_CTRL2_DMAENB_MASK;
        }

        ctrl2 &= ~(HSADC_CTRL2_SYNCB_MASK);
        if (configPtr->syncEnable)
        {
            ctrl2 |= HSADC_CTRL2_SYNCB_MASK;
        }

        pwr2 &= ~(HSADC_PWR2_DIVB_MASK);
        pwr2 |= (configPtr->clockDividor) << HSADC_PWR2_DIVB_SHIFT;

        samptim &= ~(HSADC_SAMPTIM_SAMPT_B_MASK);
        samptim |= (configPtr->samplingTime) << HSADC_SAMPTIM_SAMPT_B_SHIFT;

        calib &= ~(HSADC_CALIB_BYPB_MASK);
        if (configPtr->calibBypassEnable)
        {
            calib |= HSADC_CALIB_BYPB_MASK;
        }
    } 

    HSADC_WR_CTRL1(base, ctrl1);
    HSADC_WR_CTRL2(base, ctrl2);
    HSADC_WR_SAMPTIM(base, samptim);
    HSADC_WR_PWR2(base, pwr2);
    HSADC_WR_CALIB(base, calib);
}

void HSADC_SetConvSyncCmd(HSADC_Type *base, uint32_t convMask, bool enable)
{
    if (convMask & HSADC_CONVA)
    {
        HSADC_WR_CTRL1_SYNCA(base, enable ? 1 : 0);
    }

    if (convMask & HSADC_CONVB)
    {
        HSADC_WR_CTRL2_SYNCB(base, enable ? 1 : 0);
    }
}

void HSADC_SetConvChn67Mux(HSADC_Type *base, uint32_t convMask, uint32_t muxSeletor);
{
    if (convMask & HSADC_CONVA)
    {
        HSADC_WR_MUX67_SEL_CH6_SELA(base, muxSeletor);
        HSADC_WR_MUX67_SEL_CH7_SELA(base, muxSeletor);
    }

    if (convMask & HSADC_CONVB)
    {
        HSADC_WR_MUX67_SEL_CH6_SELB(base, muxSeletor);
        HSADC_WR_MUX67_SEL_CH7_SELB(base, muxSeletor);
    }
}

void HSADC_SetConvStopModeCmd(HSADC_Type *base, uint32_t convMask, bool enable)
{
    if (convMask & HSADC_CONVA)
    {
        HSADC_WR_CTRL1_STOPA(base, (enable ? 1 : 0));
    }

    if (convMask & HSADC_CONVB)
    {
        HSADC_WR_CTRL2_STOPB(base, (enable ? 1 : 0));
    }
}

void HSADC_StartConv(HSADC_Type *base, uint32_t convMask)
{
    if (convMask & HSADC_CONVA)
    {
        HSADC_WR_CTRL1_STARTA(base, 1);
    }

    if (convMask & HSADC_CONVB)
    {
        HSADC_WR_CTRL2_STARTB(base, 1);
    }
}

void HSADC_SetConvPowerDownCmd(HSADC_Type *base, uint32_t convMask, bool enable)
{
    if (convMask & HSADC_CONVA)
    {
        HSADC_WR_PWR_PDA(base, (enable) ? 1 : 0);
    }

    if (convMask & HSADC_CONVB)
    {
        HSADC_WR_PWR_PDB(base, (enable) ? 1 : 0);
    }
}


void HSADC_SetConvIntCmd(HSADC_Type *base, uint32_t intMask, bool enable);
{
    if (enable)
    {
        if (convMask & HSADC_INT_CONVA_END_OF_SCAN)
        {
            HSADC_WR_CTRL1_EOSIEA(base, 1);
        }

        if (convMask & HSADC_INT_CONVB_END_OF_SCAN)
        {
            HSADC_WR_CTRL2_EOSIEB(base, 1);
        }

        if (convMask & HSADC_INT_ZERO_CROSS)
        {
            HSADC_WR_CTRL1_ZCIE(base, 1);
        }

        if (convMask & HSADC_INT_LOW_LIMIT)
        {
            HSADC_WR_CTRL1_LLMTIE(base, 1);
        }

        if (convMask & HSADC_INT_HIGH_LIMIT)
        {
            HSADC_WR_CTRL1_HLMTIE(base, 1);
        }

        if (convMask & HSADC_INT_CONVA_END_OF_CALIB)
        {
            HSADC_WR_CALIB_EOCALIEA(base, 1);
        }

        if (convMask & HSADC_INT_CONVB_END_OF_CALIB)
        {
            HSADC_WR_CALIB_EOCALIEB(base, 1);
        }
    }
    
    if (enable)
    {
        if (convMask & HSADC_INT_CONVA_END_OF_SCAN)
        {
            HSADC_WR_CTRL1_EOSIEA(base, 0);
        }

        if (convMask & HSADC_INT_CONVB_END_OF_SCAN)
        {
            HSADC_WR_CTRL2_EOSIEB(base, 0);
        }

        if (convMask & HSADC_INT_ZERO_CROSS)
        {
            HSADC_WR_CTRL1_ZCIE(base, 0);
        }

        if (convMask & HSADC_INT_LOW_LIMIT)
        {
            HSADC_WR_CTRL1_LLMTIE(base, 0);
        }

        if (convMask & HSADC_INT_HIGH_LIMIT)
        {
            HSADC_WR_CTRL1_HLMTIE(base, 0);
        }

        if (convMask & HSADC_INT_CONVA_END_OF_CALIB)
        {
            HSADC_WR_CALIB_EOCALIEA(base, 0);
        }

        if (convMask & HSADC_INT_CONVB_END_OF_CALIB)
        {
            HSADC_WR_CALIB_EOCALIEB(base, 0);
        }
    }

}

uint32_t HSADC_GetConvFlag(HSADC_Type *base)
{
    uint32_t result, stat, pwr;

    stat = HSADC_RD_STAT(base);
    pwr = HSADC_RD_PWR(base);

    if (stat & HSADC_STAT_CIPA_MASK)
    {
        result |= HSADC_FLAG_CONVA_CONV_IN_PROGRESS;
    }

    if (stat & HSADC_STAT_CIPB_MASK)
    {
        result |= HSADC_FLAG_CONVB_CONV_IN_PROGRESS;
    }

    if (stat & HSADC_STAT_EOSIA_MASK)
    {
        result |= HSADC_FLAG_CONVA_END_OF_SCAN;
    }

    if (stat & HSADC_STAT_EOSIB_MASK)
    {
        result |= HSADC_FLAG_CONVB_END_OF_SCAN;
    }

    if (stat & HSADC_STAT_ZCI_MASK)
    {
        result |= HSADC_FLGA_ZERO_CROSS;
    }

    if (HSADC_STAT_LLMTI_MASK)
    {
        result |= HSADC_FLAG_LOW_LIMIT;
    }

    if (HSADC_STAT_HLMTI_MASK)
    {
        result |= HSADC_FLAG_HIGH_LIMIT;
    }

    if (stat & HSADC_STAT_EOCALIA_MASK)
    {
        result |= HSADC_FLAG_CONVA_END_OF_CALIB;
    }

    if (stat & HSADC_STAT_EOCALIB_MASK)
    {
        result |= HSADC_FLAG_CONVB_END_OF_CALIB;
    }

    if (stat & HSADC_STAT_DUMMYA_MASK)
    {
        result |= HSADC_FLAG_CONVA_DUMMY_CONV_RUNNING;
    }

    if (stat & HSADC_STAT_DUMMYB_MASK)
    {
        result |= HSADC_FLAG_CONVB_DUMMY_CONV_RUNNING;
    }

    if (stat & HSADC_STAT_CALONA_MASK)
    {
        result |= HSADC_FLAG_CONVA_CALIB_RUNNING;
    }

    if (stat & HSADC_STAT_CALONB_MASK)
    {
        result |= HSADC_FLAG_CONVB_CALIB_RUNNING;
    }

    if (pwr & HSADC_PWR_PSTSA_MASK)
    {
        result |= HSADC_FLAG_CONVA_POWERED_DOWN;
    }

    if (pwr & HSADC_PWR_PSTSB_MASK)
    {
        result |= HSADC_FLAG_CONVB_POWERED_DOWN;
    }

    return result;
}

void HSADC_ClearConvFlag(HSADC_Type *base, uint32_t flagMask)
{
    uint32_t stat;

    stat = HSADC_RD_STAT(base);

    if (flagMask & HSADC_FLAG_CONVA_END_OF_SCAN)
    {
        stat &= ~HSADC_STAT_EOSIA_MASK;
    }

    if (flagMask & HSADC_FLAG_CONVB_END_OF_SCAN)
    {
        stat &= ~HSADC_STAT_EOSIB_MASK;
    }

    if (flagMask & HSADC_FLGA_ZERO_CROSS)
    {
        HSADC_WR_ZXSTAT(base, 0xFFFF);
    }

    if (flagMask & HSADC_FLAG_LOW_LIMIT)
    {
        HSADC_WR_LOLIMSTAT(base, 0xFFFF);
    }

    if (flagMask & HSADC_FLAG_HIGH_LIMIT)
    {
        HSADC_WR_HILIMSTAT(base, 0xFFFF);
    }

    if (flagMask & HSADC_FLAG_CONVA_END_OF_CALIB)
    {
        stat &= ~HSADC_STAT_EOSIA_MASK;
    }

    if (flagMask & HSADC_FLAG_CONVB_END_OF_CALIB)
    {
        stat &= ~HSADC_STAT_EOSIB_MASK;
    }

    HSADC_WR_STAT(base, stat);
}

void HSADC_ClearSeq(HSADC_Type *base)
{
    HSADC_WR_SDIS(base, 0xFFFF;
}


void HSADC_ConfigSeqSlot(HSADC_Type *base, uint32_t slotIndex, const hsadc_slot_config_t *configPtr)
{
    uint32_t clist, ctrl1, ctrl2, zxctrl, sctrl, scinten;
    uint32_t chnNum = configPtr->chnNum;

    const uint32_t clistMask[] = 
    {
        HSADC_CLIST1_SAMPLE0_MASK,
        HSADC_CLIST1_SAMPLE1_MASK,
        HSADC_CLIST1_SAMPLE2_MASK,
        HSADC_CLIST1_SMAPLE3_MASK
    } 
    const uint32_t clistShift[] = 
    {
        HSADC_CLIST1_SAMPLE0_SHIFT,
        HSADC_CLIST1_SAMPLE1_SHIFT,
        HSADC_CLIST1_SAMPLE2_SHIFT,
        HSADC_CLIST1_SMAPLE3_SHIFT
    } 

    /* Set the slot mapped channel */
    if (slotIndex < 4)
    {
        clist = HSADC_RD_CLIST1(base);
        clist &= ~(clistMask[slotIndex-0]);
        clist |= (chnNum << clistShift[slotIndex - 0])
        HSADC_WR_CLIST1(base, clist);
    }
    else if (slotIndex < 8)
    {
        clist = HSADC_RD_CLIST2(base);
        clist &= ~(clistMask[slotIndex-4]);
        clist |= (chnNum << clistShift[slotIndex - 4])
        HSADC_WR_CLIST2(base, clist);
    }
    else if (slotIndex < 12)
    {
        clist = HSADC_RD_CLIST3(base);
        clist &= ~(clistMask[slotIndex-8]);
        clist |= (chnNum << clistShift[slotIndex - 8])
        HSADC_WR_CLIST3(base, clist);
    }
    else if (slotIndex < 16)
    {
        clist = HSADC_RD_CLIST3(base);
        clist &= ~(clistMask[slotIndex-12]);
        clist |= (chnNum << clistShift[slotIndex - 12])
        HSADC_WR_CLIST3(base, clist);
    }

    /* Set channel conversion mode */
    if (chnNum < 2)
    {
        ctrl1 = HSADC_RD_CTRL1(base);
        if (configPtr->diffConvEnable)
        {
            ctrl1 |= 0x1U;
        }
        else
        {
            ctrl1 &= ~0x1U;
        }
        HSADC_WR_CTRL1_CHNCFG_L(base, ctrl1);
    }
    else if (chnNum < 4)
    {
        ctrl1 = HSADC_RD_CTRL1(base);
        if (configPtr->diffConvEnable)
        {
            ctrl1 |= 0x2U;
        }
        else
        {
            ctrl1 &= ~0x2U;
        }
        HSADC_WR_CTRL1_CHNCFG_L(base, ctrl1);
    }
    else if (chnNum < 6)
    {
        ctrl2 = HSADC_RD_CTRL2(base);
        if (configPtr->diffConvEnable)
        {
            ctrl2 |= 0x1U;
        }
        else
        {
            ctrl2 &= ~0x1U;
        }
        HSADC_WR_CTRL2_CHNCFG_H(base, ctrl2);
    }
    else if (chnNum < 8)
    {
        ctrl2 = HSADC_RD_CTRL2(base);
        if (configPtr->diffConvEnable)
        {
            ctrl2 |= 0x2U;
        }
        else
        {
            ctrl2 &= ~0x2U;
        }
        HSADC_WR_CTRL2_CHNCFG_H(base, ctrl2);
    }
    else if (chnNum < 10)
    {
        ctrl1 = HSADC_RD_CTRL1(base);
        if (configPtr->diffConvEnable)
        {
            ctrl1 |= 0x4U;
        }
        else
        {
            ctrl1 &= ~0x4U;
        }
        HSADC_WR_CTRL1_CHNCFG_L(base, ctrl1);
    }
    else if (chnNum < 12)
    {
        ctrl1 = HSADC_RD_CTRL1(base);
        if (configPtr->diffConvEnable)
        {
            ctrl1 |= 0x8U;
        }
        else
        {
            ctrl1 &= ~0x8U;
        }
        HSADC_WR_CTRL1_CHNCFG_L(base, ctrl1);
    }
    else if (chnNum < 14)
    {
        ctrl2 = HSADC_RD_CTRL2(base);
        if (configPtr->diffConvEnable)
        {
            ctrl1 |= 0x4U;
        }
        else
        {
            ctrl1 &= ~0x4U;
        }
        HSADC_WR_CTRL2_CHNCFG_H(base, ctrl2);
    }
    else if (chnNum < 16)
    {
        ctrl2 = HSADC_RD_CTRL2(base);
        if (configPtr->diffConvEnable)
        {
            ctrl1 |= 0x8U;
        }
        else
        {
            ctrl1 &= ~0x8U;
        }
        HSADC_WR_CTRL2_CHNCFG_H(base, ctrl2);
    }

    /* Set slot zero cross mode */
    if (slotIndex < 8)
    {
        zxctrl = HSADC_RD_ZXCTRL1(base);
        zxctrl &= ~(0x3U << (slotIndex*2));
        zxctrl |= (configPtr->zeroCrossingMode << (slotIndex*2))
        HSADC_WR_ZXCTRL1(base, zxctrl);
    }
    else if (slotIndex < 16)
    {
        zxctrl = HSADC_RD_ZXCTRL2(base);
        zxctrl &= ~(0x3U << ((slotIndex-8)*2));
        zxctrl |= (configPtr->zeroCrossingMode << ((slotIndex-8)*2))
        HSADC_WR_ZXCTRL1(base, zxctrl);
    }

    HSADC_WR_LOLIM(base, slotIndex, configPtr->lowLimitValue);

    HSADC_WR_HILIM(base, slotIndex, configPtr->highLimitValue);

    HSADC_WR_OFFST(base, slotIndex, configPtr->offsetValue);

    /* Set if delay sample until a new sync input occurs */
    sctrl = HSADC_RD_SCTRL(base);
    if (configPtr->slotSyncEnable)
    {
        sctrl |= HSADC_SLOT(slotIndex);
    }
    else
    {
        sctrl &= ~HSADC_SLOT(slotIndex);
    }
    HSADC_WR_SCTRL(base, sctrl);

    scinten = HSADC_RD_SCINTEN(base);
    if (configPtr->slotReadyIntEnable)
    {
        scinten |= HSADC_SLOT(slotIndex);
    }
    else
    {
        scinten &= ~HSADC_SLOT(slotIndex);
    }
    HSADC_WR_SCINTEN(base, scinten);
}

uint32_t HSADC_GetSeqFlag(HSADC_Type *base, uint32_t slotMask, hsadc_slot_status_t flagType)
{
    switch (flagType)
    {
        case kHSADCSlotFlagOfResultReady:
            return (HSADC_RD_RDY(base) & slotMask);
            // break;
        case kHSADCSlotStatusLowLimit:
            return (HSADC_RD_LOLIMSTAT(base) & slotMask);
            // break;
        case kHSADCSlotStatusHighLimit:
            return (HSADC_RD_HILIMSTAT(base) & slotMask);
            // break;
        case kHSADCSlotStatusZeroCrossing:
            return (HSADC_RD_ZXSTAT(base) & slotMask);
            // break;
        default:
            break;
    }
}

void HSADC_ClearSeqFlag(HSADC_Type *base, uint32_t slotMask, hsadc_slot_status_t flagType)
{
    switch (flagType)
    {
        case kHSADCSlotFlagOfResultReady:
            HSADC_WR_RDY(base, slotMask);
            // break;
        case kHSADCSlotStatusLowLimit:
            HSADC_WR_LOLIMSTAT(base, slotMask);
            // break;
        case kHSADCSlotStatusHighLimit:
            HSADC_WR_HILIMSTAT(base, slotMask);
            // break;
        case kHSADCSlotStatusZeroCrossing:
            HSADC_WR_ZXSTAT(base, slotMask);
            // break;
        default:
            break;
    }
}

uint32_t HSADC_GetSeqSlotConvResult(HSADC_Type *base, uint32_t slotIndex)
{
    return HSADC_RD_RSLT(base, slotIndex);
}

void HSADC_SetConvCalibCmd(HSADC_Type *base, uint32_t convMask, uint32_t calibModeMask, bool enable)
{
    if (convMask & HSADC_CONVA)
    {
        if (enable)
        {
            if (calibModeMask & HSADC_CALIB_SE)
            {
                HSADC_SET_CALIB(base, HSADC_CALIB_REQSINGA_MASK); 
            }

            if (calibModeMask & HSADC_CALIB_DIFF)
            {
                HSADC_SET_CALIB(base, HSADC_CALIB_REQDIFA_MASK);
            }

            HSADC_SET_CALIB(base, HSADC_CALIB_CAL_REQA_MASK);
        }
        else
        {
            if (calibModeMask & HSADC_CALIB_SE)
            {
                HSADC_CLR_CALIB(base, HSADC_CALIB_REQSINGA_MASK);
            }

            if (calibModeMask & HSADC_CALIB_DIFF)
            {
                HSADC_CLR_CALIB(base, HSADC_CALIB_REQDIFA_MASK);
            }

            HSADC_CLR_CALIB(base, HSADC_CALIB_CAL_REQA_MASK);
        }
       
    }

    if (convMask & HSADC_CONVB)
    {
        if (enable)
        {
            if (calibModeMask & HSADC_CALIB_SE)
            {
                HSADC_SET_CALIB(base, HSADC_CALIB_REQSINGB_MASK);
            }

            if (calibModeMask & HSADC_CALIB_DIFF)
            {
                HSADC_SET_CALIB(base, HSADC_CALIB_REQDIFB_MASK);
            }

            HSADC_SET_CALIB(base, HSADC_CALIB_CAL_REQB_MASK);  
        }
        else
        {
            if (calibModeMask & HSADC_CALIB_SE)
            {
                HSADC_CLR_CALIB(base, HSADC_CALIB_REQSINGB_MASK);
            }

            if (calibModeMask & HSADC_CALIB_DIFF)
            {
                HSADC_CLR_CALIB(base, HSADC_CALIB_REQDIFB_MASK);
            }

            HSADC_CLR_CALIB(base, HSADC_CALIB_CAL_REQB_MASK);
        }
    }
}

uint32_t HSADC_GetConvCalibValue(HSADC_Type *base, uint32_t convMask, uint32_t calibModeMask)
{
    uint32_t result = 0;

    if (convMask & kHSAdcConvA)
    {
        if (calibModeMask & HSADC_CALIB_SE)
        {
            result |= HSADC_RD_CALVAL_A_CALVSING(base) << HSADC_CONVA_SE_CALIB_VAL_SHIFT;
        }

        if (calibModeMask & HSADC_CALIB_DIFF)
        {
            result |= HSADC_RD_CALVAL_A_CALVDIF(base) << HSADC_CONVA_DIFF_CALIB_VAL_SHIFT;
        }
    }

    if (convMask & kHSAdcConvB)
    {
        if (calibModeMask & HSADC_CALIB_SE)
        {
            result |= HSADC_RD_CALVAL_B_CALVSING(base) << HSADC_CONVB_SE_CALIB_VAL_SHIFT;
        }

        if (calibModeMask & HSADC_CALIB_DIFF)
        {
            result |= HSADC_RD_CALVAL_B_CALVDIF(base) << HSADC_CONVB_DIFF_CALIB_VAL_SHIFT;
        }
    }

    return result;
}

uint32_t HSADC_CalibAtPowerUp(HSADC_Type *base, uint32_t convMask, uint32_t calibModeMask)
{
    /* Enable converter request */
    HSADC_SetConvCalibCmd(base, convMask, calibModeMask, true);

    /* Disable converter power down. */
    HSADC_SetConvPowerDownCmd(base, convMask, false);

    /* Wait converter to power up. */
    if (convMask & HSADC_CONVA)
    {
        while (HSADC_GetConvFlag(base) & HSADC_FLAG_CONVA_POWERED_DOWN){};
    }
    if (convMask & HSADC_CONVB)
    {
        while (HSADC_GetConvFlag(base) & HSADC_FLAG_CONVB_POWERED_DOWN){};
    }

    /* Wait until calib complete. */
    if (convMask & HSADC_CONVA)
    {
        while (!HSADC_GetConvFlag(base) & HSADC_FLAG_CONVA_END_OF_CALIB);
    }
    if (convMask & HSADC_CONVB)
    {
        while (!HSADC_GetConvFlag(base) & HSADC_FLAG_CONVB_END_OF_CALIB);
    }
    
    /* Clear the end of calib interrupt flag. */
    if (convMask & HSADC_CONVA)
    {
        HSADC_ClearConvFlag(base, HSADC_FLAG_CONVA_END_OF_CALIB);
    }
    if (convMask & HSADC_CONVB)
    {
        HSADC_ClearConvFlag(base, HSADC_FLAG_CONVB_END_OF_CALIB);
    }
    
    /* Disable calib request. */
    HSADC_SetConvCalibCmd(base, convMask, calibModeMask, false); 

    return HSADC_GetConvCalibValue(base, convMask, calibModeMask);
}

uint32_t HSADC_CalibAfterPowerup(HSADC_Type *base, uint32_t convMask, uint32_t calibModeMask)
{
    /* Enable calibration request. */
    HSADC_SetConvCalibCmd(base, convMask, calibModeMask, true);   

    /* Start conversion. */
    HSADC_StartConv(base, convMask);

    /* Wait until scan complete. */
    if (convMask & HSADC_CONVA)
    {
        while (!HSADC_GetConvFlag(base) & HSADC_FLAG_CONVA_END_OF_SCAN);
    }
    if (convMask & HSADC_CONVB)
    {
        while (!HSADC_GetConvFlag(base) & HSADC_FLAG_CONVB_END_OF_SCAN);
    }

    /* Wait until calibration complete. */
    if (convMask & HSADC_CONVA)
    {
        while (!HSADC_GetConvFlag(base) & HSADC_FLAG_CONVA_END_OF_CALIB);
    }
    if (convMask & HSADC_CONVB)
    {
        while (!HSADC_GetConvFlag(base) & HSADC_FLAG_CONVB_END_OF_CALIB);
    }
      
    /* Clear the end of calib flag. */
    if (convMask & HSADC_CONVA)
    {
        HSADC_ClearConvFlag(base, HSADC_FLAG_CONVA_END_OF_CALIB);   
    }
    if (convMask & HSADC_CONVB)
    {
        HSADC_ClearConvFlag(base, HSADC_FLAG_CONVB_END_OF_CALIB);   
    }

    /* Clear the end of scan flag. */
    if (convMask & HSADC_CONVA)
    {
        HSADC_ClearConvFlag(base, HSADC_FLAG_CONVA_END_OF_SCAN);
    }
    if (convMask & HSADC_CONVB)
    {
        HSADC_ClearConvFlag(base, HSADC_FLAG_CONVB_END_OF_SCAN);
    }
            
    /* Disable calibration request. */
    HSADC_SetConvCalibCmd(base, convMask, calibModeMask, false);

    return HSADC_GetConvCalibValue(base, convMask, calibModeMask);
}
