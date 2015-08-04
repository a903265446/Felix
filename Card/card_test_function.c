#include <stdint.h>
#include <string.h>

#include "fsl_clock_manager.h"
#include "fsl_interrupt_manager.h"
#include "fsl_gpio_hal.h"
#include "fsl_port_hal.h"
#include "fsl_debug_console.h"
#include "board.h"

#include "card_test_function.h"


#define TEST_BLOCK_NUM 8
#define TEST_START_BLOCK        4U
#define LOOPS 100
static uint8_t g_refData[FSL_CARD_DEFAULT_BLOCK_SIZE*TEST_BLOCK_NUM];
static uint8_t g_testData[FSL_CARD_DEFAULT_BLOCK_SIZE*TEST_BLOCK_NUM];

static const char *g_month_str[] = {
    "Jan", "Feb", "Mar", "Apr", "May", "Jun",
    "Jul", "Aug", "Sep", "Oct", "Nov", "Dec",
};
/* The time unit and value in the TAAC in CSD register */
static const uint32_t g_taac_time_unit_ns[] = {1U, 10U, 100U, 1000U, 10000U,\
  100000U, 1000000U, 10000000U};
static const uint32_t g_taac_time_value[] = {0U, 10U, 12U, 13U, 15U, 20U, 25U, 30U, \
  35U, 40U, 45U, 50U, 55U, 60U, 70U, 80U};
#define TAAC_TIME_VALUE_DIV 10  /* The divisor of time value in the TAAC in CSD register */
/* The transfer rate unit and time value in TRAN_SPEED in CSD register */
static const uint32_t g_transpeeed_transfer_rate_unit[] = {10U, 100000U, 1000000U, \
  10000000U, 0U, 0U, 0U, 0U};
static const uint32_t g_transpeeed_time_value[] = {0U, 10U, 12U, 13U, 15U, 20U, \
  25U, 30U, 35U, 40U, 45U, 50U, 55U, 60U, 70U, 80U};

/* Host controller instance number */
#define HOST_INSTANCE 0
#define CD_USING_GPIO

static host_t g_host = {0};
static sd_t g_sd = {0};
static host_capability_t g_hostCapability; 
#define ADMA_TABLE_MAX_ENTRIES 2
static uint32_t g_admaTable[ADMA_TABLE_MAX_ENTRIES];

//#if defined CD_USING_GPIO
static volatile uint32_t cdEvent = 0;
static volatile uint32_t g_sdInsertedFlag = 0, g_sdInitFlag = 0;
GPIO_Type * const g_gpioBase[GPIO_INSTANCE_COUNT] = GPIO_BASE_PTRS;
PORT_Type * const g_portBase[PORT_INSTANCE_COUNT] = PORT_BASE_PTRS;
const IRQn_Type g_portIrqId[PORT_INSTANCE_COUNT] = PORT_IRQS;
//#endif

#if defined FSL_CARD_DRIVER_USING_IRQ
void SDHC_IRQHandler(void)
{
    SDMMC_IrqHandler(&g_host);
}
#endif /* FSL_CARD_DRIVER_USING_IRQ */

#if defined TWR_K64F120M
#define SDHC_CD_GPIO_PORT GPIOB_IDX
#define SDHC_CD_GPIO_PIN  20
#define SDHC_D3_PORT      GPIOE_IDX
#define SDHC_D3_PIN       4
#endif /* defined TWR_K64F120M */

#if defined TWR_K60F120M
#define SDHC_CD_GPIO_PORT GPIOE_IDX
#define SDHC_CD_GPIO_PIN  28
#define SDHC_D3_PORT      GPIOE_IDX
#define SDHC_D3_PIN       4
#endif /* defined TWR_K60F120M */

void sdhc_cd_irqhandler(void)
{
    if (GPIO_HAL_ReadPinInput(g_gpioBase[SDHC_CD_GPIO_PORT], SDHC_CD_GPIO_PIN))
#if defined FRDM_K64F
        g_sdInsertedFlag = 1;
    else
        g_sdInsertedFlag = 0;
#elif defined TWR_K64F120M || defined TWR_K60D100M || defined TWR_K21F120M || defined TWR_K65F180M || defined TWR_K80F150M
        g_sdInsertedFlag = 0;
    else
        g_sdInsertedFlag = 1;

    notifyCardDetectEvent();
#else
#error unknown board
#endif
}

#if defined TWR_K64F120M
void PORTB_IRQHandler()
{
    if(PORT_HAL_GetPortIntFlag(PORTB) == (1<<SDHC_CD_GPIO_PIN))
    {
        sdhc_cd_irqhandler();
    }
    /* Clear interrupt flag.*/
    PORT_HAL_ClearPortIntFlag(PORTB);
}  
#endif /* defined TWR_K64F120M */

#if defined TWR_K60F120M
void PORTB_IRQHandler()
{
    if(PORT_HAL_GetPortIntFlag(PORTE) == (1<<SDHC_CD_GPIO_PIN))
    {
        sdhc_cd_irqhandler();
    }
    /* Clear interrupt flag.*/
    PORT_HAL_ClearPortIntFlag(PORTE);
}  
#endif /* defined TWR_K60F120M */

//#endif /* CD_USING_GPIO */

static uint32_t set_sdhc_pin_mux()
{
    PORT_Type * portBase = g_portBase[SDHC_D3_PORT];

    uint32_t instance = HOST_INSTANCE;
    uint32_t err = 0;

    configure_sdhc_pins(instance);

    switch(instance)
    {
        case 0:
#if defined (CD_USING_DAT3) || defined (CD_USING_POLL_DAT3)
            PORT_HAL_SetMuxMode(portBase, SDHC_D3_PIN, kPortMuxAlt4);
            PORT_HAL_SetPullCmd(portBase, SDHC_D3_PIN, true);
            PORT_HAL_SetDriveStrengthMode(PORTE, SDHC_D3_PIN, kPortHighDriveStrength);
#else
            PORT_HAL_SetMuxMode(portBase, SDHC_D3_PIN, kPortMuxAlt4);
            PORT_HAL_SetPullMode(portBase, SDHC_D3_PIN, kPortPullUp);
            PORT_HAL_SetPullCmd(portBase, SDHC_D3_PIN, true);
            PORT_HAL_SetDriveStrengthMode(portBase, SDHC_D3_PIN, kPortHighDriveStrength);
#endif
            break;
        default:
            err = 1;
            break;
    }
    return err;
}

void init_hardware(void)
{
    GPIO_Type * gpioBase = g_gpioBase[SDHC_CD_GPIO_PORT];
    PORT_Type * portBase = g_portBase[SDHC_CD_GPIO_PORT];

    MPU->CESR &= (uint32_t) ~(0x1); /* Disable MPU */

    hardware_init();

    set_sdhc_pin_mux();

    /* Set current pin as gpio.*/
    PORT_HAL_SetMuxMode(portBase, SDHC_CD_GPIO_PIN, kPortMuxAsGpio);
    /* Set current pin as digital input.*/
    GPIO_HAL_SetPinDir(gpioBase, SDHC_CD_GPIO_PIN, kGpioDigitalInput);
    /* Configure GPIO input features. */
#if FSL_FEATURE_PORT_HAS_PULL_ENABLE
    PORT_HAL_SetPullCmd(portBase, SDHC_CD_GPIO_PIN, true);
#endif
#if FSL_FEATURE_PORT_HAS_PULL_SELECTION
    PORT_HAL_SetPullMode(portBase, SDHC_CD_GPIO_PIN, kPortPullUp);
#endif
#if FSL_FEATURE_GPIO_HAS_INTERRUPT_VECTOR
    PORT_HAL_SetPinIntMode(portBase, SDHC_CD_GPIO_PIN, kPortIntEitherEdge);
    /* Configure NVIC */
        /* Enable GPIO interrupt.*/
    INT_SYS_EnableIRQ(g_portIrqId[SDHC_CD_GPIO_PORT]);
#endif
}

static uint32_t fill_reference_data(uint8_t *pdata, uint8_t seed, uint32_t len)
{
    uint32_t i, j;
    if (!pdata || !len)
        return 1;
    memset(pdata, 0, len);
    for (i = 0; i < len; i++)
    {
        j = i % 4;
        switch(j)
        {
            case 0:
                pdata[i] = 0xA0 | (seed & 0x0F);
                break;
            case 1:
                pdata[i] = 0x0B | (seed & 0xF0);
                break;
            case 2:
                pdata[i] = 0xC0 | (seed & 0x0F);
                break;
            case 3:
                pdata[i] = 0x0D | (seed & 0xF0);
                break;
        }
    }
    return 0;
}

static void show_card_cid(sd_cid_t *cid)
{
    char temp[32];
    PRINTF("Manufacturer ID: 0x%X\r\n", cid->mid);
    PRINTF("OEM ID: 0x%X\r\n", cid->oid);
    PRINTF("Product name: %s\r\n", cid->pnm);
    PRINTF("Product serial number: 0x%X\r\n", cid->psn);
    memset(temp, 0, sizeof(temp));
    PRINTF(temp, 4, "%u.%u", (cid->prv & 0xF0) >> 4, (cid->prv & 0xF));
    PRINTF("Product revision: %s\r\n", temp);
    memset(temp, 0, sizeof(temp));
    PRINTF(temp, sizeof(temp),
            "%s %u",
            g_month_str[(cid->mdt & 0xF) - 1], 2000 + ((cid->mdt & 0xFF0) >> 4));
    PRINTF("Manufacturing data: %s\r\n", temp);
}

static uint32_t decode_taac(uint8_t taac)
{
    uint32_t t1, t2;
    t1 = taac & 0x7;
    t2 = (taac & 0x78) >> 3;
    return ((g_taac_time_unit_ns[t1] * g_taac_time_value[t2])/TAAC_TIME_VALUE_DIV) ;
}

static uint32_t decode_transpeed(uint8_t ts)
{
    uint32_t t1, t2;
    t1 = ts & 0x7;
    t2 = (ts & 0x78) >> 3;    
    return (g_transpeeed_transfer_rate_unit[t1] * g_transpeeed_time_value[t2]);
}

static void show_card_csd(sd_csd_t *csd)
{
    uint32_t i = 0;
    PRINTF("CSD Structure: 0x%X\r\n", csd->csdStructure);
    PRINTF("taac: %d ns\r\n", decode_taac(csd->taac));
    PRINTF("nsac: %d clks\r\n", csd->nsac);
    PRINTF("tran speed: %d kbps\r\n", decode_transpeed(csd->tranSpeed));
    PRINTF("ccc: class ");
    while (csd->ccc && i < 12)
    {
        if (csd->ccc & 0x1)
        {
            PRINTF("%d ", i);
        }
        i++;
        csd->ccc >>= 1;
    }
    PRINTF("\r\n");
    PRINTF("max read block length: %d Bytes\r\n", (uint32_t)1 << csd->readBlkLen);
    if (csd->flags & SD_CSD_READ_BL_PARTIAL)
        PRINTF("Support partial read\r\n");
    if (csd->flags & SD_CSD_WRITE_BLOCK_MISALIGN)
        PRINTF("Support crossing physical block boundaries writing is allowed\r\n");
    if (csd->flags & SD_CSD_READ_BLOCK_MISALIGN)
        PRINTF("Support crossing physical block boundaries reading is allowed\r\n");
    if (csd->flags & SD_CSD_DSR_IMP)
        PRINTF("DSR is implemented\r\n");
    PRINTF("c_size: %d\r\n", csd->cSize);
    if (csd->csdStructure == 0)
    {
        PRINTF("VDD_R_CURR_MIN: 0x%X\r\n", csd->vddRCurrMin);
        PRINTF("VDD_R_CURR_MAX: 0x%X\r\n", csd->vddRCurrMax);
        PRINTF("VDD_W_CURR_MIN: 0x%X\r\n", csd->vddWCurrMin);
        PRINTF("VDD_W_CURR_MAX: 0x%X\r\n", csd->vddWCurrMax);
        PRINTF("c_size_mult: %d\r\n", csd->cSizeMult);
    }
    if (csd->flags & SD_CSD_ERASE_BLOCK_ENABLED)
        PRINTF("Erase unit size is one or multiple units of 512 bytes\r\n");
    else
        PRINTF("Erase unit size is one or multiple units of %d blocks\r\n", csd->sectorSize + 2);

    PRINTF("The size of write protected group is %d blocks\r\n", csd->wpGrpSize + 2);
    if (csd->flags & SD_CSD_WP_GRP_ENABLED)
        PRINTF("Write protection group is defined\r\n");
    PRINTF("R2W_Factor: %d\r\n", csd->r2wFactor);
    PRINTF("max write block length: %d Bytes\r\n", (uint32_t)1 << csd->writeBlkLen);
    if (csd->flags & SD_CSD_WRITE_BL_PARTIAL)
        PRINTF("Smaller blocks can be used to write\r\n");
    if (csd->flags & SD_CSD_COPY)
        PRINTF("The content is copied\r\n");
    if (csd->flags & SD_CSD_PERM_WRITE_PROTECT)
        PRINTF("The content is write protected permanently\r\n");
    if (csd->flags & SD_CSD_TMP_WRITE_PROTECT)
        PRINTF("The content is write protected temporarily\r\n");
    if (!(csd->flags & SD_CSD_FILE_FORMAT_GROUP))
    {
        switch(csd->fileFormat)
        {
            case 0:
                PRINTF("Hard disk-like file system with partition table\r\n");
                break;
            case 1:
                PRINTF("DOS FAT (floppy-like) with boot sector only\r\n");
                break;
            case 2:
                PRINTF("Universla file format\r\n");
                break;
            case 3:
                PRINTF("Others/Unknown\r\n");
                break;
        }
    }
}

static void show_card_scr(sd_scr_t *scr)
{
    PRINTF("SCR Structure: 0x%X\r\n", scr->scrStructure);
    PRINTF("SD Spec: 0x%X\r\n", scr->sdSpec);
    if (scr->sdSpec)
    {
        if (scr->flags & SD_SCR_SD_SPEC3)
        {
            PRINTF("SD Spec 3.0\r\n");
        }
        else
        {
            PRINTF("SD Spec 2.0\r\n");
        }
    }
    if (scr->flags & SD_SCR_DATA_STAT_AFTER_ERASE)
        PRINTF("Data status after erase\r\n");
    switch(scr->sdSecurity)
    {
        case 0:
            PRINTF("No security\r\n");
            break;
        case 1:
            PRINTF("Not used\r\n");
            break;
        case 2:
            PRINTF("SDSC Card(Security Version 1.01)\r\n");
            break;
        case 3:
            PRINTF("SDHC Card(Security Version 2.00)\r\n");
            break;
        case 4:
            PRINTF("SDXC Card(Security Version 3.xx)\r\n");
            break;
    }
    if (scr->sdBusWidths & 0x1)
        PRINTF("Card supports 1-bit bus width\r\n");
    if (scr->sdBusWidths & 0x4)
        PRINTF("Card supports 4-bit bus width\r\n");
    if (scr->exSecurity)
        PRINTF("Extended security is supported\r\n");
    if (scr->exSecurity)
        PRINTF("Extended security is supported\r\n");
    if (scr->cmdSupport & 0x1)
        PRINTF("Support set block count command\r\n");
    if (scr->cmdSupport & 0x2)
        PRINTF("Support speed class control\r\n");
}

static void show_card_info(sd_t *card, bool showDetail)
{
    double temp;

    PRINTF("\r\n------- Card Information -------\r\n");

    if (card->caps & SD_CARD_CAPS_SDHC)
    {
        PRINTF("SDHC");
    }
    else if (card->caps & SD_CARD_CAPS_SDXC)
    {
        PRINTF("SDXC");
    }
    else
    {
        PRINTF("SDSC");
    }
    
    PRINTF("\r\nCard Capacity: ");
    temp = 1000000000 / card->blockSize;
    temp = ((float)card->blockCount / (float)temp);
    if (temp > 1.0)
    {
        PRINTF("%.02f GB\r\n", temp);
    }
    else
    {
        PRINTF("%.02f MB\r\n", temp * 1000);
    }
    PRINTF("Host Clock Max Rate: %d MHz\r\n", CLOCK_SYS_GetSdhcFreq(card->host->instance) / 1000000);
    //PRINTF("Clock Rate: %d MHz\r\n", card->host->targetClockFreq / 1000000);

    show_card_cid(&(card->cid));
    //if (IS_SD_CARD(card) && showDetail)
   // {
        show_card_csd(&(card->csd));
        show_card_scr(&(card->scr));
  //  }
}

bool test_card_detection(void)
{
    uint32_t loop_time = 10, status;
    sdmmc_status_t ret = kStatus_SDMMC_NoError;
    while (loop_time)
    {
        if (g_sdInsertedFlag && (!g_sdInitFlag))
        {
            PRINTF("A card is detected\n\r");
            ret = SD_IndentifyCard(&g_host, &g_sd);
            if (kStatus_SDMMC_NoError != ret)
            {
                if (ret == kStatus_SDMMC_NotSupportYet)
                {
                    PRINTF("Oops, this card is not supported yet\r\n");
                }
                else
                {
                    PRINTF("SD_IndentifyCard failed\r\n");
                }
                SDMMC_DeInitHost(&g_host);
                return false;
            }
            g_sdInitFlag = 1;

            //show_card_info(&g_sd, false);
            loop_time--;
            PRINTF("Card is initialized, %d times left\r\n", loop_time);
        }
        else if (!g_sdInsertedFlag && g_sdInitFlag)
        {
            SD_Shutdown(&g_sd);
            g_sdInitFlag = 0;
            PRINTF("Card is removed\n\r");
        }
#if defined CD_USING_GPIO
         waitCardDetectEvent(FSL_HOST_WAIT_FOREVER);
#else
        do
        {
            /* If card is removed, sets flag to 0. */
            if (g_sdInsertedFlag && (status == kStatus_SDMMC_NoCardInsertedError))
            {
                g_sdInsertedFlag = 0;
                break;
            }
            status = SD_DetectCard(&g_host);
            if (status == kStatus_SDMMC_CardDetectNotSupportYet)
                return false;
            delayTimeMsec(100);
        } while (status == kStatus_SDMMC_NoCardInsertedError);
        if (status == kStatus_SDMMC_NoError)
        {
            g_sdInsertedFlag = 1;
        }
#endif
    }
    return true;
}

bool test_data_access(void)
{
    SDHC_Type* base = &SDHC[HOST_INSTANCE];
    host_t *host = &g_host;
    sd_t *sd = &g_sd;
    uint32_t i, elapsed_ms, ms, current_ms, j;

    host->instance = HOST_INSTANCE;
    host->cardDetectMode = kSdmmcHostCardDetectGpio;
    //host->transferMode = kSdmmcHostTransModePio;
    //host->transferMode = kSdmmcHostTransModeAdma1;
    host->transferMode = kSdmmcHostTransModeAdma2;
    host->capability = &g_hostCapability;
    host->admaTableAddress = g_admaTable;
    host->admaTableMaxEntries = ADMA_TABLE_MAX_ENTRIES;
    host->createCmdEvent = createCmdEvent;
    host->waitCmdEvent = waitCmdEvent;
    host->notifyCmdEvent = notifyCmdEvent;
    host->deleteCmdEvent = deleteCmdEvent;
    host->createDataEvent = createDataEvent;
    host->waitDataEvent = waitDataEvent;
    host->notifyDataEvent = notifyDataEvent;
    host->deleteDataEvent = deleteDataEvent;
    host->markStartTimeMsec = markStartTimeMsec;
    host->getElapsedTimeMsec = getElapsedTimeMsec;
    host->delayTimeMsec = delayTimeMsec;

    SDMMC_InitHost(host);

//#if defined CD_USING_GPIO
    /* createCardDetectEvent(); */
//#endif
    if (kStatus_SDMMC_NoError != SD_IndentifyCard(host, sd))
    {
        return false;
    }

    if (SD_CheckReadOnly(sd))
    {
        PRINTF("Card is write-protected, skip writing tests\r\n");

        memset(g_refData, 0, sizeof(g_refData));
        if (kStatus_SDMMC_NoError != SD_ReadBlocks(sd, g_testData, 2, 1))
        {
            PRINTF("ERROR: SD_ReadBlocks failed, line %d\r\n", __LINE__);
            SD_Shutdown(sd);
            SDMMC_DeInitHost(host);
            return false;
        }
        PRINTF("Single block read test passed!\r\n");

        memset(g_refData, 0, sizeof(g_refData));
        if (kStatus_SDMMC_NoError != SD_ReadBlocks(sd, g_testData, TEST_START_BLOCK, sizeof(g_refData)/FSL_CARD_DEFAULT_BLOCK_SIZE))
        {
            PRINTF("ERROR: SD_ReadBlocks failed, line %d\r\n", __LINE__);
            SD_Shutdown(sd);
            SDMMC_DeInitHost(host);
            return false;
        }
        PRINTF("multiple block read test passed!\r\n");
        SD_Shutdown(sd);
        SDMMC_DeInitHost(host);
        return false;
    }

    if (kStatus_SDMMC_NoError != SD_EraseBlocks(sd, 0, 20))
    {
        PRINTF("ERROR:SD_EraseBlocks failed, line %d\r\n", __LINE__);
        SD_Shutdown(sd);
        SDMMC_DeInitHost(host);
        return false;
    }

    PRINTF("\r\nStart read/write test...\r\n");

    memset(g_refData, 0, sizeof(g_refData));
    if (fill_reference_data(g_refData, 0x11, sizeof(g_refData))) {
        PRINTF("ERROR: prepare reference data failed\r\n");
        SD_Shutdown(sd);
        SDMMC_DeInitHost(host);
        return false;
    }

        if (kStatus_SDMMC_NoError != SD_WriteBlocks(sd, g_refData, 2, 1))
    {
        PRINTF("ERROR: SD_WriteBlocks failed, line %d\r\n", __LINE__);
        SD_Shutdown(sd);
        SDMMC_DeInitHost(host);
        return false;
    }

    if (kStatus_SDMMC_NoError != SD_ReadBlocks(sd, g_testData, 2, 1))
    {
        PRINTF("ERROR: SD_ReadBlocks failed, line %d\r\n", __LINE__);
        SD_Shutdown(sd);
        SDMMC_DeInitHost(host);
        return false;
    }

    if (memcmp(g_testData, g_refData, FSL_CARD_DEFAULT_BLOCK_SIZE))
    {
        PRINTF("ERROR: data comparison failed, line %d\r\n", __LINE__);
        SD_Shutdown(sd);
        SDMMC_DeInitHost(host);
        return false;
    }
    PRINTF("Single block read/write test passed!\r\n");

    memset(g_refData, 0, sizeof(g_refData));
    for (i = 0; i < sizeof(g_refData)/FSL_CARD_DEFAULT_BLOCK_SIZE; i++)
    {
        if (fill_reference_data(&g_refData[i * FSL_CARD_DEFAULT_BLOCK_SIZE], 0x44 + i, FSL_CARD_DEFAULT_BLOCK_SIZE))
        {
            PRINTF("ERROR: fill data failed, line %d\r\n", __LINE__);
            SD_Shutdown(sd);
            SDMMC_DeInitHost(host);
            return false;
        }
    }

    elapsed_ms = 0;
    i = LOOPS;
    while(i--)
    {
        //ms = OSA_TimeGetMsec();
        host->markStartTimeMsec();
        if (kStatus_SDMMC_NoError != SD_WriteBlocks(sd, g_refData, TEST_START_BLOCK, sizeof(g_refData)/FSL_CARD_DEFAULT_BLOCK_SIZE))
        {
            PRINTF("ERROR: SD_WriteBlocks failed, line %d\r\n", __LINE__);
            SD_Shutdown(sd);
            SDMMC_DeInitHost(host);
            return false;
        }
        /*current_ms = OSA_TimeGetMsec();
        if (current_ms < ms)
        {
            current_ms += 60000;
        }

        elapsed_ms += current_ms - ms;*/
        current_ms = host->getElapsedTimeMsec();
        elapsed_ms += current_ms;
    }
    PRINTF("Writing %lu bytes for %u times in %u ms, at %u kB/s\r\n", sizeof(g_refData), LOOPS, elapsed_ms, sizeof(g_refData) * LOOPS / elapsed_ms);

    elapsed_ms = 0;
    i = LOOPS;
    while(i--)
    {
        host->markStartTimeMsec();
        if (kStatus_SDMMC_NoError != SD_ReadBlocks(sd, g_testData, TEST_START_BLOCK, sizeof(g_testData)/FSL_CARD_DEFAULT_BLOCK_SIZE))
        {
            PRINTF("ERROR: SD_ReadBlocks failed, line %d\r\n", __LINE__);
            SD_Shutdown(sd);
            SDMMC_DeInitHost(host);
            return false;
        }
        /*current_ms = OSA_TimeGetMsec();
        if (current_ms < ms)
        {
            current_ms += 60000;
        }

        elapsed_ms += current_ms - ms;*/
        current_ms = host->getElapsedTimeMsec();
        elapsed_ms += current_ms;
    }
    PRINTF("Reading %lu bytes for %u times in %u ms, at %u kB/s\r\n", sizeof(g_testData), LOOPS, elapsed_ms, sizeof(g_testData) * LOOPS / elapsed_ms);

       j = LOOPS;
    while(j--)
    {
        memset(g_testData, 0, sizeof(g_testData));
        if (kStatus_SDMMC_NoError != SD_ReadBlocks(sd, g_testData, TEST_START_BLOCK, sizeof(g_testData)/FSL_CARD_DEFAULT_BLOCK_SIZE))
        {
            PRINTF("ERROR: SD_ReadBlocks failed, line %d\r\n", __LINE__);
            SD_Shutdown(sd);
            SDMMC_DeInitHost(host);
            return false;
        }

        if (memcmp(g_testData, g_refData, sizeof(g_testData)))
        {
            PRINTF("ERROR: data comparison failed, line %d\r\n", __LINE__);
            SD_Shutdown(sd);
            SDMMC_DeInitHost(host);
            return false;
        }
    }
    PRINTF("Multi-block read/write test passed!\r\n");

    if (kStatus_SDMMC_NoError != SD_EraseBlocks(sd, TEST_START_BLOCK, 1))
    {
        PRINTF("ERROR: SD_EraseBlocks failed, line %d\r\n", __LINE__);
        SD_Shutdown(sd);
        SDMMC_DeInitHost(host);
        return false;
    }
    PRINTF("Erase blocks test passed!\r\n");

    SD_Shutdown(sd);
    SDMMC_DeInitHost(host);
    while (1);
    while(1)
    {
        //CLOCK_SYS_EnableSdhcClock(0);
        //
        memset(g_refData, 1, sizeof(g_refData));
        memset(g_testData, 1, sizeof(g_testData));
        
        if (kStatus_SDMMC_NoError != SD_WriteBlocks(sd, g_testData, 2, 1))
        {
            break;
        }
        if (kStatus_SDMMC_NoError != SD_ReadBlocks(sd, g_testData, 2, 1))
        {
            break;
        }
        if (memcmp(g_testData, g_refData, FSL_CARD_DEFAULT_BLOCK_SIZE))
        {
            break;
        }
        if (kStatus_SDMMC_NoError != SD_WriteBlocks(sd, g_testData, 2, TEST_BLOCK_NUM))
        {
            break;
        }
        if (kStatus_SDMMC_NoError != SD_ReadBlocks(sd, g_testData, 2, TEST_BLOCK_NUM))
        {
            break;
        }
        if (memcmp(g_testData, g_refData, FSL_CARD_DEFAULT_BLOCK_SIZE * TEST_BLOCK_NUM))
        {
            break;
        }

        //CLOCK_SYS_DisableSdhcClock(0);
         delayTimeMsec(100);
    }
    
    
    while(1);
    if (!test_card_detection())
    {
        deleteCardDetectEvent();
        return false;
    }
//#if defined CD_USING_GPIO
    deleteCardDetectEvent();
//#endif
    return true;
}