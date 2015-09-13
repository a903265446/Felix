
#include <string.h>

#include "board.h"


#include "test_sdspi_function.h"
#include "test_sdspi_callback.h"
#include "fsl_debug_console.h"

#define TEST_BLOCK_COUNT 3
#define TEST_BLOCK_SIZE  512
#define TEST_START_BLOCK 2

static sdspi_host_t g_host = {0};
static sdspi_card_t g_card = {0};
static dspi_master_state_t g_dspiState;
static dspi_device_t g_dspiDevice = {0};

static uint8_t g_testSendBuffer[TEST_BLOCK_COUNT * TEST_BLOCK_SIZE] = {0};
static uint8_t g_testRecvBuffer[TEST_BLOCK_COUNT * TEST_BLOCK_SIZE] = {0};

static bool g_card_inserted;

status_t setSpiFrequency(SPI_Type *base, uint32_t frequency)
{
    uint32_t calculatedBaudRate;
    calculatedBaudRate = DSPI_HAL_SetBaudRate(base,
            ((dspi_master_state_t *)(g_host.spiState))->whichCtar,
            frequency,
            ((dspi_master_state_t *)(g_host.spiState))->dspiSourceClock);
    if (0 == calculatedBaudRate)
    {
        return 1;
    }
    PRINTF("BaudRate set to %dHz\r\n", calculatedBaudRate);
    return 0;
}



status_t spiExchange(SPI_Type *base, const uint8_t *in, uint8_t *out, uint32_t size)
{
    if (kStatus_DSPI_Success == DSPI_DRV_MasterTransferBlocking(SDSPI_INSTANCE,
                  NULL, in, out, size, FSL_SDSPI_TIMEOUT))
    {
        return 0;
    }
    return 1;
}

void card_detect_irq_handler(void)
{
#if defined FRDM_K22F 
    if (GPIO_HAL_ReadPinInput(g_gpioBase[SDCARD_CD_PORT], SDCARD_CD_PIN))
    {
        g_card_inserted = true;
    }
    else
    {
        g_card_inserted = false;
    }
#endif
    
#if defined TWR_K22F120M 
    if (GPIO_HAL_ReadPinInput(g_gpioBase[SDCARD_CD_PORT], SDCARD_CD_PIN))
    {
        g_card_inserted = false;
    }
    else
    {
        g_card_inserted = true;
    }
    
#endif
    notify_card_detect_event();
    
}

void test_sdspi(void)
{
    dspi_master_user_config_t dspiConfig;
    uint32_t calculatedBaudRate;
    sdspi_card_t *card;
    
    uint8_t in[4] = {1, 2, 3, 4};
    uint8_t out[4];
    
    /* Wait until card is inserted */
    create_card_detect_event();
    
    card_detect_irq_handler();
    
    if (!g_card_inserted)
    {
        do 
        {
            wait_card_detect_event(500);
        }while(!g_card_inserted);
    }
       
    /* Init SPI driver */
    memset(&dspiConfig, 0, sizeof(dspiConfig));
    
    /* Dspi none DMA mode Init*/
    dspiConfig.isChipSelectContinuous = true;
    dspiConfig.isSckContinuous = false;
    dspiConfig.pcsPolarity = kDspiPcs_ActiveLow;
    dspiConfig.whichCtar = kDspiCtar0;
    dspiConfig.whichPcs = kDspiPcs0;

    DSPI_DRV_MasterInit(SDSPI_INSTANCE, &g_dspiState, &dspiConfig);

    g_dspiDevice.dataBusConfig.bitsPerFrame = 8;
    g_dspiDevice.dataBusConfig.clkPhase = kDspiClockPhase_FirstEdge;
    g_dspiDevice.dataBusConfig.clkPolarity = kDspiClockPolarity_ActiveHigh;
    g_dspiDevice.dataBusConfig.direction = kDspiMsbFirst;
    //g_dspiDevice.bitsPerSec = 1000000;
    g_dspiDevice.bitsPerSec = 400000;
    DSPI_DRV_MasterConfigureBus(SDSPI_INSTANCE, &g_dspiDevice, &calculatedBaudRate);

    /* Test SPI driver */
    DSPI_DRV_MasterTransferBlocking(SDSPI_INSTANCE, NULL, in, out, 4, FSL_SDSPI_TIMEOUT);
    DSPI_DRV_MasterTransferBlocking(SDSPI_INSTANCE, NULL, in, out, 2, FSL_SDSPI_TIMEOUT);
    DSPI_DRV_MasterTransferBlocking(SDSPI_INSTANCE, NULL, in, out, 2, FSL_SDSPI_TIMEOUT);
    DSPI_DRV_MasterTransferBlocking(SDSPI_INSTANCE, NULL, in, out, 1, FSL_SDSPI_TIMEOUT);
    DSPI_DRV_MasterTransferBlocking(SDSPI_INSTANCE, NULL, in, out, 2, FSL_SDSPI_TIMEOUT);
    DSPI_DRV_MasterTransferBlocking(SDSPI_INSTANCE, NULL, in, out, 2, FSL_SDSPI_TIMEOUT);
    DSPI_DRV_MasterTransferBlocking(SDSPI_INSTANCE, NULL, in, out, 1, FSL_SDSPI_TIMEOUT);
    DSPI_DRV_MasterTransferBlocking(SDSPI_INSTANCE, NULL, in, out, 1, FSL_SDSPI_TIMEOUT);
    DSPI_DRV_MasterTransferBlocking(SDSPI_INSTANCE, NULL, in, out, 2, FSL_SDSPI_TIMEOUT);
    //while(1);
      
    /* Init SDSPI driver */
#if defined FRDM_K22F
    g_host.base = SPI0;
#endif
#if defined TWR_K22F120M
    g_host.base = SPI1;
#endif
    g_host.setFrequency = setSpiFrequency;
    g_host.exchange = spiExchange;
    g_host.busBaudRate = calculatedBaudRate;
    g_host.getCurrentTimeMsec = get_current_time_msec;
    g_host.timeRangeMsec = get_time_range_msec();
    g_host.spiState = &g_dspiState;
    g_host.spiDevice = &g_dspiDevice;

    g_card.host = &g_host;
    card = &g_card;
    
    /* Initializes card. */
    if (kStatus_Success != SDSPI_Init(card))
    {
        PRINTF("SDSPI_Init failed\r\n");
        DSPI_DRV_MasterDeinit(SDSPI_INSTANCE);
#if defined SPI_USING_DMA
        EDMA_DRV_Deinit();
#endif
        return;
    }

    /* Read/Write card */
    memset(g_testSendBuffer, 0x17, sizeof(g_testSendBuffer));
    
    while (1)
    {
        memset(g_testRecvBuffer, 0, sizeof(g_testRecvBuffer));
        
        SDSPI_WriteBlocks(card, g_testSendBuffer, TEST_START_BLOCK, TEST_BLOCK_COUNT);
        
        SDSPI_ReadBlocks(card, g_testRecvBuffer, TEST_START_BLOCK, TEST_BLOCK_COUNT);
        
        if (memcmp(g_testRecvBuffer, g_testRecvBuffer, sizeof(g_testSendBuffer)))
        {
            break;
        }
    }
    
    delete_card_detect_event();
    
    while(1);
}
