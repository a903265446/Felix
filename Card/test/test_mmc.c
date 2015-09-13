
#include "test_sd.h"
#include "card.h"



/************************************test normal case ********************************************/

static bool test_MMC_Init(void)
{
    return true;
}

static bool test_MMC_DeInit(void)
{
    return true;
}

static bool test_MMC_CheckReadOnly(void)
{
    return true;
}

/*
 Test following 7 situations:
 1.Test read/write/erase card when transfer type is polling based DATAPORT, 
   no clocks auto gated off applied, auto CMD12 is not enabled.
 2.Test read/write/erase card when transfer type is polling based ADMA1, 
   no clocks auto gated off applied, auto CMD12 is not enabled.
 3.Test read/write/erase card when transfer type is polling based ADMA2, 
   no clocks auto gated off applied, auto CMD12 is not enabled.
 4.Test read/write/erase card when transfer type is interrupt based DATAPORT ,  
   no clocks auto gated off applied, auto CMD12 is not enabled.
 5.Test read/write/erase card when transfer type is interrupt based ADMA1 ,  
   no clocks auto gated off applied, auto CMD12 is not enabled.
 6.Test read/write/erase card when transfer type is interrupt based ADMA2 ,  
   no clocks auto gated off applied, auto CMD12 is not enabled. 
 7.Test read/write/erase card when transfer type is interrupt based ADMA2 , 
   all clocks auto gated off applied, auto CMD12 is enabled. 
 8.Test read/write card when transfer type is interrupt based ADMA2 ,no clocks auto gated off applied, 
   auto CMD12 is enabled, pre-define block count feature is enabled 
 */
static bool test_read_write_erase_condition1()
{
    return true;
}

static bool test_read_write_erase_condition2()
{
    return true;
}

static bool test_read_write_erase_condition3()
{
    return true;
}

static bool test_read_write_erase_condition4()
{
    return true;
}

static bool test_read_write_erase_condition5()
{
    return true;
}

static bool test_read_write_erase_condition6()
{
    return true;
}

static bool test_read_write_erase_condition7()
{
    return true;
}

static bool test_read_write_erase_condition8()
{
    return true;
}

/*
Mainly test following function:
•	Init, DeInit
•	Read/write/erase when transfer type is polling/interrupt based DATAPORT/ADMA1/ADMA2 
•	Read/write/erase when clocks auto gated off is enabled or disabled
•	Read/write/erase when auto CMD12 is enabled or disabled
•	Read/write/erase when pre-define block count feature is enabled or disabled in 
        MMC card(EMMC card has two data transfer way: CMD12 is used in open-ended transmission. 
        Pre-defined block count is used in exact block count transmission).
 */
void test_mmc_normal_case(void)
{
    if (!test_MMC_Init() || !test_MMC_DeInit()|| !test_MMC_CheckReadOnly())
    {
        return;
    } 
  
    if (!test_read_write_erase_condition1() || !test_read_write_erase_condition2() 
        || !test_read_write_erase_condition3() || !test_read_write_erase_condition4()
        || !test_read_write_erase_condition5() || !test_read_write_erase_condition6()
        || !test_read_write_erase_condition7())
    {
        return;
    }
}


/************************************test exception case *****************************************/
static void test_param_null(void)
{
  
}

static mmc_card_t g_card;
static sdhc_host_t g_host;
static void test_function_calling_disorder(void)
{
    uint8_t buffer[512];

    MMC_Init(&g_host, &g_card);
    MMC_DeInit(&g_card);

    MMC_ReadBlocks(&g_card, buffer, 0, 1);

    /* Check error prompt */
}

static void test_address_out_of_range(void)
{
    uint8_t buffer[512];

    MMC_Init(&g_host, &g_card);
    MMC_ReadBlocks(&g_card, buffer, 0, 1000000000U);
}

/*
Mainly test following cases:
•	Function parameter is NULL.
•	Error’s prompt when the calling order of function is error.
•	Error’s prompt when the access range is out of max address of the card.
 */
void test_mmc_exception_case(void)
{
    test_param_null();
    test_function_calling_disorder();
    test_address_out_of_range();
}


/************************************test performance ****************************************/
/*
Test the minimum performance of the system by using following test condition combination:
•	Polling based DATAPORT, auto gated off applied, auto CMD12 is not enabled, 
        pre-define block count feature is not enabled
•	Polling based ADMA2, auto gated off applied, auto CMD12 is not enabled, pre-define block 
        count feature is not enabled
Test the maximum performance of the system by using following test condition combination:
•	Interrupt based DATAPORT, no auto gated off applied, auto CMD12 is enabled, pre-define block 
        count feature is not enabled.
•	Interrupt based ADMA2, no auto gated off applied, auto CMD12 is enabled, pre-define block 
        count feature is not enabled.
(No need to test ADMA1 transfer type because of that this transfer type will not satisfy the address 
check condition and will switch to DATAPORT transfer type automatically) 
 
 */
void test_mmc_performance_case(void)
{

}

/************************************stress test****************************************/
/*
Mainly test following situation:
•	Test card (class 2/4/6/10) read/write can run 24 hours correctly when all 
        SDHC feature is disabled and data size is big (make data throughput high).
•	Test card (class 2/4/6/10) read/write can run 24 hours correctly when all 
        SDHC feature is enabled and data size is big.
•	Test card (class 2/4/6/10) read/write can run 24 hours correctly when all 
        SDHC feature is disabled and data size is small (make card access frequency high).
•	Test card (class 2/4/6/10) read/write can run 24 hours correctly when all 
        SDHC feature is enabled and data size is small.
 */
void test_mmc_stress_case(void)
{
    
}

int main(void)
{
    test_mmc_normal_case();
    test_mmc_exception_case();
    test_mmc_performance_case();
    test_mmc_stress_case();
    
    return 0;
}