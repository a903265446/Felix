
#include "test_sd.h"
#include "card.h"



/************************************test normal case ********************************************/

static bool test_SD_Init(void)
{
    return true;
}

static bool test_SD_DeInit(void)
{
    return true;
}

static bool test_SD_CheckReadOnly(void)
{
    return true;
}

/*
 Test following 7 situations:
 1.Test read/write/erase card(class 2/4/6/10) when transfer type is polling based DATAPORT, 
   no clocks auto gated off applied, auto CMD12 is not enabled.
 2.Test read/write/erase card(class 2/4/6/10) when transfer type is polling based ADMA1, 
   no clocks auto gated off applied, auto CMD12 is not enabled.
 3.Test read/write/erase card(class 2/4/6/10) when transfer type is polling based ADMA2, 
   no clocks auto gated off applied, auto CMD12 is not enabled.
 4.Test read/write/erase card(class 2/4/6/10) when transfer type is interrupt based DATAPORT ,  
   no clocks auto gated off applied, auto CMD12 is not enabled.
 5.Test read/write/erase card(class 2/4/6/10) when transfer type is interrupt based ADMA1 ,  
   no clocks auto gated off applied, auto CMD12 is not enabled.
 6.Test read/write/erase card(class 2/4/6/10) when transfer type is interrupt based ADMA2 ,  
   no clocks auto gated off applied, auto CMD12 is not enabled. 
 7.Test read/write/erase card(class 2/4/6/10) when transfer type is interrupt based ADMA2 , 
   all clocks auto gated off applied, auto CMD12 is enabled. 
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
/*
Mainly test following cases:
•	Init, DeInit
•	Read/write/erase when transfer type is polling/interrupt based DATAPORT/ADMA1/ADMA2 
•	Read/write/erase when clocks auto gated off is enabled or disabled
•	Read/write/erase when auto CMD12 is enabled or disabled
 */
void test_sd_normal_case(void)
{
    if (!test_SD_Init() || !test_SD_DeInit()|| !test_SD_CheckReadOnly())
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

static sd_card_t g_card;
static sdhc_host_t g_host;
void test_function_calling_disorder(void)
{
    uint8_t buffer[512];

    SD_Init(&g_host, &g_card);
    SD_DeInit(&g_card);

    SD_ReadBlocks(&g_card, buffer, 0, 1);

    /* Check error prompt */
}

void test_address_out_of_range(void)
{
    uint8_t buffer[512];

    SD_Init(&g_host, &g_card);
    SD_ReadBlocks(&g_card, buffer, 0, 1000000000U);
}

/*
Mainly test following case:
•	Function parameter is NULL.
•	Error’s prompt when the calling order of function is error.
•	Error’s prompt when the access range is out of max address of the card.
•	Error’s prompt when reading/writing card and suddenly remove the card
        (Card remove action can be simulated by using card detect test level bit in the PROCTL register).

 */
void test_sd_exception_case(void)
{
    test_param_null();
    test_function_calling_disorder();
    test_address_out_of_range();
}


/************************************test performance****************************************/
/*
Test the minimum performance of the system by using following test condition combination:
•	Polling based DATAPORT, auto gated off applied, auto CMD12 is not enabled
•	Polling based ADMA2, auto gated off applied, auto CMD12 is not enabled
Test the maximum performance of the system by using following test condition combination:
•	Interrupt based DATAPORT, no auto gated off applied, auto CMD12 is enabled
•	Interrupt based ADMA2, no auto gated off applied, auto CMD12 is enabled
(No need to test ADMA1 transfer type because of that transfer type will not satisfy 
the address check condition and will switch to DATAPORT transfer type automatically in the driver) 

 */
void test_sd_performance_case(void)
{

}

/*
Mainly test following cases:
•	Test card (class 2/4/6/10) read/write can run 24 hours correctly when all 
        SDHC feature is disabled and data size is big (make data throughput high).
•	Test card (class 2/4/6/10) read/write can run 24 hours correctly when all 
        SDHC feature is enabled and data size is big.
•	Test card (class 2/4/6/10) read/write can run 24 hours correctly when all 
        SDHC feature is disabled and data size is small (make card access frequency high).
•	Test card (class 2/4/6/10) read/write can run 24 hours correctly when all 
        SDHC feature is enabled and data size is small.
 */
/************************************stress test****************************************/
void test_sd_stress_case(void)
{
    
}

int main(void)
{
    test_sd_normal_case();
    test_sd_exception_case();
    test_sd_performance_case();
    test_sd_stress_case();
    
    return 0;
}