

#include <string.h>

#include "sdspi.h"
#include "test_sdspi.h"


#define TEST_START_BLOCK 1
#define TEST_BLOCK_COUNT 5
#define TEST_BLOCK_SIZE FSL_CARD_DEFAULT_BLOCK_SIZE
sdspi_card_t g_sdspi;
uint8_t g_testSendBuffer[TEST_BLOCK_COUNT*TEST_BLOCK_SIZE];
uint8_t g_testReceiveBuffer[TEST_BLOCK_COUNT*TEST_BLOCK_SIZE];

/************************************test normal case ********************************************/


static bool test_SDSPI_Init(void)
{
    return true;
}

static bool test_SDSPI_DeInit(void)
{
    return true;
}

static bool test_SDSPI_CheckReadOnly(void)
{
    return true;
}


/*
Mainly test following cases:
•   Init/DeInit
•   Read only check
•   Read/write/erase
 */
void test_sdspi_normal_case(void)
{
    if (!test_SDSPI_Init() || !test_SDSPI_DeInit()|| !test_SDSPI_CheckReadOnly())
    {
        return;
    }
    
    memset(g_testSendBuffer, 0x19, sizeof(g_testSendBuffer));
    memset(g_testReceiveBuffer, 0, sizeof(g_testSendBuffer));
    
    SDSPI_ReadBlocks(&g_sdspi, g_testSendBuffer, TEST_START_BLOCK, TEST_BLOCK_COUNT);
    
    SDSPI_WriteBlocks(&g_sdspi, g_testReceiveBuffer, TEST_START_BLOCK, TEST_BLOCK_COUNT);
    
    if (!memcmp(g_testSendBuffer, g_testReceiveBuffer, sizeof(g_testSendBuffer)))
    {
        return;
    }
}


/*******************************Test exception case***********************************************/
/*
Mainly test following cases:
•   Function parameter is NULL.
•   Error’s prompt when the calling order of function is error.
•   Error’s prompt when the access range is out of max address of the card.
•   Error’s prompt when reading/writing card and suddenly remove the card
 */
void test_sdspi_exception_case(void)
{

}


/**********************************Test performance***********************************************/
/*
Mainly test following cases:
•   Read/write/erase speed
 */
void test_sdspi_performance_case(void)
{

}


/**********************************Sress test***********************************************/
/*
Mainly test following cases:
•   Test card (class 2/4/6/10) read/write can run 24 hours correctly when data size is big 
    (make data throughput high).
•   Test card (class 2/4/6/10) read/write can run 24 hours correctly when data size is small
    (make card access frequency high).
 */
void test_sdspi_stress_case(void)
{

}

//int main(void)
//{
//    test_sdspi_normal_case();
//    test_sdspi_exception_case();
//    test_sdspi_performance_case();
//    test_sdspi_stress_case();
//    
//    return 0;
//}