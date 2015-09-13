

#include "fsl_clock_manager.h"
#include "fsl_debug_console.h"
#include "fsl_lptmr_hal.h"
#include "test_sdspi_callback.h"
#include "test_sdspi_function.h"
#include "board.h"




#define LPTMR_COUNTER_RANGE (65535U)

static volatile uint32_t g_cardDetectEvent = 0;


uint32_t get_current_time_msec()
{
    LPTMR_Type *base = LPTMR0;
    return LPTMR_HAL_GetCounterValue(base);
}

uint32_t get_time_range_msec()
{
    return LPTMR_COUNTER_RANGE;
}

bool create_card_detect_event()
{
    g_cardDetectEvent = 0;
    return true;
}

bool wait_card_detect_event(uint32_t timeout)
{
    while (g_cardDetectEvent == 0) {};
    g_cardDetectEvent = 0;
    return true;
}

bool notify_card_detect_event()
{
    g_cardDetectEvent = 1;
    return true;
}

bool delete_card_detect_event()
{
    g_cardDetectEvent = 0;
    return true;
}

void init_lptmr(void)
{
    /* Set LPTMR to 1HZ clock source, freeRunning mode. */
    /* Enable LPTMR */

    lptmr_working_mode_user_config_t working_mode_config;
    lptmr_prescaler_user_config_t prescaler_config;
    LPTMR_Type *base = LPTMR0;

    CLOCK_SYS_EnableLptmrClock(0);
    LPTMR_HAL_Disable(base);
    LPTMR_HAL_ClearIntFlag(base);

    working_mode_config.timerModeSelect = kLptmrTimerModeTimeCounter;
    working_mode_config.freeRunningEnable = true;
    working_mode_config.pinPolarity = kLptmrPinPolarityActiveHigh;
    working_mode_config.pinSelect = kLptmrPinSelectInput0;
    LPTMR_HAL_SetTimerWorkingMode(base, working_mode_config);

    prescaler_config.prescalerValue = kLptmrPrescalerDivide2;
    prescaler_config.prescalerBypass = true;
    prescaler_config.prescalerClockSelect = kLptmrPrescalerClock1;
    LPTMR_HAL_SetPrescalerMode(base, prescaler_config);
    LPTMR_HAL_Enable(base);
}


//int main(void)
//{
//    
//    init_sdspi_hardware();
//    OSA_Init();
//
//    PRINTF("SPI SD Card Demo Start!\r\n\r\n");
//    PRINTF("\r\n");
//
//    test_sdspi();
//    
//    return 0;
//}

void hard_fault_handler_c(unsigned int * hardfault_args)
{
	static unsigned int stacked_r0;
	static unsigned int stacked_r1;
	static unsigned int stacked_r2;
	static unsigned int stacked_r3;
	static unsigned int stacked_r12;
	static unsigned int stacked_lr;
	static unsigned int stacked_pc;
	static unsigned int stacked_psr;
	static unsigned int SHCSR;
	static unsigned char MFSR;
	static unsigned char BFSR;	
	static unsigned short int UFSR;
	static unsigned int HFSR;
	static unsigned int DFSR;
	static unsigned int MMAR;
	static unsigned int BFAR;

	stacked_r0 = ((unsigned long) hardfault_args[0]);
	stacked_r1 = ((unsigned long) hardfault_args[1]);
	stacked_r2 = ((unsigned long) hardfault_args[2]);
	stacked_r3 = ((unsigned long) hardfault_args[3]);
	stacked_r12 = ((unsigned long) hardfault_args[4]);
	/*异常中断发生时，这个异常模式特定的物理R14,即lr被设置成该异常模式将要返回的地址*/
	stacked_lr = ((unsigned long) hardfault_args[5]); 	
	stacked_pc = ((unsigned long) hardfault_args[6]);
	stacked_psr = ((unsigned long) hardfault_args[7]);

	SHCSR = (*((volatile unsigned long *)(0xE000ED24))); //系统Handler控制及状态寄存器
	MFSR = (*((volatile unsigned char *)(0xE000ED28)));	//存储器管理fault状态寄存器	
	BFSR = (*((volatile unsigned char *)(0xE000ED29)));	//总线fault状态寄存器	
	UFSR = (*((volatile unsigned short int *)(0xE000ED2A)));//用法fault状态寄存器		
	HFSR = (*((volatile unsigned long *)(0xE000ED2C)));  //硬fault状态寄存器			
	DFSR = (*((volatile unsigned long *)(0xE000ED30)));	//调试fault状态寄存器
	MMAR = (*((volatile unsigned long *)(0xE000ED34)));	//存储管理地址寄存器
	BFAR = (*((volatile unsigned long *)(0xE000ED38))); //总线fault地址寄存器
	while (1);
} 
