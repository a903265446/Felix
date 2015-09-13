

#ifndef __SDSPI_TEST_FUNCTION_H__
#define __SDSPI_TEST_FUNCTION_H__

#include "sdspi.h"

#if defined FRDM_K22F
#define SDCARD_CD_PORT     GPIOB_IDX
#define SDCARD_CD_PIN      16

#define SDSPI_INSTANCE   0
#endif

#if defined TWR_K22F120M
#define SDCARD_CD_PORT     GPIOC_IDX
#define SDCARD_CD_PIN      5

#define SDSPI_INSTANCE   1

#endif

void init_sdspi_hardware(void);

void card_detect_irq_handler(void);

void test_sdspi(void);



#endif
