#ifndef __TEST_SDHC_BOARD_H__
#define __TEST_SDHC_BOARD_H__

#include "fsl_device_registers.h"

void detect_card_inserted(void);
void detect_card_removed(void);
void init_sdhc_hardware(void);

#endif /* __TEST_SDHC_BOARD_H__ */

