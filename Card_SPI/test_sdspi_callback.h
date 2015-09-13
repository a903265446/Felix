

#ifndef __SDSPI_CALLBACK_BM_H__
#define __SDSPI_CALLBACK_BM_H__

#include <stdint.h>
#include <stdbool.h>

bool create_card_detect_event();
bool wait_card_detect_event(uint32_t timeout);
bool notify_card_detect_event();
bool delete_card_detect_event();

uint32_t get_current_time_msec();
uint32_t get_time_range_msec();



#endif