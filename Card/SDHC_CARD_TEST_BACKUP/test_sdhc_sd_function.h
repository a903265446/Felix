#include <stdbool.h>
//#include "card.h"

// Test function declare
void init_hardware(void);
bool test_sd();

bool createCmdEvent();
bool waitCmdEvent(uint32_t timeout); 
bool notifyCmdEvent();
bool deleteCmdEvent();
bool createDataEvent(); 
bool waitDataEvent(uint32_t timeout);
bool notifyDataEvent();
bool deleteDataEvent();

/* Maybe better if only provide the get current time interface */
uint32_t getCurrentTime();
uint32_t getRangeTime();
void delayTimeMsec(uint32_t msec);


bool createCardDetectEvent();
bool waitCardDetectEvent(uint32_t timeout);
bool notifyCardDetectEvent();
bool deleteCardDetectEvent();
