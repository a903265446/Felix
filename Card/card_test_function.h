#include <stdbool.h>
#include "card.h"

// Test function declare
void init_hardware(void);
bool test_card_detection(void);
bool test_data_access(void);

bool createCmdEvent();
bool waitCmdEvent(uint32_t timeout); 
bool notifyCmdEvent();
bool deleteCmdEvent();
bool createDataEvent(); 
bool waitDataEvent(uint32_t timeout);
bool notifyDataEvent();
bool deleteDataEvent();
void markStartTimeMsec();
uint32_t getElapsedTimeMsec();
void delayTimeMsec();
bool createCardDetectEvent();
bool waitCardDetectEvent(uint32_t timeout);
bool notifyCardDetectEvent();
bool deleteCardDetectEvent();