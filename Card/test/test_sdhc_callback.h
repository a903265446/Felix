#include <stdbool.h>

// Test callback function declare

bool createCmdEvent();
bool waitCmdEvent(uint32_t timeout); 
bool notifyCmdEvent();
bool deleteCmdEvent();
bool createDataEvent(); 
bool waitDataEvent(uint32_t timeout);
bool notifyDataEvent();
bool deleteDataEvent();

/* Maybe better if only provide the get current time interface */
uint32_t getCurrentTimeMsec();
uint32_t getTimeRangeMsec();
void delayTimeMsec(uint32_t msec);


bool createCardDetectEvent();
bool waitCardDetectEvent(uint32_t timeout);
bool notifyCardDetectEvent();
bool deleteCardDetectEvent();


