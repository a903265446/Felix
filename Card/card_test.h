// task prio
#define TASK_TEST_SDHC_CARD_PRIO         6U
// task size
#define TASK_TEST_SDHC_CARD_STACK_SIZE   1024U

// task declare
extern void task_test_sdhc_card(task_param_t param);
// task define
OSA_TASK_DEFINE(task_test_sdhc_card, TASK_TEST_SDHC_CARD_STACK_SIZE);
