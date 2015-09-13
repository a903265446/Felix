

#include "ksdk_common.h"

void SDMMC_DelayTimeMsec(uint32_t msec);

status_t SDMMC_SelectCard(sdhc_host_t *host, uint32_t rca, bool isSelected);
status_t SDMMC_SendStatus(sdhc_host_t *host, uint32_t rca);
status_t SDMMC_SendApplicationCmd(sdhc_host_t *host, uint32_t rca);
status_t SDMMC_SetBlockCount(sdhc_host_t *host, uint32_t blockCount);
status_t SDMMC_GoIdle(sdhc_host_t *host);
status_t SDMMC_StopTransmission(sdhc_host_t *host);
status_t SDMMC_SetBlockSize(sdhc_host_t *host, uint32_t blockSize);
status_t SDMMC_SetSdBusFreq(sdhc_host_t *host, uint32_t busFreq);
