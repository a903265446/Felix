

#ifndef __MMC_DISK_H__
#define __MMC_DISK_H__


#include <stdint.h>
#include "diskio.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif

#define CD_USING_GPIO 
  
/*************************************************************************************************
 * API
 ************************************************************************************************/
DSTATUS mmc_disk_initialize(uint8_t pdrv);
DSTATUS mmc_disk_status(uint8_t pdrv);
DRESULT mmc_disk_read(uint8_t pdrv, uint8_t *buf, uint32_t sector, uint8_t count);
DRESULT mmc_disk_write(uint8_t pdrv, const uint8_t *buf, uint32_t sector, uint8_t count);
DRESULT mmc_disk_ioctl(uint8_t pdrv, uint8_t cmd, void *buff);

#if defined(__cplusplus)
}
#endif


#endif /* MMC_DISK_H */
