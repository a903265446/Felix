

#include <stdint.h>
#include "test_mmc_fatfs.h"


/********************************************normal test case **********************************/
/* Use a shell to test file system function i.e disk mount/file create/file open/file read/file write
  /file opendir/ file readdir/file mkdir/file getcwd etc. */
#define INIT_DISK 0
#define GET_DISK_STATUS 1
#define READ_DISK 2
#define WRITE_DISK 3
#define DISK_IOCTL 4
void test_mmc_fatfs_normal_case(void)
{
    uint32_t instruction = 0;
    /* instruction is input by user */

    switch (instruction)
    {
        case INIT_DISK:
            break;
        case GET_DISK_STATUS:
            break;
        case READ_DISK:
            break;
        case WRITE_DISK:
            break;
        case DISK_IOCTL:
            break;
        default:
            break;
    }
}

/********************************************exception test case **********************************/

/* Test following 2 cases:
1. Use shell to test file system function error prompts when function is being run and file/directory 
   not exist i.e disk mount/file create/file open/file read/file write/file opendir/ file readdir/file 
   mkdir/file getcwd etc encounter disk/file not exits error.
*/
void test_mmc_fatfs_exception_case(void)
{

}

/********************************************performance test case **********************************/
/*
1.Test the file read/write speed when file size is very big.
 */
void test_mmc_fatfs_performance_case(void)
{

}


/********************************************stress test case **********************************/
/*
Test file read/write can run 24 hours correctly
 */
void test_mmc_fatfs_stress_case(void)
{

}

int main(void)
{
    test_mmc_fatfs_normal_case();
    test_mmc_fatfs_exception_case();
    test_mmc_fatfs_performance_case();
    test_mmc_fatfs_stress_case();
    
    return 0;
}