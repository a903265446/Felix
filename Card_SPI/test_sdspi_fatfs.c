

#include <stdint.h>


#include "ff.h"

#include "test_sdspi_fatfs.h"
#include "test_sdspi_function.h"


/************************************test normal case ********************************************/

/*
Use a shell to test file system function i.e disk mount/file create/file open/file read/
file write/file opendir/ file readdir/file mkdir/file getcwd etc.
 */
#define INIT_DISK 0
#define GET_DISK_STATUS 1
#define READ_DISK 2
#define WRITE_DISK 3
#define DISK_IOCTL 4

static FATFS fatFS; /* File system object */
static FIL fileObj; /* File object */

/* buffer size (in byte) for R/W operations */
#define BUFFER_SIZE     4096
static uint8_t buffer[BUFFER_SIZE / 4];     /* Working buffer */

void test_sdspi_fatfs_normal_case(void)
{
    FRESULT rc;		/* Result code */
    uint32_t instruction = 0;
    uint32_t bw, br, i;
    /* instruction is input by user */

//    switch (instruction)
//    {
//        case INIT_DISK:
//            break;
//        case GET_DISK_STATUS:
//            break;
//        case READ_DISK:
//            break;
//        case WRITE_DISK:
//            break;
//        case DISK_IOCTL:
//            break;
//        default:
//            break;
//    }
//    
    f_mount(1, &fatFS);       /* Register volume work area (never fails) */

    rc = f_open(&fileObj, "1:/MESSAGE.TXT", FA_READ);
    if (rc) {
        return;
    }
    else {
        for (;; ) {
            /* Read a chunk of file */
            rc = f_read(&fileObj, buffer, sizeof(buffer), &br);
            if (rc || !br) {
                break;                  /* Error or end of file */
            }
            //ptr = (uint8_t *) buffer;
            // for (i = 0; i < br; i++) {  /* Type the data */
            //     DEBUGOUT("%c", ptr[i]);
            // }
        }
        if (rc) {
            return;
        }

        rc = f_close(&fileObj);
        if (rc) {
            return;
        }
    }
}



/*******************************Test exception case***********************************************/

/*
Use a shell to test file system function error prompts when function is being run and file
/directory not exist i.e disk mount/file create/file open/file read/file write/file opendir/ 
file readdir/file mkdir/file getcwd etc encounter disk/file not exits error.

Use a shell to test file system function i.e disk mount/file create/file open/file read/file write
/file opendir/ file readdir/file mkdir/file getcwd etc  when file system function is being run and 
card is suddenly removed by the user.
 */
void test_sdspi_fatfs_exception_case(void)
{

}


/**********************************Test performance***********************************************/
/*
Test the file read/write speed when file size is very big.
 */
void test_sdspi_fatfs_performance_case(void)
{

}



/**********************************Sress test***********************************************/

/*
Test file read/write can run 24 hours correctly
 */
void test_sdspi_fatfs_stress_case(void)
{

}

int main(void)
{
    init_sdspi_hardware();
    
    test_sdspi_fatfs_normal_case();
    test_sdspi_fatfs_exception_case();
    test_sdspi_fatfs_performance_case();
    test_sdspi_fatfs_stress_case();
    
    return 0;
}
