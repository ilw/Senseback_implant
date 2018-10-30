
	

#include <stdint.h>
#include <stdbool.h>
#include "nrf.h"
#include "flashwriting.h"
#include "nrf_mbr.h"
#ifdef NRF52
#define FLASH_PAGE_SIZE (MBR_PAGE_SIZE_IN_WORDS * sizeof(uint32_t))
#define FPGA_IMAGE_PAGES (uint32_t)(FPGA_IMAGE_SIZE/FLASH_PAGE_SIZE) //0x1000 is the page size on NRF52
#define NRF_DFU_APP_DATA_AREA_SIZE FLASH_PAGE_SIZE*FPGA_IMAGE_PAGES
#endif

#ifdef SDK15_2
#define BOOTLOADER_REGION_START 0x00078000
#endif

#include "nrf_dfu_types.h"





/******************************************************
* FUNCTION DECLARATIONS
*******************************************************/
uint32_t * start_addr;

/** 
 * Function for erasing a page in flash.
 * @param page_address Address of the first word in the page to be erased.
 */
void flash_page_erase(uint32_t * page_address)
{
    // Turn on flash erase enable and wait until the NVMC is ready:
    NRF_NVMC->CONFIG = (NVMC_CONFIG_WEN_Een << NVMC_CONFIG_WEN_Pos);

    while (NRF_NVMC->READY == NVMC_READY_READY_Busy)
    {
        // Do nothing.
    }

    // Erase page:
    NRF_NVMC->ERASEPAGE = (uint32_t)page_address;

    while (NRF_NVMC->READY == NVMC_READY_READY_Busy)
    {
        // Do nothing.
    }

    // Turn off flash erase enable and wait until the NVMC is ready:
    NRF_NVMC->CONFIG = (NVMC_CONFIG_WEN_Ren << NVMC_CONFIG_WEN_Pos);

    while (NRF_NVMC->READY == NVMC_READY_READY_Busy)
    {
        // Do nothing.
    }
}


/** 
 * Function for filling a page in flash with a value.
 * @param[in] address Address of the first word in the page to be filled.
 * @param[in] value Value to be written to flash.
 * Note 32 bit words written! To write less than 32 bit words set the bits that should
 * remain unchanged in the word to '1'
 */
void flash_word_write(uint32_t * address, uint32_t value)
{
    // Turn on flash write enable and wait until the NVMC is ready:
    NRF_NVMC->CONFIG = (NVMC_CONFIG_WEN_Wen << NVMC_CONFIG_WEN_Pos);

    while (NRF_NVMC->READY == NVMC_READY_READY_Busy)
    {
        // Do nothing.
    }

    *address = value;

    while (NRF_NVMC->READY == NVMC_READY_READY_Busy)
    {
        // Do nothing.
    }

    // Turn off flash write enable and wait until the NVMC is ready:
    NRF_NVMC->CONFIG = (NVMC_CONFIG_WEN_Ren << NVMC_CONFIG_WEN_Pos);

    while (NRF_NVMC->READY == NVMC_READY_READY_Busy)
    {
        // Do nothing.
    }
}



void init_flash(int erase)
{
	int j;
	uint32_t * addr;
	uint32_t  pg_size = CODE_PAGE_SIZE;
	uint32_t start_pg_num = (BOOTLOADER_REGION_START/pg_size) - (uint32_t)(FPGA_IMAGE_SIZE/pg_size + 1);  // Use last pages in flash with 14 pages


	//while (BOOTLOADER_REGION_START != (DFU_REGION_END)) {}; //catch any errors wrt bootloader address
	start_addr = (uint32_t *) (start_pg_num * pg_size);

	//Erase the pages
	if (erase == 1) {
		for (j=0; j<(71339/pg_size + 1);j++)
		{
			// Set address:
			addr = (uint32_t *)(start_addr + j*pg_size/4); //Divide by 4 because address is a count of 32 bit words and pg_size is a count of bytes
			// Erase page:
			flash_page_erase(addr);
		}
	}
}






/*********************************************** 
* Function to write an array - slightly dodgy casting?
************************************************/
bool flash_array_write(uint32_t * address, uint8_t dataIn[], uint32_t dataLength, uint32_t * checksum)
{
	uint32_t  * data;
	int j;

	if ((dataLength*4)%4 !=0) return false; //Had to make temporary change while I figure out how to pass actual number of bytes instead of number of ints.
	//Would the line above ever fail?
	data = (uint32_t *) dataIn;  //Is this ok?
	
	for (j=0;j<dataLength;j++)
	{
		flash_word_write(address+j, *(data+j)); //data is 4 bytes
		*checksum += *(data+j);
	}
	return true;
}	
	
	
	
	




