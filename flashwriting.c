
	
#include "bsp.h"

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
	uint32_t  pg_size = NRF_FICR->CODEPAGESIZE;
	uint32_t  start_pg_num = NRF_FICR->CODESIZE - (71339/pg_size + 1);  // Use last pages in flash
	start_addr = (uint32_t *) (start_pg_num * pg_size);

	//Erase the pages
	if (erase == 1) {
		for (j=0; j<(71339/pg_size + 1);j++)
		{
			// Set address:
			addr = (uint32_t *)(start_addr + j*pg_size/4);
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

	if ((dataLength*4)%4 !=0) return false; //Had to make temporary change while I figure out how to pass actual number of bytes instead of number of ints
	data = (uint32_t *) dataIn;  //Is this ok?
	
	for (j=0;j<dataLength;j++)
	{
		flash_word_write(address+j, *(data+j)); //data is 4 bytes
		*checksum += *(data+j);
	}
	return true;
}	
	
	
	
	




