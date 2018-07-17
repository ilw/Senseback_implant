/******************************************************
* INCLUDES & DEFINES
*******************************************************/





/******************************************************
* Global variables
*******************************************************/

extern uint32_t * start_addr;

void flashwriting_page_erase(uint32_t * page_address);

void flash_word_write(uint32_t * address, uint32_t value);

void init_flash();

bool flash_array_write(uint32_t * address, uint8_t dataIn[], uint32_t dataLength, uint32_t * checksum);

void flashwriting_block_write(uint32_t * address, uint32_t const * const dataIn, uint32_t dataLength);

uint32_t flash_word_read(uint32_t * address);
