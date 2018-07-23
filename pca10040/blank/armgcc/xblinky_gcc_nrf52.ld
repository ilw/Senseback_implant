/* Linker script to configure memory regions. */

SEARCH_DIR(.)
GROUP(-lgcc -lc -lnosys)

MEMORY
{
  FLASH (rx) : ORIGIN = 0x40000, LENGTH = 0x40000
  UICR (rx) : ORIGIN = 0x10001000, LENGTH = 0x0FFFF000
  RAM (rwx) :  ORIGIN = 0x20000000, LENGTH = 0x10000
}



INCLUDE "nrf5x_common.ld"
