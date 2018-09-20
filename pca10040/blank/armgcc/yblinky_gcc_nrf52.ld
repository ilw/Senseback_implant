/* Linker script to configure memory regions. */

SEARCH_DIR(.)
GROUP(-lgcc -lc -lnosys)

MEMORY
{
  UICR (rx) : ORIGIN = 0x10001000, LENGTH = 0x0FFFF000
  RAM (rwx) :  ORIGIN = 0x20000000, LENGTH = 0x10000
}

SECTIONS
{
  .uicr 0x1000120C :
  {
	KEEP(*(.uicr))
  } > UICR
} INSERT AFTER .data;

INCLUDE "nrf5x_common.ld"
