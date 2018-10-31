/* Linker script to configure memory regions. */

SEARCH_DIR(.)
GROUP(-lgcc -lc -lnosys)

MEMORY
{
  FLASH (rx) : ORIGIN = 0x0, LENGTH = 0x80000
  UICR (rx) : ORIGIN = 0x10001000, LENGTH = 0x0FFFF000
  RAM (rwx) :  ORIGIN = 0x20000000, LENGTH = 0x10000
}

SECTIONS
{
  .uicr 0x1000120C :
  {
	KEEP(*(.uicr))
  } > UICR
  .fs_data :
  {
    PROVIDE(__start_fs_data = .);
    KEEP(*(.fs_data))
    PROVIDE(__stop_fs_data = .);
  } > RAM
} INSERT AFTER .data;

SECTIONS
{

.log_const_data :
{
PROVIDE(__start_log_const_data = .);
KEEP(*(.log_const_data))
PROVIDE(__stop_log_const_data = .);
} > FLASH

.sdh_stack_observers :
{
PROVIDE(__start_sdh_stack_observers = .);
KEEP(*(SORT(.sdh_stack_observers*)))
PROVIDE(__stop_sdh_stack_observers = .);
} > FLASH

.sdh_req_observers :
{
PROVIDE(__start_sdh_req_observers = .);
KEEP(*(SORT(.sdh_req_observers*)))
PROVIDE(__stop_sdh_req_observers = .);
} > FLASH

.sdh_state_observers :
{
PROVIDE(__start_sdh_state_observers = .);
KEEP(*(SORT(.sdh_state_observers*)))
PROVIDE(__stop_sdh_state_observers = .);
} > FLASH

.sdh_ble_observers :
{
PROVIDE(__start_sdh_ble_observers = .);
KEEP(*(SORT(.sdh_ble_observers*)))
PROVIDE(__stop_sdh_ble_observers = .);
} > FLASH

} INSERT AFTER .text

INCLUDE "nrf_common.ld"