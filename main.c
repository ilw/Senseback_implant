//FPGA programming image variables here
#define FPGAIMAGE_SIZE 71337

//Pin connections for schematics_test board (SENSEBACK)
//SPI:    CS - PIN 9		CSK - PIN 31    MOSI - PIN 10		MISO - PIN 30

//Define SPI CS pins
#define SPI_CS_PIN  9 /**< SPIS CS Pin. Should be shortened with @ref SPI_CS_PIN */
#define SPI_CS_PIN2 29

//ESB payload length
#define RX_PAYLOAD_LENGTH 250

//FPGA programming pins: reset, done
#define CS_RESET_B 28
#define CDONE 3
#define FPGA_RESET_PIN 2


//Include SDK files
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "sdk_common.h"
#include "nrf.h"
#include "nrf_esb.h"
#include "nrf_error.h"
#include "nrf_esb_error_codes.h"
#include "nrf_delay.h"
#include "nrf_drv_gpiote.h"
#include "nrf_drv_spi.h"
#include "app_error.h"
#include "app_util_platform.h"
#include "app_util.h"
#include "boards.h"
#include "incbin.h"
#include "flashwriting.h"

INCBIN(FPGAimg, "FPGAimage.bin");

//Make Microcontroller NFC pins usable as GPIOs
const uint32_t __attribute__((section (".uicr"))) UICR_ADDR_0x20C = 0xFFFFFFFE;
//const uint32_t     __attribute__((at(0x1000120C))) __attribute__((used)) = 0xFFFFFFFE;

//Instantiate SPI
#define SPI_INSTANCE 0 /**< SPI instance index. */
static const nrf_drv_spi_t spi = NRF_DRV_SPI_INSTANCE(SPI_INSTANCE);/**< SPI instance. */

//Declare SPI variables
static uint8_t       m_tx_buf[2];        /**< TX buffer. */
static uint8_t       m_rx_buf[2];//sizeof(DEFAULT_STRING)+1];    /**< RX buffer. */
static const uint8_t m_length = 2;        		/**< Transfer length. */
static volatile uint8_t transmit_to_chip[1024]; //spi FIFO-style buffer with read and write pointers
static volatile uint32_t spibuffer_r_ptr = 0;
static volatile uint32_t spibuffer_w_ptr = 0;
static uint32_t fpga_checksum = 0;
static volatile uint32_t spitransaction_flag = 0, readpackets_flag = 0, fpga_accept_flag = 0, debug_flag = 0;
static volatile int spibuffer_sz = 0;
static volatile uint8_t packetid = 0;
static volatile int8_t tx_fifo_size = 0;

//static volatile uint32_t *spi_rx_flag_ptr = &spi_rx_flag, *spitransaction_flag_ptr = &spitransaction_flag, *spi_tx_flag_ptr = &spi_tx_flag;

//Declare ESB data variables
//static volatile bool esb_xfer_done;
static nrf_esb_payload_t rx_payload;

static nrf_esb_payload_t tx_payload = NRF_ESB_CREATE_PAYLOAD(0, 0x00);
static nrf_esb_payload_t rxfifo_empty_payload = NRF_ESB_CREATE_PAYLOAD(0, 0x00, 0xF1, 0xF0);
static nrf_esb_payload_t validation_payload = NRF_ESB_CREATE_PAYLOAD(0, 0x00, 0xFB, 0x9A, 0x00);

//Other variables
static uint32_t errcode;

//Start system high freq clock
void clocks_start( void )
{
    NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
    NRF_CLOCK->TASKS_HFCLKSTART = 1;

    while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0);
}

//ESB event handler - only receive events are relevant
void nrf_esb_event_handler(nrf_esb_evt_t const * p_event)
{
    switch (p_event->evt_id)
    {
        case NRF_ESB_EVENT_TX_SUCCESS:


            break;
        case NRF_ESB_EVENT_TX_FAILED:

            break;
        case NRF_ESB_EVENT_RX_RECEIVED:
        	//Received ESB payload
        	readpackets_flag = 1;
            break;
    }
}

//Initialize ESB
void esb_init( void )
{
    uint8_t base_addr_0[4] = {0xE7, 0xE7, 0xE7, 0xE7};
    uint8_t base_addr_1[4] = {0xC2, 0xC2, 0xC2, 0xC2};
    uint8_t addr_prefix[8] = {0xE7, 0xC2, 0xC3, 0xC4, 0xC5, 0xC6, 0xC7, 0xC8 };

		//Configuration params
    nrf_esb_config_t nrf_esb_config         = NRF_ESB_DEFAULT_CONFIG;
	nrf_esb_config.protocol                 = NRF_ESB_PROTOCOL_ESB_DPL;
    nrf_esb_config.payload_length           = RX_PAYLOAD_LENGTH;
    nrf_esb_config.bitrate                  = NRF_ESB_BITRATE_2MBPS;
    nrf_esb_config.mode                     = NRF_ESB_MODE_PRX;			//rx mode
    nrf_esb_config.event_handler            = nrf_esb_event_handler;
    nrf_esb_config.selective_auto_ack       = false;
    nrf_esb_config.retransmit_count			= 5;
    //nrf_esb_config.retransmit_delay			= 500;

    nrf_esb_init(&nrf_esb_config);

    nrf_esb_set_base_address_0(base_addr_0);

    nrf_esb_set_base_address_1(base_addr_1);

    nrf_esb_set_prefixes(addr_prefix, 8);
}

void in_pin_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action) {

	//Called on detection of state high on the spi irq line pin driven by the fpga. Triggers SPI transfer provided SPI is idle.
	if (spitransaction_flag == 0) {
	spitransaction_flag = 1;
	}
}

//{ FPGA PROGRAMMING FUNCTIONS

void bitbang_spi(uint8_t data)
{
	int i;
	nrf_gpio_pin_clear(SPI_CS_PIN2);// Set CS line of FPGA programming spi (active) low
	nrf_delay_us(1);// small delay
	for (i=0;i<8;i++) {
		//SPI data: CLK normally HI, latched on transition from LO to HI

		nrf_gpio_pin_clear(SPI0_CONFIG_SCK_PIN);// Set SPI_CLK low
		nrf_delay_us(1);// small delay
		//Program FPGA MSB first
		if (((data<<i) & 0x80) == 0x80) { //if checked bit is HI
			nrf_gpio_pin_set(SPI0_CONFIG_MOSI_PIN);// Set SPI_MOSI high
		}
		else {
			nrf_gpio_pin_clear(SPI0_CONFIG_MOSI_PIN);// Set SPI_MOSI low
		}
		nrf_delay_us(1);// small Delay
		nrf_gpio_pin_set(SPI0_CONFIG_SCK_PIN);// Set SPI_CLK high
		nrf_delay_us(1);// small Delay

	}
	nrf_gpio_pin_set(SPI_CS_PIN2);// Set CS line of FPGA programming spi (inactive) high
	nrf_delay_us(1);// small delay

}

void Send_Clocks(int num_clocks)
{
	int i;
	for (i = 0; i < num_clocks; i++)
	{
		nrf_gpio_pin_clear(SPI0_CONFIG_SCK_PIN);// Set SPI_CLK low
		nrf_delay_us(1);// small delay
		nrf_gpio_pin_set(SPI0_CONFIG_SCK_PIN);// Set SPI_CLK high
		nrf_delay_us(1);// small Delay
	}
}



void config_FPGA(int newimage_flag)
{
	int i=0;
	uint32_t fpgaimage_size;
	uint8_t* addr = (uint8_t*)start_addr;
	//uint8_t* fpgaimage_addr;

	//Set spi and creset pins as outputs
	nrf_gpio_pin_dir_set(SPI0_CONFIG_SCK_PIN,NRF_GPIO_PIN_DIR_OUTPUT);
	nrf_gpio_pin_dir_set(SPI0_CONFIG_MOSI_PIN,NRF_GPIO_PIN_DIR_OUTPUT);
	nrf_gpio_pin_dir_set(SPI_CS_PIN2,NRF_GPIO_PIN_DIR_OUTPUT);
	nrf_gpio_pin_dir_set(CS_RESET_B,NRF_GPIO_PIN_DIR_OUTPUT);
	nrf_gpio_pin_dir_set(CDONE,NRF_GPIO_PIN_DIR_INPUT);


	// Clear SSEL and CS_Reset
	nrf_gpio_pin_clear(SPI_CS_PIN2);
	nrf_gpio_pin_clear(CS_RESET_B);
	nrf_gpio_pin_clear(SPI0_CONFIG_MOSI_PIN);

	//Set clk
	nrf_gpio_pin_set(SPI0_CONFIG_SCK_PIN);

	//delay >200ns
	nrf_delay_us(10);

	//set creset high
	nrf_gpio_pin_set(CS_RESET_B);

	//delay >1.2ms
	nrf_delay_ms(3);

	//set ssel high
	nrf_gpio_pin_set(SPI_CS_PIN2);

	//send 8 clocks
	Send_Clocks(8);

	if (newimage_flag == 1) { //we have a new FPGA image
		fpgaimage_size = *(start_addr+1);
		for (i=0;i<fpgaimage_size;i++)
		{

			bitbang_spi(*(addr+8+i));
		}

		//fpgaimage_addr = (uint8_t*)(start_addr+2);
	}
	else {
		//fpgaimage_size = gFPGAimgSize;
		//fpgaimage_addr = gFPGAimgData;
		for (i=0;i<gFPGAimgSize;i++)
		{
			bitbang_spi(*(gFPGAimgData+i));
		}
	}
	//Program FPGA (call spi_bitbang) here
	//Send file 1 byte at a time


	//Take control of sck again
	nrf_gpio_pin_set(SPI_CS_PIN2);

	//send 100 clocks
	Send_Clocks(100);

	//Return pins to default configurations //Necessary?
	nrf_gpio_cfg_default(SPI0_CONFIG_SCK_PIN );
	nrf_gpio_cfg_default(SPI0_CONFIG_MOSI_PIN );

	if   (	nrf_gpio_pin_read(CDONE)) {
		return;// PASS if CDONE is true -
	}
	else {
		validation_payload.data[3] = 0x00;
		validation_payload.data[0] = packetid;
		nrf_esb_write_payload(&validation_payload);
		tx_fifo_size++;
		packetid++;
	}
}

void spi_transaction() {
	int i;
	while (spibuffer_sz > 0 || nrf_drv_gpiote_in_is_set(26)) {
		if (spibuffer_sz > 0) {
			m_tx_buf[0] = transmit_to_chip[spibuffer_r_ptr];
			spibuffer_r_ptr++;
			spibuffer_sz--;
			m_tx_buf[1] = transmit_to_chip[spibuffer_r_ptr];
			spibuffer_r_ptr++;
			spibuffer_sz--;
			if (spibuffer_r_ptr >= 1024) {
				spibuffer_r_ptr = 0;
			}
			if (spibuffer_sz < 0) {
				//ERROR CONDITION WE HAVE SOMEHOW OVERREAD THE BUFFER; NOTIFY USER (TODO)
			}
		}
		else {
			m_tx_buf[0] = 0;
			m_tx_buf[1] = 0;
		}
		nrf_drv_spi_transfer(&spi, m_tx_buf, m_length, m_rx_buf, m_length);

		if (m_rx_buf[0] == 0x00 && m_rx_buf[1] == 0x00) {
			//Data is all zeroes, do nothing
		}
		else {
			tx_payload.data[tx_payload.length] = m_rx_buf[0];
			tx_payload.length++;
			tx_payload.data[tx_payload.length] = m_rx_buf[1];
			tx_payload.length++;
		}

		if (tx_payload.length>=RX_PAYLOAD_LENGTH) { //if buffer full, push packet to FIFO
			tx_payload.data[0] = packetid;
			errcode = nrf_esb_write_payload(&tx_payload);
			if (errcode != 0) {
				i = -1;
			}
			else {
				tx_fifo_size++;
				packetid++;
			}
			for (i=0;i<tx_payload.length;i++) {
				tx_payload.data[i] = 0;
			}
			tx_payload.length = 1;
			return;
		}
		m_rx_buf[0] = 0; //Clear buffer
		m_rx_buf[1] = 0; //Clear buffer
	}
	spitransaction_flag = 0;
}

int main(void)
{
		int i=0;
		unsigned int fpgaimage_intcount = 0;
		uint32_t tmp = 0;

	//Start clocks and init ESB
	clocks_start();
	esb_init();

	init_flash(0); //Call the function to initialize flash variables (start address location etc) without erasing any flash.
	nrf_gpio_pin_dir_set(FPGA_RESET_PIN,NRF_GPIO_PIN_DIR_OUTPUT);
	nrf_gpio_pin_set(FPGA_RESET_PIN);
	if (*start_addr != 0xFFFFFFFF) { //The flash isn't erased; there's a valid FPGA image in flash
		config_FPGA(1);
	}
	else {
		config_FPGA(0);
	}
	nrf_gpio_pin_clear(FPGA_RESET_PIN);
	nrf_delay_ms(3);
	nrf_gpio_pin_set(FPGA_RESET_PIN);

	//Initialize SPI
    nrf_drv_spi_config_t spi_config = NRF_DRV_SPI_DEFAULT_CONFIG(SPI_INSTANCE);
    	spi_config.ss_pin               = SPI_CS_PIN;
    	spi_config.mode					= NRF_DRV_SPI_MODE_0;
    nrf_drv_spi_init(&spi, &spi_config, NULL);

	for (i=0;i<1024;i++) { //initialize SPI FIFO buffer
		transmit_to_chip[i] = 0;
	}

	if(!nrf_drv_gpiote_is_init())	{
			nrf_drv_gpiote_init();
	}


	nrf_drv_gpiote_in_config_t config = GPIOTE_CONFIG_IN_SENSE_LOTOHI(true); //configure input pin using high frequency clocks for maximum responsiveness
	config.pull = NRF_GPIO_PIN_PULLDOWN;
	nrf_drv_gpiote_in_init(26, &config, in_pin_handler); //set watch on pin 26 calling in_pin_handler on pin state change from low to high
	nrf_drv_gpiote_in_event_enable(26, true);

	//Start ESB reception
	tx_payload.length = 1;
	tx_payload.data[0] = packetid;
	nrf_esb_start_rx();

	while(true) {
		if (readpackets_flag == 1) {
			while (nrf_esb_read_rx_payload(&rx_payload) == NRF_SUCCESS) {
				tx_fifo_size--;
				if (tx_fifo_size <= 0) {
					rxfifo_empty_payload.data[0] = packetid;
					errcode = nrf_esb_write_payload(&rxfifo_empty_payload);
					packetid++;
					tx_fifo_size = 1;
				}
				//nrf_esb_flush_tx();
				if (rx_payload.length%2 == 0) { //If payload contains even number of bytes then it is a data packet to be transmitted to chip
					//Transfer data to intermediate buffer
					int length = rx_payload.length/2;

					for (i=0;i<length;i++) {
						transmit_to_chip[spibuffer_w_ptr] = rx_payload.data[2*i];
						spibuffer_w_ptr++;
						spibuffer_sz++;
						transmit_to_chip[spibuffer_w_ptr] = rx_payload.data[2*i+1];
						if ((spibuffer_w_ptr+1) >= 1024) {
							spibuffer_w_ptr = 0;
							spibuffer_sz++;
						}
						else {
							spibuffer_w_ptr++;
							spibuffer_sz++;
						}
						if (spibuffer_w_ptr == spibuffer_r_ptr) {
							//ERROR CONDITION WE ARE OVERWRITING UNSENT DATA; NOTIFY USER (TODO)
						}
					}
					//Load SPI buffers and fire
					spi_transaction();

				}
				else { //If payload contains odd number of bytes then it is a command packet
					if (rx_payload.data[0] == 0x61) {
						//Dummy command: used by TX to drain the RX esb FIFO
						//Could use switch case here instead of (potentially) many if statements

					}
					else if ((rx_payload.data[0] == 0x71) && (rx_payload.data[1] == 0x71) && (rx_payload.data[2] == 0x71)) {
						//QUERY COMMAND ("QQ"): flushes the current SPI buffer into the next packet payload to be sent

						if (tx_payload.length > 1 && tx_payload.length <= 32) {
							for (i=tx_payload.length;i<32;i++) {
								tx_payload.data[i] = 0;
							}
							tx_payload.length = 33;
							tx_payload.data[0] = packetid;
							errcode = nrf_esb_write_payload(&tx_payload);
							if (errcode != 0) {
								i = -1;
							}
							else {
								tx_fifo_size++;
								packetid++;
							}
							for (i=0;i<tx_payload.length;i++) {
								tx_payload.data[i] = 0;
							}
							tx_payload.length = 1;
						}
						else if (tx_payload.length > 32) {
							tx_payload.data[0] = packetid;
							errcode = nrf_esb_write_payload(&tx_payload);
							if (errcode != 0) {
								i = -1;
								//TODO: pop tx fifo and instead insert a payload to alert TX that RX tx FIFO is full, on next transmission
							}
							else {
								tx_fifo_size++;
								packetid++;
							}
							for (i=0;i<tx_payload.length;i++) {
								tx_payload.data[i] = 0;
							}
							tx_payload.length = 1;
						}



					}
					else {
						if (rx_payload.data[0] == 0x12 && rx_payload.data[1] == 0x35 && rx_payload.data[2] == 0x37) {
							//Reset command: received when TX is booted, or when manual reset of RX is needed.
							//Reset actions: reset FPGA, reset chip, flush RX and TX FIFO, reinitialize SPI, set FIFO counter to 0
							//Note: can be done with full reset of MCU
							nrf_delay_ms(200);
							NVIC_SystemReset();
						}
						if ((rx_payload.data[0] == 0xFB) && (rx_payload.data[1] == 0x9A) && (rx_payload.data[2] == 0xBB)) { //Command: begin fpga image upload
							fpga_checksum = 0;
							fpgaimage_intcount = 0;
							fpga_accept_flag = 2;
							nrf_delay_ms(200);
							init_flash(1);
							validation_payload.data[0] = packetid;
							validation_payload.data[3] = 0x01;
							nrf_esb_write_payload(&validation_payload);
							tx_fifo_size++;
							packetid++;
							validation_payload.data[3] = 0x00;
							debug_flag++;
						}
						else if ((rx_payload.data[0] == 0xFB) && (rx_payload.data[1] == 0x9A) && (rx_payload.data[2] == 0xCC)) { //Command: validate fpga image
							fpga_accept_flag = 1;
							tmp = *((uint32_t*)(rx_payload.data+3));
							//if (fpgaimage_intcount != FPGAIMAGE_SIZE) {fpga_accept_flag = 0;}
							if (fpga_checksum != tmp) {fpga_accept_flag = 0;}
							if (fpga_accept_flag == 1) {
								flash_array_write((uint32_t *)(start_addr),&rx_payload.data[3],2,&fpga_checksum);
								validation_payload.data[3] = 0x01;
								validation_payload.data[0] = packetid;
								nrf_esb_write_payload(&validation_payload);
								tx_fifo_size++;
								packetid++;
								validation_payload.data[3] = 0x00;
							}
							else {
								fpga_checksum = 0;
								fpgaimage_intcount = 0;
								nrf_delay_ms(200);
								init_flash(1);
								validation_payload.data[0] = packetid;
								nrf_esb_write_payload(&validation_payload);
								tx_fifo_size++;
								packetid++;
								validation_payload.data[3] = 0x00;
							}

						}
						else if ((rx_payload.data[0] == 0xFB) && (rx_payload.data[1] == 0x9A)) { //Packet is fpga image data
							if (fpga_accept_flag == 2) {
								flash_array_write((uint32_t *)(start_addr+fpgaimage_intcount+2),&rx_payload.data[3],rx_payload.data[2],&fpga_checksum);
								//nrf_delay_ms(100);
								fpgaimage_intcount += rx_payload.data[2];
								validation_payload.data[3] = 0x01;
								validation_payload.data[0] = packetid;
								nrf_esb_write_payload(&validation_payload);
								tx_fifo_size++;
								packetid++;
								debug_flag = 1;
							}
							else {
								validation_payload.data[3] = 0x00;
								validation_payload.data[0] = packetid;
								nrf_esb_write_payload(&validation_payload);
								tx_fifo_size++;
								packetid++;
							}
							validation_payload.data[3] = 0x00;
						}
						else {

						}
					}
				}
			}
			//No more packets in FIFO to be read
			readpackets_flag = 0;
		}
		if (spitransaction_flag == 1) {
			spi_transaction();
		}
    }
}

