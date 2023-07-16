/*
 * <ORIGINAL HEADER>
 * nrf24l01_plus.c
 *
 *  Created on: 2021. 7. 20.
 *      Author: mokhwasomssi
 * </ORIGINAL HEADER>
 */

/*
 * This file is modified from mokhwasomssi's version to fix breaking changes
 * and follow the datasheet more accurately.
 * Modified version by: muaiyadh
 *
 */


#include "nrf24l01p.h"


static void cs_high()
{
    HAL_GPIO_WritePin(NRF24L01P_SPI_CS_PIN_PORT, NRF24L01P_SPI_CS_PIN_NUMBER, GPIO_PIN_SET);
}

static void cs_low()
{
    HAL_GPIO_WritePin(NRF24L01P_SPI_CS_PIN_PORT, NRF24L01P_SPI_CS_PIN_NUMBER, GPIO_PIN_RESET);
}

static void ce_high()
{
    HAL_GPIO_WritePin(NRF24L01P_CE_PIN_PORT, NRF24L01P_CE_PIN_NUMBER, GPIO_PIN_SET);
}

static void ce_low()
{
    HAL_GPIO_WritePin(NRF24L01P_CE_PIN_PORT, NRF24L01P_CE_PIN_NUMBER, GPIO_PIN_RESET);
}

static uint8_t read_register(uint8_t reg)
{
    uint8_t command = NRF24L01P_CMD_R_REGISTER | reg;
    uint8_t status;
    uint8_t read_val;

    cs_low();
    HAL_SPI_TransmitReceive(NRF24L01P_SPI, &command, &status, 1, 2000);
    HAL_SPI_Receive(NRF24L01P_SPI, &read_val, 1, 2000);
    cs_high();

    return read_val;
}

static void read_register_multiple(uint8_t reg, uint8_t* buf, uint8_t len)
{
    uint8_t command = NRF24L01P_CMD_R_REGISTER | reg;
    uint8_t status;

    cs_low();
    HAL_SPI_TransmitReceive(NRF24L01P_SPI, &command, &status, 1, 2000);
    HAL_SPI_Receive(NRF24L01P_SPI, buf, len, 2000);
    cs_high();
}

static uint8_t write_register(uint8_t reg, uint8_t value)
{
    uint8_t command = NRF24L01P_CMD_W_REGISTER | reg;
    uint8_t status;
    uint8_t write_val = value;

    cs_low();
    HAL_SPI_TransmitReceive(NRF24L01P_SPI, &command, &status, 1, 2000);
    Fast_SPI_Transmit(NRF24L01P_SPI->Instance, &write_val, 1, false);
    cs_high();

    return write_val;
}

static void write_address(uint8_t addr, uint8_t value[5])
{
    uint8_t command = NRF24L01P_CMD_W_REGISTER | addr;
    uint8_t status;

    cs_low();
    HAL_SPI_TransmitReceive(NRF24L01P_SPI, &command, &status, 1, 2000);
    Fast_SPI_Transmit(NRF24L01P_SPI->Instance, value, 5, false);
    cs_high();
}

int _write(int file, char *ptr, int len)

{   int DataIdx;

    for(DataIdx = 0; DataIdx < len; DataIdx++)
    {
    	ITM_SendChar(*ptr++);
    }

     return len;

}

void print_byte_register(const char* name, uint8_t reg, uint8_t qty)
{
    printf("%s\t=", name);
    while (qty--) {
        printf(" 0x%02x", read_register(reg++));
    }
    printf("\r\n");
}

void print_address_register(const char* name, uint8_t reg, uint8_t qty)
{
    printf("%s\t=", name);
    const int addr_width = 5;
    while (qty--) {
        uint8_t* buffer = (uint8_t*)malloc(addr_width * sizeof(uint8_t));
        read_register_multiple(reg++ & NRF24L01P_REG_MASK, buffer, addr_width);

        printf(" 0x");
        uint8_t* bufptr = buffer + addr_width;
        while (--bufptr >= buffer) {
            printf("%02x", *bufptr);
        }
        free(buffer);
    }
    printf("\r\n");
}

#define _BV(x) (1 << (x))

air_data_rate getDataRate()
{
	air_data_rate result;
	uint8_t dr = read_register(NRF24L01P_REG_RF_SETUP) & (_BV(NRF24L01P_RF_DR_LOW) | _BV(NRF24L01P_RF_DR_HIGH));

	// switch uses RAM (evil!)
	// Order matters in our case below
	if (dr == _BV(NRF24L01P_RF_DR_LOW)) {
		// '10' = 250KBPS
		result = _250kbps;
	}
	else if (dr == _BV(NRF24L01P_RF_DR_HIGH)) {
		// '01' = 2MBPS
		result = _2Mbps;
	}
	else {
		// '00' = 1MBPS
		result = _1Mbps;
	}
	return result;
}

rf24_crclength_e getCRCLength()
{
    rf24_crclength_e result = RF24_CRC_DISABLED;
    uint8_t AA = read_register(NRF24L01P_REG_EN_AA);
    uint8_t config_reg = read_register(NRF24L01P_REG_CONFIG);

    if (config_reg & _BV(NRF24L01P_EN_CRC) || AA) {
        if (config_reg & _BV(NRF24L01P_CRCO)) {
            result = RF24_CRC_16;
        }
        else {
            result = RF24_CRC_8;
        }
    }

    return result;
}

void printDetails()
{
    //print_status(get_status());

    print_address_register("RX_ADDR_P0-1", NRF24L01P_REG_RX_ADDR_P0, 2);
    print_byte_register("RX_ADDR_P2-5", NRF24L01P_REG_RX_ADDR_P2, 4);
    print_address_register("TX_ADDR\t", NRF24L01P_REG_TX_ADDR, 1);

    print_byte_register("RX_PW_P0-6", NRF24L01P_REG_RX_PW_P0, 6);
    print_byte_register("EN_AA\t", NRF24L01P_REG_EN_AA, 1);
    print_byte_register("EN_RXADDR", NRF24L01P_REG_EN_RXADDR, 1);
    print_byte_register("RF_CH\t", NRF24L01P_REG_RF_CH, 1);
    print_byte_register("RF_SETUP", NRF24L01P_REG_RF_SETUP, 1);
    print_byte_register("CONFIG\t", NRF24L01P_REG_CONFIG, 1);
    print_byte_register("DYNPD/FEATURE", NRF24L01P_REG_DYNPD, 2);

    printf("Data Rate\t%d\r\n", getDataRate());
    //printf("Model\t\t%d= \r\n", isPVariant());
    printf("CRC Length\t%d\r\n", getCRCLength());
    //printf("PA Power\t%d\r\n", getPALevel());
}

/* nRF24L01+ Main Functions */
void nrf24l01p_rx_init(channel MHz, air_data_rate bps)
{
    nrf24l01p_reset();

    nrf24l01p_prx_mode();
    nrf24l01p_power_up();

    nrf24l01p_rx_set_payload_widths(NRF24L01P_PAYLOAD_LENGTH);

    nrf24l01p_set_rf_channel(MHz);
    nrf24l01p_set_rf_air_data_rate(bps);
    nrf24l01p_set_rf_tx_output_power(_0dBm);

    nrf24l01p_set_crc_length(1);
    nrf24l01p_set_address_widths(5);

    nrf24l01p_auto_retransmit_count(3);
    nrf24l01p_auto_retransmit_delay(250);
    
    ce_high();
}

void nrf24l01p_tx_init(channel MHz, air_data_rate bps, output_power out_pwr, uint8_t addr[5], bool auto_ack)
{
    nrf24l01p_reset();

    printf("Placeholder message\n"); // to get around SWV ITM data console first message not being sent properly
    printDetails();
    printf("\n\n\nAfter setting things up:\n");

    nrf24l01p_ptx_mode();
    nrf24l01p_power_up();

    nrf24l01p_set_rf_channel(MHz);
    nrf24l01p_set_rf_air_data_rate(bps);
    nrf24l01p_set_rf_tx_output_power(out_pwr);

    nrf24l01p_set_crc_length(1);
    nrf24l01p_set_address_widths(5);

    nrf24l01p_tx_set_address(addr);

    if (auto_ack) {
        write_register(NRF24L01P_REG_EN_AA, 0x3F);
    	nrf24l01p_rx_set_address(addr, 0);
	    write_register(NRF24L01P_REG_RX_PW_P0, 0x20);
	    write_register(NRF24L01P_REG_RX_PW_P1, 0x20);
	    write_register(NRF24L01P_REG_RX_PW_P2, 0x20);
	    write_register(NRF24L01P_REG_RX_PW_P3, 0x20);
	    write_register(NRF24L01P_REG_RX_PW_P4, 0x20);
	    write_register(NRF24L01P_REG_RX_PW_P5, 0x20);
    } else {
    	write_register(NRF24L01P_REG_EN_AA, 0x00);
    }

    nrf24l01p_auto_retransmit_count(3);
    nrf24l01p_auto_retransmit_delay(250);

    ce_high();
}

void nrf24l01p_rx_receive(uint8_t* rx_payload)
{
    nrf24l01p_read_rx_fifo(rx_payload);
    nrf24l01p_clear_rx_dr();

    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
}

void nrf24l01p_tx_transmit(uint8_t* tx_payload, bool include_crc)
{
    nrf24l01p_write_tx_fifo(tx_payload, include_crc);
}

void nrf24l01p_rx_set_address(address addr, uint8_t pipe)
{
	if (pipe >= 0 && pipe <= 5)	// ensure valid pipe
		write_address(NRF24L01P_REG_RX_ADDR_P0 + pipe, addr);
}

void nrf24l01p_tx_set_address(address addr)
{
	write_address(NRF24L01P_REG_TX_ADDR, addr);
}

void nrf24l01p_tx_irq()
{
    uint8_t tx_ds = nrf24l01p_get_status();
    tx_ds &= 0x20;

    if(tx_ds)
    {   
        // TX_DS
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
        nrf24l01p_clear_tx_ds();
    }

    else
    {
        // MAX_RT
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, SET);
        nrf24l01p_clear_max_rt();
    }
}

/* nRF24L01+ Sub Functions */
void nrf24l01p_reset()
{
    // Reset pins
    cs_high();
    ce_low();

    // Reset registers
    write_register(NRF24L01P_REG_CONFIG, 0x08);
    write_register(NRF24L01P_REG_EN_AA, 0x3F);
    write_register(NRF24L01P_REG_EN_RXADDR, 0x03);
    write_register(NRF24L01P_REG_SETUP_AW, 0x03);
    write_register(NRF24L01P_REG_SETUP_RETR, 0x03);
    write_register(NRF24L01P_REG_RF_CH, 0x02);
    write_register(NRF24L01P_REG_RF_SETUP, 0x07);
    write_register(NRF24L01P_REG_STATUS, 0x7E);
    write_register(NRF24L01P_REG_OBSERVE_TX, 0x00);
    write_register(NRF24L01P_REG_RPD, 0x00);
    write_address(NRF24L01P_REG_RX_ADDR_P0, (address){0xE7,0xE7,0xE7,0xE7,0xE7});
    write_address(NRF24L01P_REG_RX_ADDR_P1, (address){0xC2,0xC2,0xC2,0xC2,0xC2});
    write_register(NRF24L01P_REG_RX_ADDR_P2, 0xC3);
    write_register(NRF24L01P_REG_RX_ADDR_P3, 0xC4);
    write_register(NRF24L01P_REG_RX_ADDR_P4, 0xC5);
    write_register(NRF24L01P_REG_RX_ADDR_P5, 0xC6);
    write_address(NRF24L01P_REG_TX_ADDR, (address){0xE7,0xE7,0xE7,0xE7,0xE7});
    write_register(NRF24L01P_REG_RX_PW_P0, 0x00);
    write_register(NRF24L01P_REG_RX_PW_P0, 0x00);
    write_register(NRF24L01P_REG_RX_PW_P1, 0x00);
    write_register(NRF24L01P_REG_RX_PW_P2, 0x00);
    write_register(NRF24L01P_REG_RX_PW_P3, 0x00);
    write_register(NRF24L01P_REG_RX_PW_P4, 0x00);
    write_register(NRF24L01P_REG_RX_PW_P5, 0x00);
    write_register(NRF24L01P_REG_FIFO_STATUS, 0x11);
    write_register(NRF24L01P_REG_DYNPD, 0x00);
    write_register(NRF24L01P_REG_FEATURE, 0x00);

    // Reset FIFO
    nrf24l01p_flush_rx_fifo();
    nrf24l01p_flush_tx_fifo();
}

void nrf24l01p_prx_mode()
{
    uint8_t new_config = read_register(NRF24L01P_REG_CONFIG);
    new_config |= 1 << 0;

    write_register(NRF24L01P_REG_CONFIG, new_config);
}

void nrf24l01p_ptx_mode()
{
    uint8_t new_config = read_register(NRF24L01P_REG_CONFIG);
    new_config &= 0xFE;
    //new_config &= 0x7E;

    write_register(NRF24L01P_REG_CONFIG, new_config);


//    write_register(NRF24L01P_REG_RX_PW_P0, 0x20);
//    write_register(NRF24L01P_REG_RX_PW_P1, 0x20);
//    write_register(NRF24L01P_REG_RX_PW_P2, 0x20);
//    write_register(NRF24L01P_REG_RX_PW_P3, 0x20);
//    write_register(NRF24L01P_REG_RX_PW_P4, 0x20);
//    write_register(NRF24L01P_REG_RX_PW_P5, 0x20);
}

uint8_t nrf24l01p_read_rx_fifo(uint8_t* rx_payload)
{
    uint8_t command = NRF24L01P_CMD_R_RX_PAYLOAD;
    uint8_t status;

    cs_low();
    HAL_SPI_TransmitReceive(NRF24L01P_SPI, &command, &status, 1, 2000);
    HAL_SPI_Receive(NRF24L01P_SPI, rx_payload, NRF24L01P_PAYLOAD_LENGTH, 2000);
    cs_high();

    return status;
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
    cs_high();
}

uint8_t nrf24l01p_write_tx_fifo(uint8_t* tx_payload, bool include_crc)
{
    uint8_t command = NRF24L01P_CMD_W_TX_PAYLOAD;
    uint8_t status;

    cs_low();
    NRF24L01P_SPI->Instance->CR1 &= ~SPI_CR1_CRCEN;
    NRF24L01P_SPI->Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    HAL_SPI_TransmitReceive(NRF24L01P_SPI, &command, &status, 1, 2000);
    //Fast_SPI_Transmit(NRF24L01P_SPI->Instance, tx_payload, NRF24L01P_PAYLOAD_LENGTH, false);
    if (include_crc) {
    	NRF24L01P_SPI->Instance->CR1 |= SPI_CR1_CRCEN;
    	NRF24L01P_SPI->Init.CRCCalculation = SPI_CRCCALCULATION_ENABLE;
    }
    HAL_SPI_Transmit_DMA(NRF24L01P_SPI, tx_payload, NRF24L01P_PAYLOAD_LENGTH - include_crc);


    return status;
}


void nrf24l01p_flush_rx_fifo()
{
    uint8_t command = NRF24L01P_CMD_FLUSH_RX;
    uint8_t status;

    cs_low();
    HAL_SPI_TransmitReceive(NRF24L01P_SPI, &command, &status, 1, 2000);
    cs_high();
}

void nrf24l01p_flush_tx_fifo()
{
    uint8_t command = NRF24L01P_CMD_FLUSH_TX;
    uint8_t status;

    cs_low();
    HAL_SPI_TransmitReceive(NRF24L01P_SPI, &command, &status, 1, 2000);
    cs_high();
}

uint8_t nrf24l01p_get_status()
{
    uint8_t command = NRF24L01P_CMD_NOP;
    uint8_t status;

    cs_low();
    HAL_SPI_TransmitReceive(NRF24L01P_SPI, &command, &status, 1, 2000);
    cs_high();

    return status;
}

uint8_t nrf24l01p_get_fifo_status()
{
    return read_register(NRF24L01P_REG_FIFO_STATUS);
}

void nrf24l01p_rx_set_payload_widths(widths bytes)
{
    write_register(NRF24L01P_REG_RX_PW_P0, bytes);
}

void nrf24l01p_clear_rx_dr()
{
    uint8_t new_status = nrf24l01p_get_status();
    new_status |= 0x40;

    write_register(NRF24L01P_REG_STATUS, new_status);
}

void nrf24l01p_clear_tx_ds()
{
    uint8_t new_status = nrf24l01p_get_status();
    new_status |= 0x20;

    write_register(NRF24L01P_REG_STATUS, new_status);     
}

void nrf24l01p_clear_max_rt()
{
    uint8_t new_status = nrf24l01p_get_status();
    new_status |= 0x10;

    write_register(NRF24L01P_REG_STATUS, new_status); 
}

void nrf24l01p_power_up()
{
    uint8_t new_config = read_register(NRF24L01P_REG_CONFIG);
    new_config |= 1 << 1;

    write_register(NRF24L01P_REG_CONFIG, new_config);
}

void nrf24l01p_power_down()
{
    uint8_t new_config = read_register(NRF24L01P_REG_CONFIG);
    new_config &= 0xFD;

    write_register(NRF24L01P_REG_CONFIG, new_config);
}

void nrf24l01p_set_crc_length(length bytes)
{
    uint8_t new_config = read_register(NRF24L01P_REG_CONFIG);
    
    switch(bytes)
    {
        // CRCO bit in CONFIG resiger set 0
        case 1:
            new_config &= 0xFB;
            break;
        // CRCO bit in CONFIG resiger set 1
        case 2:
            new_config |= 1 << 2;
            break;
    }

    write_register(NRF24L01P_REG_CONFIG, new_config);
}

void nrf24l01p_set_address_widths(widths bytes)
{
    write_register(NRF24L01P_REG_SETUP_AW, bytes - 2);
}

void nrf24l01p_auto_retransmit_count(count cnt)
{
    uint8_t new_setup_retr = read_register(NRF24L01P_REG_SETUP_RETR);
    
    // Reset ARC register 0
    new_setup_retr |= 0xF0;
    new_setup_retr |= cnt;
    write_register(NRF24L01P_REG_SETUP_RETR, new_setup_retr);
}

void nrf24l01p_auto_retransmit_delay(delay us)
{
    uint8_t new_setup_retr = read_register(NRF24L01P_REG_SETUP_RETR);

    // Reset ARD register 0
    new_setup_retr |= 0x0F;
    new_setup_retr |= ((us / 250) - 1) << 4;
    write_register(NRF24L01P_REG_SETUP_RETR, new_setup_retr);
}

void nrf24l01p_set_rf_channel(channel MHz)
{
	uint16_t new_rf_ch = MHz - 2400;
    write_register(NRF24L01P_REG_RF_CH, new_rf_ch);
}

void nrf24l01p_set_rf_tx_output_power(output_power dBm)
{
    uint8_t new_rf_setup = read_register(NRF24L01P_REG_RF_SETUP) & 0xF9;
    new_rf_setup |= (dBm << 1);

    write_register(NRF24L01P_REG_RF_SETUP, new_rf_setup);
}

void nrf24l01p_set_rf_air_data_rate(air_data_rate bps)
{
    // Set value to 0
    uint8_t new_rf_setup = read_register(NRF24L01P_REG_RF_SETUP) & 0xD7;
    
    switch(bps)
    {
        case _1Mbps: 
            break;
        case _2Mbps: 
            new_rf_setup |= 1 << 3;
            break;
        case _250kbps:
            new_rf_setup |= 1 << 5;
            break;
    }
    write_register(NRF24L01P_REG_RF_SETUP, new_rf_setup);
}
