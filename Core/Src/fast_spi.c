#include "fast_spi.h"

void Fast_SPI_Transmit (SPI_TypeDef *SPI, uint8_t *data, int size, bool include_crc)
{
	// Transmit bytes over SPI without using HAL.


	if (include_crc) {
		SPI->CR1 &= ~SPI_CR1_CRCEN;
		SPI->CR1 |= SPI_CR1_CRCEN;
	}

	/****************** STEPS *********************
	1. Wait for the TXE bit to set in the Status Register
	2. Write the data to the Data Register
	3. After the data has been transmitted, wait for the BSY bit to reset in Status Register
	4. Clear the Overrun flag by reading DR and SR
	 ***********************************************/
	int i;
	for (i = 0; i < size; ++i)
	{
		// Used to wait for TXE bit to set -> This will indicate that the buffer is empty
		while (!(SPI->SR & SPI_SR_TXE)) {};
		SPI->DR = data[i];  // load the data into the Data Register
	}

	if (include_crc)
	{
		SPI->CR1 |= SPI_CR1_CRCNEXT;
	}


	/*
	 * During discontinuous communications, there is a 2 APB clock period delay between the
	 * write operation to the SPI_DR register and BSY bit setting. As a consequence it is
	 * mandatory to wait first until TXE is set and then until BSY is cleared after writing the last
	 * data.
	 */

	// Used to wait for TXE bit to set -> This will indicate that the buffer is empty
	while (!(SPI->SR & SPI_SR_TXE)) {};
	while ( SPI->SR & SPI_SR_BSY ) {};  // wait for BSY bit to Reset -> This will indicate that SPI is not busy in communication

	//  Clear the Overrun flag by reading DR and SR
	uint8_t temp = SPI->DR;
	temp = SPI->SR;
}
