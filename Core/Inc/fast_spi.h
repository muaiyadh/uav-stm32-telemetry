/*
 * fast_spi.h
 *
 *  Created on: Jul 3, 2023
 *      Author: muaiyadh
 */

#ifndef FAST_SPI_H_
#define FAST_SPI_H_

#include "stm32f407xx.h"
#include <stdbool.h>

void Fast_SPI_Transmit (SPI_TypeDef *SPI, uint8_t *data, int size, bool include_crc);


#endif /* FAST_SPI_H_ */
