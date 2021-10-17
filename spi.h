#pragma once

#include "stddef.h"
#include "stm32f103_md.h"

// master, read/write, 8-bit transfers, pol/pha = 00
// SPI1 will use DMA1 Channel 2/3,  SPI2 will use DMA1 Channel 4/5
void spi_init(struct SPI_Type *spi, uint8_t div);

// call after init to disable use of the nss pin
void spi_nonss(struct SPI_Type *spi);

// users must provide 2 external callback functions to set/clear the proper slave function select.
// slave_func == 0 should de-assert the chip select pin for that slave.
// if spi channel has a single slave, can use the NSS pin and this can be a dummy function.
extern void spi1_slave_select(int slave_func);
extern void spi2_slave_select(int slave_func);

// Enqueues a xmit on this spi's queue.
// The spi will dma/irq the buffer out, after which the _deq function will return func, len and buf with the exchanged
// contents. The buffer contents should not be modified between the _end and _deq calls. Returns 0 on succesful
// enqueueing, -1 if the queue is full.
int spi_xmit_enq(struct SPI_Type *spi, int slave_func, size_t len, uint8_t *buf);

// Polls the spi queue for finished xmits and dequeues if one is available.
// If no xmits are ready, sets *slave_func to 0, leaves len and buf unmodified and returns 0.
// If at least 1 xmit is read sets *slave_func, *len and **buf set to the values enqueued by the corresponding _enq
// call, and returns 0 for a succesful transfer or a non-zero error code corresponding to spi->SR & 0xff
uint16_t spi_xmit_deq(struct SPI_Type *spi, int *slave_func, size_t *len, uint8_t **buf);

// spi_wait blocks until the spi's queue has at least one finished xmit to deq, or the queue is idle.
void spi_wait(struct SPI_Type *spi);

void spi_dump(struct SPI_Type *spi);