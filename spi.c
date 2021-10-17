#include "spi.h"

static inline int spi_index(struct SPI_Type *spi) {
    if (spi == &SPI1)
        return 0;
    if (spi == &SPI2)
        return 1;

    for (;;)
        __NOP();  // hang
    return 0;
}

static struct {
    struct DMA_Channel_Type *rxchan;
    struct DMA_Channel_Type *txchan;
    enum IRQn_Type rx_dma_irqn;
} spi_dma_info[2] = {
        {&DMA1_Channel2, &DMA1_Channel3, DMA1_Channel2_IRQn},
        {&DMA1_Channel4, &DMA1_Channel5, DMA1_Channel4_IRQn},
};

struct XmitItem {
    uint16_t status;
    int slave_func;  // tag to remember what this was for after xmit is done
    size_t len;
    uint8_t *buf;
};

// a SpiXmitQ is a ringbuffer with one element between head and tail being currently transmitted by the SPI unit
struct SpiXmitQ {
    volatile uint16_t head;
    volatile uint16_t curr;
    volatile uint16_t tail;
    struct XmitItem xmit[8];  // must be power of 2
};

#define NELEM(x) (sizeof((x)) / sizeof((x)[0]))

// head >= curr >= tail
// empty:  head == curr == tail == 0
// push one at xmit[0]: head == 1, curr == 0 (running), tail == 0 -> deq_empty because curr=tail
// xmit[0] done:   head == 1, curr = 1 (stop) (enq empty because head == curr), tail = 0: 1 available fro deq
// head == tail + 7 -> full
static inline int enq_empty(struct SpiXmitQ *q) {
    return q->head == q->curr;
}  // no more to xmit
static inline int deq_empty(struct SpiXmitQ *q) {
    return q->curr == q->tail;
}  // no more to dequeue
static inline uint16_t enq_free(struct SpiXmitQ *q) {
    return NELEM(q->xmit) - (q->head - q->tail) - 1;
}
static inline int enq_full(struct SpiXmitQ *q) {
    return enq_free(q) < 1;
}
static inline void clearq(struct SpiXmitQ *q) {
    q->tail = q->curr = q->head;
}

static struct SpiXmitQ spiq[2];

// valid values for div are 0...7 for divisor f/(2^(div+1))
void spi_init(struct SPI_Type *spi, uint8_t div) {
    spi->CR1 &= ~SPI_CR1_SPE;

    // 8 bit master mode
    spi->CR1 = SPI_CR1_MSTR | ((div & 0x7) << 3);
    spi->CR2 = SPI_CR2_SSOE | SPI_CR2_RXDMAEN | SPI_CR2_TXDMAEN;

    int idx = spi_index(spi);
    clearq(spiq + idx);

    NVIC_EnableIRQ(spi_dma_info[idx].rx_dma_irqn);
}

// disable use of the NSS pin (requires the slave is hooked up to a different ChipSelect pin)
void spi_nonss(struct SPI_Type *spi) {
    spi->CR1 |= SPI_CR1_SSM | SPI_CR1_SSI;
    spi->CR2 &= ~SPI_CR2_SSOE;
}

inline static void spidmamove(struct SPI_Type *spi, struct DMA_Channel_Type *dmac, uint8_t *mem, size_t len) {
    dmac->CCR = DMA_CCR1_MINC;  // all defaults, disable, except the Memory Increment flag
    dmac->CPAR = (uint32_t)&spi->DR;
    dmac->CMAR = (uint32_t)mem;
    dmac->CNDTR = len;
}

inline static void startxmit(struct SPI_Type *spi) {
    int idx = spi_index(spi);
    struct SpiXmitQ *q = spiq + idx;
    struct XmitItem *x = q->xmit + (q->curr % NELEM(q->xmit));
    switch (idx) {
    case 0:
        spi1_slave_select(x->slave_func);
        break;
    case 1:
        spi2_slave_select(x->slave_func);
        break;
    }

    spidmamove(spi, spi_dma_info[idx].rxchan, x->buf, x->len);
    spi_dma_info[idx].rxchan->CCR |= DMA_CCR1_TEIE | DMA_CCR1_TCIE | DMA_CCR1_EN;

    spidmamove(spi, spi_dma_info[idx].txchan, x->buf, x->len);
    spi_dma_info[idx].txchan->CCR |= DMA_CCR1_TEIE | DMA_CCR1_DIR | DMA_CCR1_EN;

    spi->CR1 |= SPI_CR1_SPE;
}

int spi_xmit_enq(struct SPI_Type *spi, int slave_func, size_t len, uint8_t *buf) {
    int idx = spi_index(spi);
    struct SpiXmitQ *q = spiq + idx;
    if (enq_full(q)) {
        return -1;
    }

    struct XmitItem *x = q->xmit + (q->head % NELEM(q->xmit));
    x->status = 0;
    x->slave_func = slave_func;
    x->len = len;
    x->buf = buf;

    q->head++;

    // NVIC_DisableIRQ(spi_dma_info[idx].rx_dma_irqn);
    if ((spi->CR1 & SPI_CR1_SPE) == 0) {
        startxmit(spi);
    }
    // NVIC_EnableIRQ(spi_dma_info[idx].rx_dma_irqn);

    return 0;
}

// SPI1 RX DMA
void DMA1_Channel2_IRQ_Handler(void) {
    DMA1.IFCR = DMA1.ISR & 0x00f0;
    spiq[0].xmit[spiq[0].curr % NELEM(spiq[0].xmit)].status = SPI1.SR & 0xf0;
    SPI1.CR1 &= ~SPI_CR1_SPE;
    spi1_slave_select(0);
    spiq[0].curr++;
    if (!enq_empty(spiq + 0)) {
        startxmit(&SPI1);
    }
}

// SPI2 RX DMA
void DMA1_Channel4_IRQ_Handler(void) {
    DMA1.IFCR = DMA1.ISR & 0xf000;
    spiq[1].xmit[spiq[1].curr % NELEM(spiq[1].xmit)].status = SPI2.SR & 0xf0;
    SPI2.CR1 &= ~SPI_CR1_SPE;
    spi2_slave_select(0);
    spiq[1].curr++;
    if (!enq_empty(spiq + 1)) {
        startxmit(&SPI2);
    }
}

// Polls the spi queue for finished xmits and dequeues if one is available.
// If no xmits are ready, sets *slave_func to 0, leaves len and buf unmodified and returns 0.
// If at least 1 xmit is read sets *slave_func, *len and **buf set to the values enqueued by the corresponding _enq
// call, and returns 0 for a succesful transfer or a non-zero error code corresponding to spi->SR & 0xff
uint16_t spi_xmit_deq(struct SPI_Type *spi, int *slave_func, size_t *len, uint8_t **buf) {
    struct SpiXmitQ *q = spiq + spi_index(spi);
    if (deq_empty(q)) {
        *slave_func = 0;
        return 0;
    }
    struct XmitItem *x = q->xmit + (q->tail % NELEM(q->xmit));
    if (slave_func != NULL){
        *slave_func = x->slave_func;
    }
    if (len) {
        *len = x->len;
    }
    if (buf) {
        *buf = x->buf;
    }
    uint16_t s = x->status;
    q->tail++;
    return s;
}

// idle == not enabled. start_xmit sets enable, dma irq handler clears it.
static inline int spi_idle(struct SPI_Type *spi) {
    return (spi->CR1 & SPI_CR1_SPE) ? 0 : 1;
}

// spi_wait blocks until the spi's queue has at least one finished xmit to deq, or the queue is idle.
void spi_wait(struct SPI_Type *spi) {
    struct SpiXmitQ *q = spiq + spi_index(spi);
    while (deq_empty(q)) {
        if (spi_idle(spi) && enq_empty(q))
            return;
        __WFI();
    }
}
