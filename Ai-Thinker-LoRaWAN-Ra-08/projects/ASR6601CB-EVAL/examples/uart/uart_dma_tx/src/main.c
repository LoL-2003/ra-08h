#include <stdio.h>
#include <stdarg.h>
#include "tremo_uart.h"
#include "tremo_dma.h"
#include "tremo_dma_handshake.h"
#include "tremo_gpio.h"
#include "tremo_rcc.h"
#include <stdint.h>

#define ENABLE_RX 1
#define ENABLE_TX 0
#define FRAME_SIZE 32  // Fixed UART frame size
#define HEADER_1 0xAA
#define HEADER_2 0xFF
#define TARGET_COUNT_INDEX 2
#define FOOTER_1 0x55
#define FOOTER_2 0xCC

static volatile int is_rx_uart_dma_done = 0;
static volatile int is_tx_uart_dma_done = 0;

#if ENABLE_RX
static uint8_t rx_buf[12] = {0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x3A, 0x3B};
#endif

#if ENABLE_TX
static uint8_t tx_buf[10] = {0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39};
#endif

//single-Target Detection Commands
uint8_t Single_Target_Detection_CMD[12] = {0xFD, 0xFC, 0xFB, 0xFA, 0x02, 0x00, 0x80, 0x00, 0x04, 0x03, 0x02, 0x01};
// Multi-Target Detection Command
uint8_t Multi_Target_Detection_CMD[12] = {0xFD, 0xFC, 0xFB, 0xFA, 0x02, 0x00, 0x90, 0x00, 0x04, 0x03, 0x02, 0x01};

#if ENABLE_RX
void rx_uart_dma_irq_handle(void)
{
    is_rx_uart_dma_done = 1;
}
#endif

#if ENABLE_TX
void tx_uart_dma_irq_handle(void)
{
    is_tx_uart_dma_done = 1;
}
#endif

#if ENABLE_RX
void uart_dma_rx_init(uart_t* uartx)
{
    dma_dev.dma_num    = 0;
    dma_dev.ch         = 0;
    dma_dev.mode       = P2M_MODE;
    dma_dev.src        = (uint32_t) & (uartx->DR);
    dma_dev.dest       = (uint32_t)(rx_buf);
    dma_dev.priv       = rx_uart_dma_irq_handle;
    dma_dev.data_width = 0;
    dma_dev.block_size = 12;
    dma_dev.src_msize  = 1;
    dma_dev.dest_msize = 1;
    dma_dev.handshake  = DMA_HANDSHAKE_UART_0_RX;

    dma_init(&dma_dev);
    dma_ch_enable(dma_dev.dma_num, 0);
    uart_dma_config(uartx, UART_DMA_REQ_RX, true);
}
#endif

#if ENABLE_TX
void uart_dma_tx_init(uart_t* uartx)
{
    dma_dev.dma_num    = 0;
    dma_dev.ch         = 0;
    dma_dev.mode       = M2P_MODE;
    dma_dev.src        = (uint32_t)(tx_buf);
    dma_dev.dest       = (uint32_t) & (uartx->DR);
    dma_dev.priv       = tx_uart_dma_irq_handle;
    dma_dev.data_width = 0;
    dma_dev.block_size = 10;
    dma_dev.src_msize  = 1;
    dma_dev.dest_msize = 1;
    dma_dev.handshake  = DMA_HANDSHAKE_UART_0_TX;

    dma_init(&dma_dev);
    dma_ch_enable(dma_dev.dma_num, 0);
    uart_dma_config(uartx, UART_DMA_REQ_TX, true);
}
#endif

void process_uart_data(uint8_t *uart_data)
{
    if (uart_data[0] != HEADER_1 || uart_data[1] != HEADER_2 || uart_data[FRAME_SIZE - 2] != FOOTER_1 || uart_data[FRAME_SIZE - 1] != FOOTER_2) {
        printf("Error: Invalid UART Frame!\n");
        return;
    }

    int num_targets = uart_data[TARGET_COUNT_INDEX];
    if (num_targets > 3) {
        printf("Error: Too many targets detected!\n");
        return;
    }

    for (int i = 0; i < num_targets; i++) {
        int offset = 4 + (i * 8);

        uint16_t x_raw = uart_data[offset] | (uart_data[offset + 1] << 8);
        uint16_t y_raw = uart_data[offset + 2] | (uart_data[offset + 3] << 8);
        uint16_t speed_raw = uart_data[offset + 4] | (uart_data[offset + 5] << 8);
        uint16_t dist_raw = uart_data[offset + 6] | (uart_data[offset + 7] << 8);

        int16_t x_coord = 0 - x_raw;
        int16_t y_coord = y_raw - 32768;
        int16_t speed = 0 - speed_raw;
        uint16_t distance = dist_raw;

        const char* movement = (speed < 0) ? "Approaching" : "Moving Away";

        printf("\nTarget %d X Coordinate: %d mm\n", i + 1, x_coord);
        printf("Target %d Y Coordinate: %d mm\n", i + 1, y_coord);
        printf("Target %d Speed: %d cm/s (%s)\n", i + 1, speed, movement);
        printf("Target %d Distance Resolution: %u mm\n", i + 1, distance);
    }
}

int main(void)
{
    uart_t* uartx = UART0;
    dma_dev_t dma_dev;
    rcc_enable_peripheral_clk(RCC_PERIPHERAL_UART0, true);
    rcc_enable_peripheral_clk(RCC_PERIPHERAL_GPIOB, true);
    rcc_enable_peripheral_clk(RCC_PERIPHERAL_SYSCFG, true);
    rcc_enable_peripheral_clk(RCC_PERIPHERAL_DMA0, true);

    gpio_set_iomux(GPIOB, GPIO_PIN_0, 1);
    gpio_set_iomux(GPIOB, GPIO_PIN_1, 1);

    uart_config_t uart_init_struct;
    uart_config_init(&uart_init_struct);
    uart_init_struct.fifo_mode = ENABLE;
    uart_init(uartx, &uart_init_struct);
    uart_cmd(uartx, ENABLE);
    
#if ENABLE_RX   
    uart_set_rx_fifo_threshold(uartx, UART_IFLS_RX_1_4);
    uart_dma_rx_init(uartx);
    while (!is_rx_uart_dma_done) { }
    process_uart_data(rx_buf);
#endif

#if ENABLE_TX
    uart_dma_tx_init(uartx);
    while (!is_tx_uart_dma_done) { }
#endif

    while (1) { }
}

#ifdef USE_FULL_ASSERT
void assert_failed(void* file, uint32_t line)
{
    (void)file;
    (void)line;
    while (1) { }
}
#endif