#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include "tremo_uart.h"
#include "tremo_dma.h"
#include "tremo_dma_handshake.h"
#include "tremo_gpio.h"
#include "tremo_rcc.h"

#define CONFIG_DEBUG_UART UART0

#define BUFFER_SIZE 64

// DMA RX completion flag and buffer
static volatile int is_rx_uart_dma_done = 0;
static uint8_t buf[BUFFER_SIZE]; // Buffer to store received data

// DMA RX interrupt handler
void rx_uart_dma_irq_handle(void)
{
    is_rx_uart_dma_done = 1;
}

// UART initialization for logging
void uart_log_init(void)
{
    // UART0 configuration (PA0: TX, PA1: RX)
    gpio_set_iomux(GPIOA, GPIO_PIN_0, 1); // TX
    gpio_set_iomux(GPIOA, GPIO_PIN_1, 1); // RX

    /* UART config struct init */
    uart_config_t uart_config;
    uart_config_init(&uart_config);

    uart_config.baudrate = UART_BAUDRATE_115200;
    uart_config.fifo_mode = ENABLE; // Enable FIFO for DMA
    uart_set_rx_fifo_threshold(CONFIG_DEBUG_UART, UART_IFLS_RX_1_4);

    uart_init(CONFIG_DEBUG_UART, &uart_config);
    uart_cmd(CONFIG_DEBUG_UART, ENABLE);
}

// Custom putchar to redirect printf to UART
int putchar(int c)
{
    uart_send_data(CONFIG_DEBUG_UART, (uint8_t)c);
    return c;
}

int main(void)
{
    uart_t* uartx = CONFIG_DEBUG_UART;
    dma_dev_t dma_dev;

    // Enable peripheral clocks
    rcc_enable_peripheral_clk(RCC_PERIPHERAL_UART0, true);
    rcc_enable_peripheral_clk(RCC_PERIPHERAL_GPIOA, true);
    rcc_enable_peripheral_clk(RCC_PERIPHERAL_SYSCFG, true);
    rcc_enable_peripheral_clk(RCC_PERIPHERAL_DMA0, true);
    rcc_enable_peripheral_clk(RCC_PERIPHERAL_DMA1, true);

    // Initialize UART
    uart_log_init();

    // Configure DMA
    dma_deinit(0); // Reset DMA channel
    dma_dev.dma_num    = 0;
    dma_dev.ch         = 0;
    dma_dev.mode       = P2M_MODE; // Peripheral to Memory
    dma_dev.src        = (uint32_t) & (uartx->DR); // UART Data Register
    dma_dev.dest       = (uint32_t)(buf); // Destination buffer
    dma_dev.priv       = rx_uart_dma_irq_handle; // Callback
    dma_dev.data_width = DMA_DATA_WIDTH_BYTE; // Corrected for 8-bit data
    dma_dev.block_size = BUFFER_SIZE; // Receive BUFFER_SIZE bytes
    dma_dev.src_msize  = DMA_MSIZE_1; // Corrected
    dma_dev.dest_msize = DMA_MSIZE_1; // Corrected
    dma_dev.handshake  = DMA_HANDSHAKE_UART_0_RX; // UART0 RX handshake

    dma_init(&dma_dev);

    // Enable DMA interrupt
    dma_interrupt_enable(0, 0);
    
    while (1) {
        // Clear buffer before receiving new data
        memset(buf, 0, sizeof(buf));

        // Reset DMA completion flag before enabling DMA
        is_rx_uart_dma_done = 0;

        // Enable DMA channel for new reception
        dma_ch_enable(dma_dev.dma_num, dma_dev.ch);
        uart_dma_config(uartx, UART_DMA_REQ_RX, true); // Enable UART RX DMA

        printf("Waiting for UART RX data...\r\n");
        while (is_rx_uart_dma_done == 0) {
            ;
        }

        // Print received buffer contents
        printf("Received data: ");
        for (uint8_t i = 0; i < BUFFER_SIZE; i++) {
            printf("%c", buf[i]); // Print as characters
        }
        printf("\r\n");

        // Ensure `is_rx_uart_dma_done` is reset only after printing
        is_rx_uart_dma_done = 0;
    }
}

#ifdef USE_FULL_ASSERT
void assert_failed(void* file, uint32_t line)
{
    (void)file;
    (void)line;

    while (1) { }
}
#endif
