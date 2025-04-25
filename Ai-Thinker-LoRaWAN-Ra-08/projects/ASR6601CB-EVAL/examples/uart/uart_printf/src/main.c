#include <stdio.h>
#include "tremo_uart.h"
#include "tremo_gpio.h"
#include "tremo_rcc.h"
#include "tremo_delay.h"

#define UART_INSTANCE UART0
#define UART_BAUDRATE 256000

// Multi-Target Detection Command
uint8_t Multi_Target_Detection_CMD[12] = {0xFD, 0xFC, 0xFB, 0xFA, 0x02, 0x00, 0x90, 0x00, 0x04, 0x03, 0x02, 0x01};

void uart_send_data_array(uint8_t *data, uint8_t length)
{
    for (uint8_t i = 0; i < length; i++)
    {
        while (uart_get_flag_status(UART_INSTANCE, UART_FLAG_TX_FIFO_EMPTY) == RESET);
        uart_send_data(UART_INSTANCE, data[i]);
        while (uart_get_flag_status(UART_INSTANCE, UART_FLAG_BUSY) == SET); // Ensure full transmission
    }
}

void my_uart_init(void)
{
    uart_config_t uart_config;
    uart_config_init(&uart_config);
    uart_config.baudrate = UART_BAUDRATE;
    uart_config.data_width = UART_DATA_WIDTH_8;
    uart_config.parity = UART_PARITY_NO;
    uart_config.stop_bits = UART_STOP_BITS_1;
    uart_config.fifo_mode = ENABLE;

    uart_init(UART_INSTANCE, &uart_config);
    uart_cmd(UART_INSTANCE, ENABLE);
}

int main(void)
{
    // Enable UART and GPIO clocks
    rcc_enable_peripheral_clk(RCC_PERIPHERAL_UART0, true);
    rcc_enable_peripheral_clk(RCC_PERIPHERAL_GPIOB, true);

    // Set GPIO pins for UART TX/RX
    gpio_set_iomux(GPIOB, GPIO_PIN_0, 1);  // TX
    gpio_set_iomux(GPIOB, GPIO_PIN_1, 1);  // RX

    my_uart_init();

    while (1)
    {
        printf("\nSent Frame: ");
        for (uint8_t i = 0; i < sizeof(Multi_Target_Detection_CMD); i++)
        {
            printf("%02X ", Multi_Target_Detection_CMD[i]); // Print hex values to debug
        }
        printf("\n\r");
        uart_send_data_array(Multi_Target_Detection_CMD, sizeof(Multi_Target_Detection_CMD));
    }

    return 0;
}