#include <stdio.h>
#include <string.h>
#include "tremo_system.h"
#include "radio.h"
#include "tremo_delay.h"
#include "tremo_uart.h"
#include "tremo_gpio.h"
#include "tremo_rcc.h"

#define RF_FREQUENCY 865000000  // LoRa Frequency for India
#define TX_OUTPUT_POWER 14      // dBm
#define LORA_BANDWIDTH 0        // 125 kHz
#define LORA_SPREADING_FACTOR 7 // SF7
#define LORA_CODINGRATE 1       // 4/5
#define LORA_PREAMBLE_LENGTH 8
#define LORA_SYMBOL_TIMEOUT 0
#define LORA_FIX_LENGTH_PAYLOAD_ON false
#define LORA_IQ_INVERSION_ON false
#define TX_INTERVAL 5000  // 5 seconds interval

#define UART_INSTANCE UART0
#define UART_BAUDRATE 256000 

#define MULTI 1 // change value to 0 for single-target detection

//single-target detection command
uint8_t Single_Target_Detection_CMD[12] = {0xFD, 0xFC, 0xFB, 0xFA, 0x02, 0x00, 0x80, 0x00, 0x04, 0x03, 0x02, 0x01};
// Multi-Target Detection Command
uint8_t Multi_Target_Detection_CMD[12] = {0xFD, 0xFC, 0xFB, 0xFA, 0x02, 0x00, 0x90, 0x00, 0x04, 0x03, 0x02, 0x01};


#define HEADER_1 0xAA
#define RX_BUFFER_SIZE 30  // Frame size is fixed at 32 bytes

static RadioEvents_t RadioEvents;
uint8_t rx_buffer[RX_BUFFER_SIZE];
volatile uint8_t rx_index = 0;
volatile uint8_t frame_received = 0;
volatile uint8_t frame_started = 0; // Flag to track frame start
volatile uint32_t last_byte_time = 0; // To track time since last byte
volatile uint32_t system_time = 0; // Simple system time counter

// Function to reset the UART reception state
void reset_uart_reception() {
    frame_started = 0;
    rx_index = 0;
}

void UART0_IRQHandler(void) {
    if (uart_get_interrupt_status(UART_INSTANCE, UART_INTERRUPT_RX_DONE)) {
        uint8_t received_byte = uart_receive_data(UART_INSTANCE);
        last_byte_time = system_time; // Update last byte time
        
        // Check for header sequence
        if (!frame_started) {
            if (received_byte == HEADER_1) {
                rx_index = 0;
                rx_buffer[rx_index++] = received_byte;
                frame_started = 1;
            }
        } else {
            // Continue collecting frame data
            if (rx_index < RX_BUFFER_SIZE) {
                rx_buffer[rx_index++] = received_byte;
                
                // Check for complete frame
                if (rx_index == RX_BUFFER_SIZE) {
                    frame_received = 1;
                    frame_started = 0;
                    rx_index = 0;
                }
            } else {
                // Buffer overflow protection
                reset_uart_reception();
            }
        }
        
        uart_clear_interrupt(UART_INSTANCE, UART_INTERRUPT_RX_DONE);
    }
}


void uart_send_data_array(uint8_t *data, uint8_t length)
{
    printf("\n\r");
    for (uint8_t i = 0; i < length; i++)
    {
        uart_send_data(UART_INSTANCE, data[i]);
        while (uart_get_flag_status(UART_INSTANCE, UART_FLAG_TX_FIFO_EMPTY) == RESET);
    }
    
    // Ensure transmission is completed before proceeding
    // while (uart_get_flag_status(UART_INSTANCE, UART_FLAG_BUSY) == SET);
    
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
    uart_config_interrupt(UART_INSTANCE, UART_INTERRUPT_RX_DONE, ENABLE);
    NVIC_EnableIRQ(UART0_IRQn);
}

// Function to periodically request new data from the sensor
void request_new_data(void) {
    #if MULTI
    uart_send_data_array(Multi_Target_Detection_CMD, sizeof(Multi_Target_Detection_CMD));
    #else
    uart_send_data_array(Single_Target_Detection_CMD, sizeof(Single_Target_Detection_CMD));
    #endif
}

static RadioEvents_t RadioEvents;

void OnTxDone(void)
{
    printf("\n\rMessage Sent!");
    delay_ms(TX_INTERVAL);
    Radio.Send(rx_buffer, RX_BUFFER_SIZE);
}

void OnTxTimeout(void)
{
    printf("TX Timeout - Retrying...\n");
    Radio.Send(rx_buffer, RX_BUFFER_SIZE);
}

void app_start(void)
{
    printf("LoRa TX Device: Initializing...\n");

    rcc_enable_peripheral_clk(RCC_PERIPHERAL_UART0, true);
    rcc_enable_peripheral_clk(RCC_PERIPHERAL_GPIOB, true);
    gpio_set_iomux(GPIOB, GPIO_PIN_0, 1);
    gpio_set_iomux(GPIOB, GPIO_PIN_1, 1);

    my_uart_init();
    
    request_new_data();

    uint32_t last_request_time = 0;
    uint32_t request_interval = 300; // Request new data every 500ms

    RadioEvents.TxDone = OnTxDone;
    RadioEvents.TxTimeout = OnTxTimeout;
    
    Radio.Init(&RadioEvents);
    Radio.SetChannel(RF_FREQUENCY);
    Radio.SetTxConfig(MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                      LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                      LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                      true, 0, 0, LORA_IQ_INVERSION_ON, 3000);
                      
    while (1)
    {
        delay_ms(1);
        system_time++; // Increment our system time counter
        
        // Periodically request new data to ensure fresh frames
        if (system_time - last_request_time >= request_interval) {
            request_new_data();
            last_request_time = system_time;
        }
        if (frame_received)
        {
            frame_received = 0;
            printf("\n\rSending Data: ");
            for (int i = 0; i < RX_BUFFER_SIZE; i++)
            {
                printf("%02X ", rx_buffer[i]);
            }
            printf("\n");
            
            Radio.Send(rx_buffer, RX_BUFFER_SIZE);

        }
        // delay_ms(10);
        // Timeout mechanism to recover from partial frames
        if (frame_started && (system_time - last_byte_time > 100)) {
            printf("Frame reception timeout - resetting\r\n");
            reset_uart_reception();
        }
    }
}
