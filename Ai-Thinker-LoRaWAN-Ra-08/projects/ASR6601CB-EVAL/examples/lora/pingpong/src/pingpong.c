

#define RECEIVE 1

#if RECEIVE 

#include <stdio.h>
#include <string.h>
#include "tremo_system.h"
#include "radio.h"
#include<math.h>

#define RF_FREQUENCY 865000000  // LoRa Frequency for India
#define LORA_BANDWIDTH 0        // 125 kHz
#define LORA_SPREADING_FACTOR 7 // SF7
#define LORA_CODINGRATE 1       // 4/5
#define LORA_PREAMBLE_LENGTH 8
#define LORA_SYMBOL_TIMEOUT 0
#define LORA_FIX_LENGTH_PAYLOAD_ON false
#define LORA_IQ_INVERSION_ON false
#define RX_TIMEOUT_VALUE 10000  // 10 seconds

#define RX_BUFFER_SIZE 32  // Frame size is fixed at 32 bytes
#define HEADER_1 0xAA
#define HEADER_2 0xFF
#define HEADER_3 0x03
#define HEADER_4 0x00
#define FOOTER_1 0x55
#define FOOTER_2 0xCC
#define NUM_TARGETS 3 // Number of targets to process

static RadioEvents_t RadioEvents;
uint8_t rx_buffer[32];

int validate_frame()
{
   return (rx_buffer[0] == HEADER_1 && rx_buffer[1] == HEADER_2 && rx_buffer[2] == HEADER_3 &&rx_buffer[3] == HEADER_4 &&
           rx_buffer[RX_BUFFER_SIZE - 2] == FOOTER_1 && rx_buffer[RX_BUFFER_SIZE - 1] == FOOTER_2);
}

void processing_data(void)
{


   // Process each target's data (Each target occupies 8 bytes)
   for (int i = 0; i < NUM_TARGETS; i++)
   {
       int offset = 4 + (i * 8); // Offset for each target in the frame


       uint16_t x_raw = rx_buffer[offset] | (rx_buffer[offset + 1] << 8);
       uint16_t y_raw = rx_buffer[offset + 2] | (rx_buffer[offset + 3] << 8);
       uint16_t speed_raw = rx_buffer[offset + 4] | (rx_buffer[offset + 5] << 8);
       uint16_t dist_raw = rx_buffer[offset + 6] | (rx_buffer[offset + 7] << 8);


       // Apply correct formula for sign handling
       int16_t x_coord = 0 - x_raw;                     // X_final = 0 - X_raw
       int16_t y_coord = y_raw - 32768;                 // Y_final = raw_Y - 2^15
       int16_t speed = 0 - speed_raw;                   // Speed_final = 0 - Speed_raw
       uint16_t distance = dist_raw;                    // Distance remains unchanged
       double_t target_distance = (sqrt(pow(x_coord, 2) + pow(y_coord, 2)))/10; // distance calculation


       // Speed Interpretation
       const char *movement = (speed < 0) ? "Approaching" : "Moving Away";
       if(y_coord == - 32768)
       {
           y_coord = 0; // Handle special case for Y coordinate
       }


       // Print Results
       if(speed == 0 && x_coord == 0 && y_coord == 0)
       {
           continue; // Skip if all values are zero
       }
       else
       {
           if(speed ==0)
           {
               movement = "Stationary"; // Handle stationary case
           }
           printf("\r\nTarget %d X Coordinate: %d cm\r\n", i + 1, x_coord/10);
           printf("Target %d Y Coordinate: %d cm\r\n", i + 1, y_coord/10);
           printf("Target %d Speed: %d cm/s (%s)\r\n", i + 1, speed/10, movement);
           printf("Target %d Distance Resolution: %u cm\r\n", i + 1, distance/10);
           printf("Target %d Distance: %f cm\r\n", i + 1, target_distance);
       }
   }
}

// Custom square root function
double_t sqrt(double_t x) {
    if (x < 0) return -1;  // Error case for negative numbers
    if (x == 0) return 0;
   
    double_t guess = x / 2.0;
    double_t epsilon = 0.0000001;  // Precision
   
    while (1) {
        double_t new_guess = (guess + x / guess) / 2.0;
        if (guess - new_guess < epsilon && new_guess - guess < epsilon) {
            return new_guess;
        }
        guess = new_guess;
    }
 }

void PrintReceivedBuffer(uint8_t *buffer, uint16_t size)
{
    printf("\n\rReceived (HEX): ");
    for (uint8_t i = 0; i < size; i++)
    {
        printf("%02X ", buffer[i]);  
    }
    printf("\n\r");
}

void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr)
{
    Radio.Sleep();
    memcpy(rx_buffer, payload, size);
    PrintReceivedBuffer(rx_buffer, size);
    // printf("RSSI: %d dBm, SNR: %d dB\n", rssi, snr);
    Radio.Rx(RX_TIMEOUT_VALUE);  // Restart RX mode
}

void OnRxTimeout(void)
{
    printf("RX Timeout - Waiting...\n");
    Radio.Rx(RX_TIMEOUT_VALUE);
}

void OnRxError(void)
{
    printf("RX Error - Restarting RX...\n");
    Radio.Rx(RX_TIMEOUT_VALUE);
}

void app_start(void)
{
    printf("LoRa RX Device: Listening...\n");

    RadioEvents.RxDone = OnRxDone;
    RadioEvents.RxTimeout = OnRxTimeout;
    RadioEvents.RxError = OnRxError;

    Radio.Init(&RadioEvents);
    Radio.SetChannel(RF_FREQUENCY);
    Radio.SetRxConfig(MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                      LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                      LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
                      0, true, 0, 0, LORA_IQ_INVERSION_ON, true);

    Radio.Rx(RX_TIMEOUT_VALUE);

    while (1)
    {
        Radio.IrqProcess();
        if(validate_frame())
        {
            processing_data(); // Process the received data
        }
    }
}

#else

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
#define RX_BUFFER_SIZE 32  // Frame size is fixed at 32 bytes

static RadioEvents_t RadioEvents;
uint8_t rx_buffer[RX_BUFFER_SIZE];
volatile uint8_t rx_index = 0;
volatile uint8_t frame_received = 0;
volatile uint8_t frame_started = 0; // Flag to track frame start


void UART0_IRQHandler(void)
{
   if (uart_get_interrupt_status(UART_INSTANCE, UART_INTERRUPT_RX_DONE))
   {
       uint8_t received_byte = uart_receive_data(UART_INSTANCE);


       if (!frame_started) // Detect frame start
       {
           if (received_byte == HEADER_1) // Header detected, start frame capture
           {
               rx_index = 0;
               rx_buffer[rx_index++] = received_byte;
               frame_started = 1;
           }
       }
       else
       {
           if (rx_index < RX_BUFFER_SIZE)
           {
               rx_buffer[rx_index++] = received_byte;
           }
           if (rx_index >= RX_BUFFER_SIZE) // Frame complete
           {
               frame_received = 1;
               frame_started = 0;
               rx_index = 0; // Reset index for next frame
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
        printf("%02X",data[i]);
        uart_send_data(UART_INSTANCE, data[i]);
        while (uart_get_flag_status(UART_INSTANCE, UART_FLAG_TX_FIFO_EMPTY) == RESET);
    }
    
    // Ensure transmission is completed before proceeding
    while (uart_get_flag_status(UART_INSTANCE, UART_FLAG_BUSY) == SET);
    
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
    #if MULTI
    uart_send_data_array(Multi_Target_Detection_CMD, sizeof(Multi_Target_Detection_CMD));
    #else
    uart_send_data_array(Single_Target_Detection_CMD, sizeof(Single_Target_Detection_CMD));
    #endif

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
        delay_ms(10);
    }
}

#endif