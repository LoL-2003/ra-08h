// #include <stdio.h>
// #include "tremo_uart.h"
// #include "tremo_gpio.h"
// #include "tremo_rcc.h"
// #include "tremo_delay.h"
// #include <math.h> 


// #define UART_INSTANCE UART0
// #define RX_BUFFER_SIZE 30  // Frame size is fixed at 30 bytes
// #define HEADER_1 0xAA
// #define HEADER_2 0xFF
// #define HEADER_3 0x03
// #define HEADER_4 0x00
// #define FOOTER_1 0x55
// #define FOOTER_2 0xCC
// #define NUM_TARGETS 3 // Number of targets to process


// #define UART_BAUDRATE 256000 // Baudrate for UART communication
// #define MULTI 1 // change value to 0 for single-target detection


// //single-target detection command
// uint8_t Single_Target_Detection_CMD[12] = {0xFD, 0xFC, 0xFB, 0xFA, 0x02, 0x00, 0x80, 0x00, 0x04, 0x03, 0x02, 0x01};
// // Multi-Target Detection Command
// uint8_t Multi_Target_Detection_CMD[12] = {0xFD, 0xFC, 0xFB, 0xFA, 0x02, 0x00, 0x90, 0x00, 0x04, 0x03, 0x02, 0x01};


// uint8_t rx_buffer[RX_BUFFER_SIZE];
// volatile uint8_t rx_index = 0;
// volatile uint8_t frame_received = 0;
// volatile uint8_t frame_started = 0; // Flag to track frame start




// // Custom square root function
// double_t sqrt(double_t x) {
//    if (x < 0) return -1;  // Error case for negative numbers
//    if (x == 0) return 0;
  
//    double_t guess = x / 2.0;
//    double_t epsilon = 0.0000001;  // Precision
  
//    while (1) {
//        double_t new_guess = (guess + x / guess) / 2.0;
//        if (guess - new_guess < epsilon && new_guess - guess < epsilon) {
//            return new_guess;
//        }
//        guess = new_guess;
//    }
// }




// void UART0_IRQHandler(void)
// {
//    if (uart_get_interrupt_status(UART_INSTANCE, UART_INTERRUPT_RX_DONE))
//    {
//        uint8_t received_byte = uart_receive_data(UART_INSTANCE);


//        if (!frame_started) // Detect frame start
//        {
//            if (received_byte == HEADER_1) // Header detected, start frame capture
//            {
//                rx_index = 0;
//                rx_buffer[rx_index++] = received_byte;
//                frame_started = 1;
//            }
//        }
//        else
//        {
//            if (rx_index < RX_BUFFER_SIZE)
//            {
//                rx_buffer[rx_index++] = received_byte;
//            }
//            if (rx_index >= RX_BUFFER_SIZE) // Frame complete
//            {
//                frame_received = 1;
//                frame_started = 0;
//                rx_index = 0; // Reset index for next frame
//            }
//        }


//        uart_clear_interrupt(UART_INSTANCE, UART_INTERRUPT_RX_DONE);
//    }
// }


// void uart_send_data_array(uint8_t *data, uint8_t length)
// {
//     for (uint8_t i = 0; i < length; i++)
//     {
//         uart_send_data(UART_INSTANCE, data[i]);
//         while (uart_get_flag_status(UART_INSTANCE, UART_FLAG_TX_FIFO_EMPTY) == RESET);
//     }
    
//     // Ensure transmission is completed before proceeding
//     while (uart_get_flag_status(UART_INSTANCE, UART_FLAG_BUSY) == SET);
    
// }

// int validate_frame()
// {
//    return (rx_buffer[0] == HEADER_1 && rx_buffer[1] == HEADER_2 && rx_buffer[2] == HEADER_3 &&rx_buffer[3] == HEADER_4 &&
//            rx_buffer[RX_BUFFER_SIZE - 2] == FOOTER_1 && rx_buffer[RX_BUFFER_SIZE - 1] == FOOTER_2);
// }


// void processing_data(void)
// {


//    // Process each target's data (Each target occupies 8 bytes)
//    for (int i = 0; i < NUM_TARGETS; i++)
//    {
//        int offset = 4 + (i * 8); // Offset for each target in the frame


//        uint16_t x_raw = rx_buffer[offset] | (rx_buffer[offset + 1] << 8);
//        uint16_t y_raw = rx_buffer[offset + 2] | (rx_buffer[offset + 3] << 8);
//        uint16_t speed_raw = rx_buffer[offset + 4] | (rx_buffer[offset + 5] << 8);
//        uint16_t dist_raw = rx_buffer[offset + 6] | (rx_buffer[offset + 7] << 8);


//        // Apply correct formula for sign handling
//        int16_t x_coord = 0 - x_raw;                     // X_final = 0 - X_raw
//        int16_t y_coord = y_raw - 32768;                 // Y_final = raw_Y - 2^15
//        int16_t speed = 0 - speed_raw;                   // Speed_final = 0 - Speed_raw
//        uint16_t distance = dist_raw;                    // Distance remains unchanged
//        double_t target_distance = (sqrt(pow(x_coord, 2) + pow(y_coord, 2)))/10; // distance calculation


//        // Speed Interpretation
//        const char *movement = (speed < 0) ? "Approaching" : "Moving Away";
//        if(y_coord == - 32768)
//        {
//            y_coord = 0; // Handle special case for Y coordinate
//        }


//        // Print Results
//        if(speed == 0 && x_coord == 0 && y_coord == 0)
//        {
//            continue; // Skip if all values are zero
//        }
//        else
//        {
//            if(speed ==0)
//            {
//                movement = "Stationary"; // Handle stationary case
//            }
//            printf("\r\nTarget %d X Coordinate: %d cm\r\n", i + 1, x_coord/10);
//            printf("Target %d Y Coordinate: %d cm\r\n", i + 1, y_coord/10);
//            printf("Target %d Speed: %d cm/s (%s)\r\n", i + 1, speed/10, movement);
//            printf("Target %d Distance Resolution: %u cm\r\n", i + 1, distance/10);
//            printf("Target %d Distance: %f cm\r\n", i + 1, target_distance);
//        }
//    }
// }


// void my_uart_init(void)
// {
//    uart_config_t uart_config;
//    uart_config_init(&uart_config);
//    uart_config.baudrate = UART_BAUDRATE;
//    uart_config.data_width = UART_DATA_WIDTH_8;
//    uart_config.parity = UART_PARITY_NO;
//    uart_config.stop_bits = UART_STOP_BITS_1;
//    uart_config.fifo_mode = ENABLE;


//    uart_init(UART_INSTANCE, &uart_config);
//    uart_cmd(UART_INSTANCE, ENABLE);


//    uart_config_interrupt(UART_INSTANCE, UART_INTERRUPT_RX_DONE, ENABLE);
//    NVIC_EnableIRQ(UART0_IRQn);
// }


// int main(void)
// {
//    rcc_enable_peripheral_clk(RCC_PERIPHERAL_UART0, true);
//    rcc_enable_peripheral_clk(RCC_PERIPHERAL_GPIOB, true);


//    gpio_set_iomux(GPIOB, GPIO_PIN_0, 1);
//    gpio_set_iomux(GPIOB, GPIO_PIN_1, 1);


//    my_uart_init();

//    while (1)
//    {
//        delay_ms(1);
//        if (frame_received)
//        {
//            frame_received = 0; // Reset flag

//            #if MULTI
//            uart_send_data_array(Multi_Target_Detection_CMD, sizeof(Multi_Target_Detection_CMD));
//            #else
//            uart_send_data_array(Single_Target_Detection_CMD, sizeof(Single_Target_Detection_CMD));
//            #endif
//            printf("\r\nValid Frame Received: ");
//            for (uint8_t i = 0; i < RX_BUFFER_SIZE; i++)
//            {
//                printf("%02X ", rx_buffer[i]);
//            }
//            printf("\r\n");


//            processing_data(); // Process the received data
//        }
//    }
//    return 0;
// }

//-------------------------------------------------------------------------------use of 2 uart --------------------------------------------------------------------------------------------


// #include <stdio.h>
// #include <string.h>
// #include <math.h>
// #include "tremo_uart.h"
// #include "tremo_gpio.h"
// #include "tremo_rcc.h"
// #include "tremo_delay.h"

// #define UART0_INSTANCE UART0
// #define UART1_INSTANCE UART1

// #define RX_BUFFER_SIZE 30
// #define HEADER_1 0xAA
// #define HEADER_2 0xFF
// #define HEADER_3 0x03
// #define HEADER_4 0x00
// #define FOOTER_1 0x55
// #define FOOTER_2 0xCC
// #define NUM_TARGETS 3
// #define UART_BAUDRATE 256000
// #define MULTI 1

// uint8_t Single_Target_Detection_CMD[12] = {0xFD, 0xFC, 0xFB, 0xFA, 0x02, 0x00, 0x80, 0x00, 0x04, 0x03, 0x02, 0x01};
// uint8_t Multi_Target_Detection_CMD[12]  = {0xFD, 0xFC, 0xFB, 0xFA, 0x02, 0x00, 0x90, 0x00, 0x04, 0x03, 0x02, 0x01};

// uint8_t uart0_rx_buffer[RX_BUFFER_SIZE];
// uint8_t uart1_rx_buffer[RX_BUFFER_SIZE];
// volatile uint8_t uart0_rx_index = 0;
// volatile uint8_t uart1_rx_index = 0;
// volatile uint8_t uart0_frame_received = 0;
// volatile uint8_t uart1_frame_received = 0;

// void processing_data(int uart_id);

// // Custom square root function
// double_t sqrt(double_t x) {
//     if (x < 0) return -1;  // Error case for negative numbers
//     if (x == 0) return 0;
   
//     double_t guess = x / 2.0;
//     double_t epsilon = 0.0000001;  // Precision
   
//     while (1) {
//         double_t new_guess = (guess + x / guess) / 2.0;
//         if (guess - new_guess < epsilon && new_guess - guess < epsilon) {
//             return new_guess;
//         }
//         guess = new_guess;
//     }
//  }

// int validate_frame(uint8_t *buffer) {
//     return (buffer[0] == HEADER_1 && buffer[1] == HEADER_2 &&
//             buffer[2] == HEADER_3 && buffer[3] == HEADER_4 &&
//             buffer[RX_BUFFER_SIZE - 2] == FOOTER_1 &&
//             buffer[RX_BUFFER_SIZE - 1] == FOOTER_2);
// }

// void UART0_IRQHandler(void) {
//     if (uart_get_interrupt_status(UART0_INSTANCE, UART_INTERRUPT_RX_DONE)) {
//         uart0_rx_buffer[uart0_rx_index++] = uart_receive_data(UART0_INSTANCE);
//         if (uart0_rx_index >= RX_BUFFER_SIZE) {
//             uart0_rx_index = 0;
//             uart0_frame_received = 1;
//         }
//         uart_clear_interrupt(UART0_INSTANCE, UART_INTERRUPT_RX_DONE);
//     }
// }

// void UART1_IRQHandler(void) {
//     if (uart_get_interrupt_status(UART1_INSTANCE, UART_INTERRUPT_RX_DONE)) {
//         uart1_rx_buffer[uart1_rx_index++] = uart_receive_data(UART1_INSTANCE);
//         if (uart1_rx_index >= RX_BUFFER_SIZE) {
//             uart1_rx_index = 0;
//             uart1_frame_received = 1;
//         }
//         uart_clear_interrupt(UART1_INSTANCE, UART_INTERRUPT_RX_DONE);
//     }
// }

// void my_uart_init(void) {
//     uart_config_t uart_config;
//     uart_config_init(&uart_config);
//     uart_config.baudrate = UART_BAUDRATE;
//     uart_config.data_width = UART_DATA_WIDTH_8;
//     uart_config.parity = UART_PARITY_NO;
//     uart_config.stop_bits = UART_STOP_BITS_1;
//     uart_config.fifo_mode = ENABLE;

//     uart_init(UART0_INSTANCE, &uart_config);
//     uart_cmd(UART0_INSTANCE, ENABLE);
//     uart_config_interrupt(UART0_INSTANCE, UART_INTERRUPT_RX_DONE, ENABLE);
//     NVIC_EnableIRQ(UART0_IRQn);

//     uart_init(UART1_INSTANCE, &uart_config);
//     uart_cmd(UART1_INSTANCE, ENABLE);
//     uart_config_interrupt(UART1_INSTANCE, UART_INTERRUPT_RX_DONE, ENABLE);
//     NVIC_EnableIRQ(UART1_IRQn);
// }

// void uart_send_data_array(uint8_t *data, uint8_t length) {
//     for (uint8_t i = 0; i < length; i++) {
//         uart_send_data(UART1_INSTANCE, data[i]);
//         while (uart_get_flag_status(UART1_INSTANCE, UART_FLAG_TX_FIFO_EMPTY) == RESET);
//     }
//     while (uart_get_flag_status(UART1_INSTANCE, UART_FLAG_BUSY) == SET);
// }

// void processing_data(int uart_id) {
//     uint8_t temp_buffer[RX_BUFFER_SIZE];
//     memcpy(temp_buffer, uart_id == 0 ? uart0_rx_buffer : uart1_rx_buffer, RX_BUFFER_SIZE);

//     for (int i = 0; i < NUM_TARGETS; i++) {
//         int offset = 4 + (i * 8);

//         uint16_t x_raw     = temp_buffer[offset] | (temp_buffer[offset + 1] << 8);
//         uint16_t y_raw     = temp_buffer[offset + 2] | (temp_buffer[offset + 3] << 8);
//         uint16_t speed_raw = temp_buffer[offset + 4] | (temp_buffer[offset + 5] << 8);
//         uint16_t dist_raw  = temp_buffer[offset + 6] | (temp_buffer[offset + 7] << 8);

//         int16_t x_coord = -x_raw;
//         int16_t y_coord = y_raw - 32768;
//         int16_t speed = -speed_raw;
//         uint16_t distance = dist_raw;

//         if (y_coord == -32768) y_coord = 0;

//         double target_distance = sqrt(pow(x_coord, 2) + pow(y_coord, 2)) / 10.0;

//         if (speed == 0 && x_coord == 0 && y_coord == 0) continue;

//         const char *movement = (speed == 0) ? "Stationary" : (speed < 0 ? "Approaching" : "Moving Away");

//         printf("\r\nTarget %d:\r\n", i + 1);
//         printf("  X Coordinate: %d cm\r\n", x_coord / 10);
//         printf("  Y Coordinate: %d cm\r\n", y_coord / 10);
//         printf("  Speed: %d cm/s (%s)\r\n", speed / 10, movement);
//         printf("  Distance Resolution: %u cm\r\n", distance / 10);
//         printf("  Calculated Distance: %.2f cm\r\n", target_distance);
//     }
// }

// int main(void) {
//     rcc_enable_peripheral_clk(RCC_PERIPHERAL_UART0, true);
//     rcc_enable_peripheral_clk(RCC_PERIPHERAL_UART1, true);
//     rcc_enable_peripheral_clk(RCC_PERIPHERAL_GPIOA, true);
//     rcc_enable_peripheral_clk(RCC_PERIPHERAL_GPIOB, true);

//     gpio_set_iomux(GPIOA, GPIO_PIN_4, 1); // UART1_RX
//     gpio_set_iomux(GPIOA, GPIO_PIN_5, 1); // UART1_TX
//     gpio_set_iomux(GPIOB, GPIO_PIN_0, 1); // UART0_RX
//     gpio_set_iomux(GPIOB, GPIO_PIN_1, 1); // UART0_TX

//     my_uart_init();

//     while (1) {
//         delay_ms(10);

//         if (uart0_frame_received) {
//             uart0_frame_received = 0;
//             if (validate_frame(uart0_rx_buffer)) {
//                 printf("Valid frame received on UART0:\n\r");
//                 for (uint8_t i = 0; i < RX_BUFFER_SIZE; i++) printf("%02X ", uart0_rx_buffer[i]);
//                 printf("\n\r");
//                 processing_data(0);
//             }
//         }

//         if (uart1_frame_received) {
//             uart1_frame_received = 0;
//             if (validate_frame(uart1_rx_buffer)) {
//                 printf("Valid frame received on UART1:\n\r");
//                 for (uint8_t i = 0; i < RX_BUFFER_SIZE; i++) printf("%02X ", uart1_rx_buffer[i]);
//                 printf("\n\r");
//                 processing_data(1);
//             }
//         }
//     }

//     return 0;
// }





//--------------------------------------------------------------------- use of 3 uart ---------------------------------------------------------------------------------------------

#include <stdio.h>
#include <string.h>
#include <math.h>
#include "tremo_uart.h"
#include "tremo_gpio.h"
#include "tremo_rcc.h"
#include "tremo_delay.h"

#define UART0_INSTANCE UART0
#define UART1_INSTANCE UART1
#define UART2_INSTANCE UART2

#define RX_BUFFER_SIZE 30
#define HEADER_1 0xAA
#define HEADER_2 0xFF
#define HEADER_3 0x03
#define HEADER_4 0x00
#define FOOTER_1 0x55
#define FOOTER_2 0xCC
#define NUM_TARGETS 3
#define UART_BAUDRATE 256000

uint8_t uart0_rx_buffer[RX_BUFFER_SIZE];
uint8_t uart1_rx_buffer[RX_BUFFER_SIZE];
uint8_t uart2_rx_buffer[RX_BUFFER_SIZE];

volatile uint8_t uart0_rx_index = 0;
volatile uint8_t uart1_rx_index = 0;
volatile uint8_t uart2_rx_index = 0;

volatile uint8_t uart0_frame_received = 0;
volatile uint8_t uart1_frame_received = 0;
volatile uint8_t uart2_frame_received = 0;


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

int validate_frame(uint8_t *buffer) {
    return (buffer[0] == HEADER_1 && buffer[1] == HEADER_2 &&
            buffer[2] == HEADER_3 && buffer[3] == HEADER_4 &&
            buffer[RX_BUFFER_SIZE - 2] == FOOTER_1 &&
            buffer[RX_BUFFER_SIZE - 1] == FOOTER_2);
}

void processing_data(uint8_t *rx_buffer, const char* uart_name) {
    for (int i = 0; i < NUM_TARGETS; i++) {
        int offset = 4 + (i * 8);

        uint16_t x_raw     = rx_buffer[offset] | (rx_buffer[offset + 1] << 8);
        uint16_t y_raw     = rx_buffer[offset + 2] | (rx_buffer[offset + 3] << 8);
        uint16_t speed_raw = rx_buffer[offset + 4] | (rx_buffer[offset + 5] << 8);
        uint16_t dist_raw  = rx_buffer[offset + 6] | (rx_buffer[offset + 7] << 8);

        int16_t x_coord = -x_raw;
        int16_t y_coord = y_raw - 32768;
        int16_t speed = -speed_raw;
        uint16_t distance = dist_raw;

        if (y_coord == -32768) y_coord = 0;
        if (speed == 0 && x_coord == 0 && y_coord == 0) continue;

        double target_distance = sqrt(pow(x_coord, 2) + pow(y_coord, 2)) / 10.0;
        const char *movement = (speed == 0) ? "Stationary" : (speed < 0 ? "Approaching" : "Moving Away");

        printf("\r\n[%s] Target %d:\r\n", uart_name, i + 1);
        printf("  X: %d cm | Y: %d cm | Speed: %d cm/s (%s) | Distance Raw: %u cm | Calc. Dist: %.2f cm\r\n",
            x_coord / 10, y_coord / 10, speed / 10, movement, distance / 10, target_distance);
    }
}

void UART0_IRQHandler(void) {
    if (uart_get_interrupt_status(UART0_INSTANCE, UART_INTERRUPT_RX_DONE)) {
        uart0_rx_buffer[uart0_rx_index++] = uart_receive_data(UART0_INSTANCE);
        if (uart0_rx_index >= RX_BUFFER_SIZE) {
            uart0_rx_index = 0;
            uart0_frame_received = 1;
        }
        uart_clear_interrupt(UART0_INSTANCE, UART_INTERRUPT_RX_DONE);
    }
}

void UART1_IRQHandler(void) {
    if (uart_get_interrupt_status(UART1_INSTANCE, UART_INTERRUPT_RX_DONE)) {
        uart1_rx_buffer[uart1_rx_index++] = uart_receive_data(UART1_INSTANCE);
        if (uart1_rx_index >= RX_BUFFER_SIZE) {
            uart1_rx_index = 0;
            uart1_frame_received = 1;
        }
        uart_clear_interrupt(UART1_INSTANCE, UART_INTERRUPT_RX_DONE);
    }
}

void UART2_IRQHandler(void) {
    if (uart_get_interrupt_status(UART2_INSTANCE, UART_INTERRUPT_RX_DONE)) {
        uart2_rx_buffer[uart2_rx_index++] = uart_receive_data(UART2_INSTANCE);
        if (uart2_rx_index >= RX_BUFFER_SIZE) {
            uart2_rx_index = 0;
            uart2_frame_received = 1;
        }
        uart_clear_interrupt(UART2_INSTANCE, UART_INTERRUPT_RX_DONE);
    }
}

void my_uart_init(void) {
    uart_config_t uart_config;
    uart_config_init(&uart_config);
    uart_config.baudrate = UART_BAUDRATE;
    uart_config.data_width = UART_DATA_WIDTH_8;
    uart_config.parity = UART_PARITY_NO;
    uart_config.stop_bits = UART_STOP_BITS_1;
    uart_config.fifo_mode = ENABLE;

    uart_init(UART0_INSTANCE, &uart_config);
    uart_cmd(UART0_INSTANCE, ENABLE);
    uart_config_interrupt(UART0_INSTANCE, UART_INTERRUPT_RX_DONE, ENABLE);
    NVIC_EnableIRQ(UART0_IRQn);

    uart_init(UART1_INSTANCE, &uart_config);
    uart_cmd(UART1_INSTANCE, ENABLE);
    uart_config_interrupt(UART1_INSTANCE, UART_INTERRUPT_RX_DONE, ENABLE);
    NVIC_EnableIRQ(UART1_IRQn);

    uart_init(UART2_INSTANCE, &uart_config);
    uart_cmd(UART2_INSTANCE, ENABLE);
    uart_config_interrupt(UART2_INSTANCE, UART_INTERRUPT_RX_DONE, ENABLE);
    NVIC_EnableIRQ(UART2_IRQn);
}

int main(void) {
    // Clock Enable
    rcc_enable_peripheral_clk(RCC_PERIPHERAL_UART0, true);
    rcc_enable_peripheral_clk(RCC_PERIPHERAL_UART1, true);
    rcc_enable_peripheral_clk(RCC_PERIPHERAL_UART2, true);
    rcc_enable_peripheral_clk(RCC_PERIPHERAL_GPIOA, true);
    rcc_enable_peripheral_clk(RCC_PERIPHERAL_GPIOB, true);

    // GPIO Mux for UART0 (B0 RX, B1 TX)
    gpio_set_iomux(GPIOB, GPIO_PIN_0, 1); // UART0_RX og rx i.e IO16
    gpio_set_iomux(GPIOB, GPIO_PIN_1, 1); // UART0_TX og tx i.e IO17

    // GPIO Mux for UART1 (A4 RX, A5 TX)
    gpio_set_iomux(GPIOA, GPIO_PIN_4, 1); // UART1_RX IO04
    gpio_set_iomux(GPIOA, GPIO_PIN_5, 1); // UART1_TX IO05

    // GPIO Mux for UART2 (A8 RX, A9 TX)
    gpio_set_iomux(GPIOA, GPIO_PIN_8, 1); // UART2_RX IO08
    gpio_set_iomux(GPIOA, GPIO_PIN_9, 1); // UART2_TX IO09

    my_uart_init();

    while (1) {
        delay_ms(10);

        if (uart0_frame_received) {
            uart0_frame_received = 0;
            if (validate_frame(uart0_rx_buffer)) {
                printf("\n[UART0] Valid Frame Received:\n");
                for (uint8_t i = 0; i < RX_BUFFER_SIZE; i++) printf("%02X ", uart0_rx_buffer[i]);
                printf("\n");
                processing_data(uart0_rx_buffer, "UART0");
            }
        }

        if (uart1_frame_received) {
            uart1_frame_received = 0;
            if (validate_frame(uart1_rx_buffer)) {
                printf("\n[UART1] Valid Frame Received:\n");
                for (uint8_t i = 0; i < RX_BUFFER_SIZE; i++) printf("%02X ", uart1_rx_buffer[i]);
                printf("\n");
                processing_data(uart1_rx_buffer, "UART1");
            }
        }

        if (uart2_frame_received) {
            uart2_frame_received = 0;
            if (validate_frame(uart2_rx_buffer)) {
                printf("\n[UART2] Valid Frame Received:\n");
                for (uint8_t i = 0; i < RX_BUFFER_SIZE; i++) printf("%02X ", uart2_rx_buffer[i]);
                printf("\n");
                processing_data(uart2_rx_buffer, "UART2");
            }
        }
    }

    return 0;
}

