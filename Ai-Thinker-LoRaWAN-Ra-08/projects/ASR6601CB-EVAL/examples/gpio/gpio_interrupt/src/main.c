#include <stdio.h>
#include "tremo_rcc.h"
#include "tremo_gpio.h"
#include "tremo_delay.h"
#include "tremo_it.h"

#define BUTTON_GPIO GPIOA
#define BUTTON_PIN GPIO_PIN_11
#define LED_GPIO GPIOA
#define LED_PIN GPIO_PIN_5

// Define global variables for tremo_it.c
gpio_t *g_test_gpiox = BUTTON_GPIO;
uint8_t g_test_pin = BUTTON_PIN;
volatile uint32_t g_gpio_interrupt_flag = 0; // Flag set on button press

int main(void)
{
    // Enable clock for GPIOA
    rcc_enable_peripheral_clk(RCC_PERIPHERAL_GPIOA, true);
    
    // Initialize delay system
    delay_init();

    // Configure button as input with internal pull-down resistor
    gpio_init(BUTTON_GPIO, BUTTON_PIN, GPIO_MODE_INPUT_PULL_DOWN);
    
    // Configure interrupt for rising edge (button press)
    gpio_config_interrupt(BUTTON_GPIO, BUTTON_PIN, GPIO_INTR_RISING_EDGE);
    
    // Enable GPIO interrupt in NVIC
    NVIC_EnableIRQ(GPIO_IRQn);
    NVIC_SetPriority(GPIO_IRQn, 2);

    // Configure LED as output
    gpio_init(LED_GPIO, LED_PIN, GPIO_MODE_OUTPUT_PP_LOW);
    gpio_write(LED_GPIO, LED_PIN, 0); // Ensure LED is OFF initially

    while (1) {
        if (g_gpio_interrupt_flag) {
            g_gpio_interrupt_flag = 0; // Reset flag
            
            // Blink LED 5 times
            for (int i = 0; i < 5; i++) {
                gpio_write(LED_GPIO, LED_PIN, 1); // LED ON
                delay_ms(500);
                gpio_write(LED_GPIO, LED_PIN, 0); // LED OFF
                delay_ms(500);
            }
        }
    }
}

