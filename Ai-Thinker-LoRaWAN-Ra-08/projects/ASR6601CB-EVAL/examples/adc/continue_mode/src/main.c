#include <stdio.h>
#include <string.h>
#include "tremo_regs.h"
#include "tremo_adc.h"
#include "tremo_rcc.h"
#include "tremo_gpio.h"
#include "tremo_uart.h"
#include "tremo_delay.h"

#define ADC_SAMPLES 10
#define MOVING_AVG_WINDOW 5
#define ADC_TIMEOUT_LOOPS 100 // Approx. 100ms timeout (1ms per loop)
#define ADC_RESET_INTERVAL_LOOPS 120 // Approx. 60s (120 * 500ms)

uint16_t adc_data_1 = 0;
float calibrated_sample_1 = 0.0;
float moving_avg_buffer[MOVING_AVG_WINDOW];
uint8_t moving_avg_index = 0;
float moving_avg_sum = 0.0;

#if defined(pin11)
#define gpioadc GPIO_PIN_11
#else
#define gpioadc GPIO_PIN_8
#endif

void uart_log_init(void)
{
    gpio_set_iomux(GPIOB, GPIO_PIN_0, 1);
    gpio_set_iomux(GPIOB, GPIO_PIN_1, 1);

    uart_config_t uart_config;
    uart_config_init(&uart_config);

    uart_config.baudrate = UART_BAUDRATE_115200;
    uart_init(CONFIG_DEBUG_UART, &uart_config);
    uart_cmd(CONFIG_DEBUG_UART, ENABLE);
}

void adc_continue_mode_init(void)
{
    float gain_value, dco_value;

    // Get calibration values
    adc_get_calibration_value(false, &gain_value, &dco_value);

    // Configure GPIO as analog input for ADC channel 1 (PA8)
    gpio_init(GPIOA, gpioadc, GPIO_MODE_ANALOG);

    adc_init();
    adc_config_clock_division(24); // Increased clock division for stability

    // Configure ADC to read channel 1 continuously
    adc_config_sample_sequence(0, 1);  
    adc_config_conv_mode(ADC_CONV_MODE_CONTINUE);

    adc_enable(true);
    adc_start(true);

    // Initialize moving average buffer
    memset(moving_avg_buffer, 0, sizeof(moving_avg_buffer));
    moving_avg_index = 0;
    moving_avg_sum = 0.0;
}

void adc_reset(void)
{
    // Disable ADC
    adc_enable(false);
    delay_ms(10);

    // Reinitialize ADC
    adc_init();
    adc_config_clock_division(24);
    adc_config_sample_sequence(0, 1);
    adc_config_conv_mode(ADC_CONV_MODE_CONTINUE);

    // Re-enable and start ADC
    adc_enable(true);
    adc_start(true);
}

float adc_read_average(void)
{
    uint32_t sum = 0;
    uint16_t data;
    uint8_t valid_samples = 0;
    uint32_t timeout_counter = 0;

    for (int i = 0; i < ADC_SAMPLES; i++) {
        adc_start(true);

        timeout_counter = 0;
        while (!adc_get_interrupt_status(ADC_ISR_EOC)) {
            delay_ms(1); 
            timeout_counter++;
            if (timeout_counter > ADC_TIMEOUT_LOOPS) {
                printf("ADC Timeout, resetting ADC\r\n");
                adc_reset();
                return 0.0; 
            }
        }

        // Read ADC data
        data = adc_get_data();
        adc_clear_interrupt_status(ADC_ISR_EOC);

        // Validate ADC data (basic range check)
        if (data < 4096) { // 12-bit ADC
            sum += data;
            valid_samples++;
        }

        delay_ms(10); // Small delay between samples
    }

    if (valid_samples == 0) {
        printf("No valid ADC samples\r\n");
        return 0.0; // Return 0 if no valid samples
    }

    // Calculate average and calibrate
    float avg_data = (float)sum / valid_samples;
    float gain_value, dco_value;
    adc_get_calibration_value(false, &gain_value, &dco_value);
    return (avg_data * gain_value + dco_value) * (1.2 / 4096); // Calibrated voltage
}

float update_moving_average(float new_sample)
{
    // Subtract the oldest sample
    moving_avg_sum -= moving_avg_buffer[moving_avg_index];
    
    // Add the new sample
    moving_avg_buffer[moving_avg_index] = new_sample;
    moving_avg_sum += new_sample;
    
    // Update index
    moving_avg_index = (moving_avg_index + 1) % MOVING_AVG_WINDOW;
    
    // Return the moving average
    return moving_avg_sum / MOVING_AVG_WINDOW;
}

float calculate_battery_percentage(float voltage)
{
    const float v_min = 0.78;
    const float v_max = 1.42;
    
    if (voltage <= v_min) return 0.0;
    if (voltage >= v_max) return 100.0;
    
    return ((voltage - v_min) / (v_max - v_min)) * 100.0;
}

void adc_continue_mode_read(void)
{
    uint32_t loop_counter = 0;

    while (1)
    {
        // Periodically reset ADC to prevent lockup
        if (loop_counter >= ADC_RESET_INTERVAL_LOOPS) {
            printf("Periodic ADC reset\r\n");
            adc_reset();
            loop_counter = 0;
        }

        // Read averaged and calibrated ADC sample
        calibrated_sample_1 = adc_read_average();
        
        if (calibrated_sample_1 > 0.0) { // Valid reading
            // Update moving average
            float smoothed_voltage = update_moving_average(calibrated_sample_1);
            
            // Calculate battery percentage
            float battery_percentage = calculate_battery_percentage(smoothed_voltage);
            
            // Print results
            printf("ADC Voltage: %.4f V, Battery: %.1f%%\r\n", 
                   smoothed_voltage, battery_percentage);
        } else {
            printf("Invalid ADC reading, resetting ADC\r\n");
            adc_reset();
        }

        delay_ms(500); // Update every 500ms
        loop_counter++;
    }
}

int main(void)
{
    rcc_set_adc_clk_source(RCC_ADC_CLK_SOURCE_RCO48M);
    rcc_enable_peripheral_clk(RCC_PERIPHERAL_GPIOA, true);
    rcc_enable_peripheral_clk(RCC_PERIPHERAL_GPIOB, true);
    rcc_enable_peripheral_clk(RCC_PERIPHERAL_GPIOC, true);
    rcc_enable_peripheral_clk(RCC_PERIPHERAL_GPIOD, true);
    rcc_enable_peripheral_clk(RCC_PERIPHERAL_ADC, true);
    rcc_enable_peripheral_clk(RCC_PERIPHERAL_UART0, true);

    uart_log_init();
    adc_continue_mode_init();
    adc_continue_mode_read();

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