#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"

#define ADC_CHANNEL_PIN_ONE 1
#define ADC_CHANNEL_PIN_TWO 2
#define ADC_BASE_PIN1 27
#define ADC_BASE_PIN2 28
#define WHITE_THRESHOLD 0.8

int main(void) {
    // Initialise all pins
    stdio_init_all();
    adc_init();
    adc_set_temp_sensor_enabled(true);

    gpio_set_function(ADC_BASE_PIN1, GPIO_FUNC_SIO);
    gpio_disable_pulls(ADC_BASE_PIN1);
    gpio_set_input_enabled(ADC_BASE_PIN1, false);

    gpio_set_function(ADC_BASE_PIN2, GPIO_FUNC_SIO);
    gpio_disable_pulls(ADC_BASE_PIN2);
    gpio_set_input_enabled(ADC_BASE_PIN2, false);

    // Initialise conversion factor
    const float conversion_factor = 3.3f / (1 << 12);
    while (1) 
    {
	// Read value on left IR
        adc_select_input(ADC_CHANNEL_PIN_ONE);
        uint32_t leftRead = adc_read();
        float leftResult = leftRead * conversion_factor;
        printf("Left Read: %f V\n", leftResult);
	
	// Read value on right IR
        adc_select_input(ADC_CHANNEL_PIN_TWO);
        uint32_t rightRead = adc_read();
        float rightResult = rightRead * conversion_factor;
        printf("Right Read: %f V\n", rightResult);
	
	// If left IR sense white and right sense black, turn left
        if(leftResult < WHITE_THRESHOLD && rightResult > WHITE_THRESHOLD)
        {
            printf("Turn left");
        }

	// If right IR sense white and left sense black, turn right
        else if (rightResult < WHITE_THRESHOLD && leftResult > WHITE_THRESHOLD)
        {
            printf("Turn right");
        }
	
	// If both IR sense black, car is on track
        else if (rightResult > WHITE_THRESHOLD && leftResult > WHITE_THRESHOLD)
        {
            printf("On track");
        }
	
	// If both IR sense white, car is off track
        else
        {
            printf("Off track");
        }
	
        printf("\n");
	// Delay for approximately 1 second until next iteration
        sleep_ms(1000);
    }
}
