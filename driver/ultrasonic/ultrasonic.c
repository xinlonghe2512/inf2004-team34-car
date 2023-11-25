#include <stdio.h>
#include <stdlib.h>
#include "hardware/gpio.h"
#include "hardware/timer.h"
#include "hardware/irq.h"
#include "pico/stdlib.h"

#define TRIGGER_PIN 2
#define ECHO_PIN 3
#define TIMEOUT_US 26100

absolute_time_t startTime;
absolute_time_t endTime;

// Function for trigger pin to send out the initiation pulse
void sendInitiationPulse(uint trigPin)
{
    gpio_put(trigPin, 1);
    busy_wait_us(10);
    gpio_put(trigPin, 0);
}

// Function to calculate the distance from the ultrasonic
void getUltrasonicDetection(uint64_t pulseLength)
{
    // Convert the ultrasonic pulse into cm
    float ultrasonic_in_cm = pulseLength / 29 / 2;

    // Get the average of the ultrasonic for smoothing
    float ultrasonic_average = 0;
    for (int i = 0; i < 50; ++i)
    {
        ultrasonic_average = ultrasonic_average + ultrasonic_in_cm;
    }

    ultrasonic_average = (ultrasonic_average / 50);

    // Display the results obtained after smoothing
    printf("Moving Distance: %.2f cm\n\n", ultrasonic_average);

    // Enable trigger pin to send out the next pulse again
    gpio_put(TRIGGER_PIN, 1);
}

// Callback function for ultrasonic repeating timer to get ultrasonic detection
void handle_echo(uint gpio, uint32_t events)
{
    if (events == GPIO_IRQ_EDGE_RISE)
    {
        // Disable trigger pin
        gpio_put(TRIGGER_PIN, 0);

        // Get the start time when edge rise
        startTime = get_absolute_time();
    }

    if (events == GPIO_IRQ_EDGE_FALL)
    {
        // Get the end time when edge fall
        endTime = get_absolute_time();

        // Check if difference between start time and end time is more than timeout
        if (absolute_time_diff_us(startTime, endTime) > TIMEOUT_US)
        {
            startTime = endTime;
        }

        // Pass the difference to calculate ultrasonic distance
        getUltrasonicDetection(absolute_time_diff_us(startTime, endTime));
    }
}

// Timer to send ultrasonic initiation pulse to initiate a measurement
bool repeating_timer_callback(struct repeating_timer *t)
{
    sendInitiationPulse(TRIGGER_PIN);
    return true;
}

// Function to setup the ultrasonic pins
void setupUltrasonicPins(uint trigPin, uint echoPin)
{
    gpio_init(trigPin);
    gpio_init(echoPin);
    gpio_set_dir(trigPin, GPIO_OUT);
    gpio_set_dir(echoPin, GPIO_IN);
    gpio_pull_up(echoPin);

    // Set up interrupt for the echoPin
    gpio_set_irq_enabled_with_callback(ECHO_PIN, GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE, true, &handle_echo);
}

int main()
{
    stdio_init_all();

    // Set up the ultrasonic pins
    setupUltrasonicPins(TRIGGER_PIN, ECHO_PIN);

    // Get the distance between ultrasonic and the object infront every 100ms
    struct repeating_timer timer;
    add_repeating_timer_ms(100, repeating_timer_callback, NULL, &timer);

    while (1)
    {
        tight_loop_contents();
    }
}