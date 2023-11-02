#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <stdint.h>
#include "hardware/gpio.h"
#include "hardware/uart.h"
#include "hardware/timer.h"
#include "hardware/i2c.h"
#include "hardware/irq.h"
#include "pico/stdlib.h"
#include "pico/binary_info.h"

#define TRIGGER_PIN 2
#define ECHO_PIN 3
#define TIMEOUT_US 26100

absolute_time_t startTime;
absolute_time_t endTime;

// Kalman filter structure
typedef struct
{
    float Q; // Process noise covariance
    float R; // Measurement noise covariance
    float x; // Estimated state
    float P; // Estimation error covariance
    float K; // Kalman gain
} KalmanFilter;

// Initialize Kalman filter parameters
void initKalmanFilter(KalmanFilter *filter, float Q, float R)
{
    filter->Q = Q;
    filter->R = R;
    filter->x = 0; // Initial state estimation
    filter->P = 1; // Initial error covariance estimation
    filter->K = 0; // Initial Kalman gain
}

// Update Kalman filter with new measurement
void updateKalmanFilter(KalmanFilter *filter, float measurement)
{
    // Prediction step
    float x_pred = filter->x;
    float P_pred = filter->P + filter->Q;

    // Update step
    filter->K = P_pred / (P_pred + filter->R);
    filter->x = x_pred + filter->K * (measurement - x_pred);
    filter->P = (1 - filter->K) * P_pred;
}

void handle_echo(uint gpio, uint32_t events)
{
    if (events == GPIO_IRQ_EDGE_RISE)
    {
        startTime = get_absolute_time();
    }
    
    if (events == GPIO_IRQ_EDGE_FALL)
    {
        endTime = get_absolute_time();
        if (absolute_time_diff_us(startTime, endTime) > TIMEOUT_US)
        {
            startTime = endTime;
        }
    }
}

// Function to setup the ultrasonic pins
void setupUltrasonicPins(uint trigPin, uint echoPin)
{
    gpio_init(trigPin);
    gpio_init(echoPin);
    gpio_set_dir(trigPin, GPIO_OUT);
    gpio_set_dir(echoPin, GPIO_IN);
    gpio_pull_up(echoPin);
}

// Function to get the ultrasonic pulse
int getPulse(uint trigPin, uint echoPin)
{
    gpio_put(trigPin, 1);
    busy_wait_us(10);
    gpio_put(trigPin, 0);

    return absolute_time_diff_us(startTime, endTime);
}

// Function to convert the ultrasonic pulse into cm
uint64_t getCm(uint trigPin, uint echoPin)
{
    uint64_t pulseLength = getPulse(trigPin, echoPin);
    return pulseLength / 29 / 2;
}

/* Calculate the distance from the car to the wall on each side */
void getUltrasonicDetection(uint trigPin, uint echoPin)
{
    float ultrasonic_in_cm = getCm(trigPin, echoPin);

    // Initialize Kalman filters for the ultrasonic data
    KalmanFilter filter;
    initKalmanFilter(&filter, 0.01, 0.1);

    // Perform Kalman filtering for ultrasonic data
    updateKalmanFilter(&filter, ultrasonic_in_cm);
    float filteredData = filter.x;

    // Display the results obtained after kalman filtering
    printf("Kalman Distance: %.2f cm\n", filteredData);

    // Get the average of the ultrasonic for smoothing
    float ultrasonic_average = 0;
    for (int i = 0; i < 50; ++i)
    {
        ultrasonic_average = ultrasonic_average + ultrasonic_in_cm;
    }

    ultrasonic_average = (ultrasonic_average / 50);

    // Display the results obtained after smoothing
    printf("Moving Distance: %.2f cm\n\n", ultrasonic_average);
}


// Callback function for ultrasonic repeating timer to get ultrasonic detection
bool repeating_timer_callback(struct repeating_timer *t)
{
    getUltrasonicDetection(TRIGGER_PIN, ECHO_PIN);
    return true;
}

int main()
{
    stdio_init_all();

    // Set up the ultrasonic pins
    setupUltrasonicPins(TRIGGER_PIN, ECHO_PIN);

    // Set up interrupt for the echoPin
    gpio_set_irq_enabled_with_callback(ECHO_PIN, GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE, true, &handle_echo);

    // Get the distance between ultrasonic and the object infront every 50ms
    struct repeating_timer timer;
    add_repeating_timer_ms(50, repeating_timer_callback, NULL, &timer);

    while (1)
    {
        tight_loop_contents();
    }
}