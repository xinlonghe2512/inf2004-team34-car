#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"

#define ENCODER_LEFT 6
#define ENCODER_RIGHT 7

//static char event_str[128];
static int num_edge_l;
static int num_edge_r;
static float pulse_width_l;
static float pulse_width_r;
static float t_distance_travelled;
struct repeating_timer timer;

void gpio_event_string(char *buf, uint32_t events);

void gpio_callback(uint gpio, uint32_t events) {
    // Put the GPIO event(s) that just happened into event_str
    // so we can print it
    static uint32_t edge_fall_time_l;
    static uint32_t edge_fall_time_r;
    //gpio_event_string(event_str, events);
    if (gpio == ENCODER_LEFT){ // 
        pulse_width_l = (float) (time_us_32() - edge_fall_time_l)/(1000000.0f);
        num_edge_l++;
        edge_fall_time_l = time_us_32(); // Time is in microseconds
    } else if (gpio == ENCODER_RIGHT){    // Edge rise
        pulse_width_r = (float) (time_us_32() - edge_fall_time_r)/(1000000.0f);
        num_edge_r++;
        edge_fall_time_r = time_us_32(); // Time is in microseconds
    }
}

bool print_out(struct repeating_timer *t) {
    float speed_per_sec_l = 0; // Measure in cm/s
    float speed_per_sec_r = 0; // Measure in cm/s
    float distance_per_sec = 0; // Measured in cm
    
    // Approximation of distance travelled. Found using diameter of wheel encoder disc (2cm), circumfrence (6.28cm), Each slit + pillar is approx. 6.28/20 = 0.314cm
    distance_per_sec= (((num_edge_l+num_edge_r)/2)*0.314); 
    t_distance_travelled += distance_per_sec;
    
    // 0.314cm is the estimatd length of each slit
    speed_per_sec_l = (pulse_width_l > 0) ? 0.314/pulse_width_l : 0; 
    speed_per_sec_r = (pulse_width_r > 0) ? 0.314/pulse_width_r : 0; 
    
    // Print onto console
    printf("Total distance: %f\n", t_distance_travelled);
    printf("Speed using edge per sec: %f cm/s\n", distance_per_sec);
    printf("Speed using left pluse width: %f cm/s\n", speed_per_sec_l);
    printf("Speed using right pluse width: %f cm/s\n\n", speed_per_sec_r);
    
    // Reset number of edge per second
    num_edge_l = 0;
    num_edge_r = 0; 
    return true;
}

int main() {
    stdio_init_all();

    // Set pin 8 as power

    printf("Hello GPIO IRQ\n"); 
    gpio_set_function(ENCODER_LEFT, GPIO_IN);
    gpio_set_function(ENCODER_RIGHT, GPIO_IN);

    // Configure GPIO pin 6
    gpio_set_irq_enabled_with_callback(ENCODER_LEFT, GPIO_IRQ_EDGE_FALL, true, &gpio_callback);
    // Configure GPIO pin 7
    gpio_set_irq_enabled_with_callback(ENCODER_RIGHT, GPIO_IRQ_EDGE_FALL, true, &gpio_callback);

    add_repeating_timer_ms(-1000, print_out, NULL, &timer);
    // Wait forever
    while (1);
}
