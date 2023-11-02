#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"

static char event_str[128];
static int num_edge;
struct repeating_timer timer;
static uint32_t pulse_width;
static int width_data[4] = {0};
static int width_index = 0;
static int width_count = 0;
static float width_average = 0;
static int width_sum = 0;
static float d_travelled = 0;

void gpio_event_string(char *buf, uint32_t events);

void gpio_callback(uint gpio, uint32_t events) {
    // Put the GPIO event(s) that just happened into event_str
    // so we can print it
    static uint32_t edge_fall_time = 0;
    gpio_event_string(event_str, events);
    if (events == 4){
        num_edge++;
        edge_fall_time = time_us_32(); // Time is in microseconds
    } else if (events == 8){   
        width_sum -= width_data[width_index];
        width_data[width_index] = (time_us_32() - edge_fall_time); //Convert microsecond to millisecond
        width_sum += width_data[width_index];
        width_index = (width_index + 1) % 4;
        if (width_count < 4) width_count++; 
        width_average = (float) (width_sum/(1000000.0f))/width_count;
    }
}

bool print_out(struct repeating_timer *t) {
    float speed_per_sec = 0; // Measure in cm/s
    float distance_per_sec = 0; // Measured in cm
    // Approximation of distance travelled. Found using diameter of wheel encoder disc (2cm), circumfrence (6.28cm), Each slit + pillar is approx. 6.28/20 = 0.314cm
    distance_per_sec= (num_edge*0.314); 
    d_travelled += distance_per_sec;
    // 0.314/2 = 0.157cm is the estimatd length of each slit
    speed_per_sec = 0.157/width_average; 
    printf("Speed using edge per sec: %f cm/s\n", distance_per_sec);
    printf("Speed using pluse width: %f cm/s\n\n", speed_per_sec);
    num_edge = 0; //Reset number of edge per second
    return true;
}

int main() {
    stdio_init_all();

    printf("Hello GPIO IRQ\n"); 
    gpio_set_irq_enabled_with_callback(2, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &gpio_callback);
    add_repeating_timer_ms(-1000, print_out, NULL, &timer);
    // Wait forever
    while (1);
}


static const char *gpio_irq_str[] = {
        "LEVEL_LOW",  // 0x1
        "LEVEL_HIGH", // 0x2
        "EDGE_FALL",  // 0x4
        "EDGE_RISE"   // 0x8
};

void gpio_event_string(char *buf, uint32_t events) {
    for (uint i = 0; i < 4; i++) {
        uint mask = (1 << i);
        if (events & mask) {
            // Copy this event string into the user string
            const char *event_str = gpio_irq_str[i];
            while (*event_str != '\0') {
                *buf++ = *event_str++;
            }
            events &= ~mask;

            // If more events add ", "
            if (events) {
                *buf++ = ',';
                *buf++ = ' ';
            }
        }
    }
    *buf++ = '\0';
}
