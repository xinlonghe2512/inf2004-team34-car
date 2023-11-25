/**
 CHANNEL B = LEFT WHEEL 
 CHANNEL A = RIGHT WHEEL 
 */

// Output PWM signals on pins 0 and 1

#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include <stdio.h>


// function to stop car
void stopCar(uint slice_num0, uint slice_num1){ 
    //stop the car by changing duty cycle to 0%
    pwm_set_chan_level(slice_num0, PWM_CHAN_A, 0);  // Stop Channel A
    pwm_set_chan_level(slice_num1, PWM_CHAN_B, 0);  // Stop Channel B
    pwm_set_enabled(slice_num0, true);
    pwm_set_enabled(slice_num1, true);
}

// function to start car
void startCar(uint slice_num0, uint slice_num1){ 
    //stop the car by changing duty cycle to 50%
    pwm_set_chan_level(slice_num0, PWM_CHAN_A, 12500/2);  // Start Channel A
    pwm_set_chan_level(slice_num1, PWM_CHAN_B, 12500/2);  // Start Channel B
    pwm_set_enabled(slice_num0, true);
    pwm_set_enabled(slice_num1, true);
}


// function to increase speed 
void speed3(uint slice_num0, uint slice_num1){ 
    //increase speed by setting to max duty cycle
    pwm_set_chan_level(slice_num0, PWM_CHAN_A, 12500);  // Start Channel A
    pwm_set_chan_level(slice_num1, PWM_CHAN_B, 12500);  // Start Channel B
    pwm_set_enabled(slice_num0, true);
    pwm_set_enabled(slice_num1, true);
}

// function to increase speed 
void speed2(uint slice_num0, uint slice_num1){ 
    //increase speed by setting to max duty cycle
    pwm_set_chan_level(slice_num0, PWM_CHAN_A, 12500/2);  // Start Channel A
    pwm_set_chan_level(slice_num1, PWM_CHAN_B, 12500/2);  // Start Channel B
    pwm_set_enabled(slice_num0, true);
    pwm_set_enabled(slice_num1, true);
}
// function to increase speed 
void speed1(uint slice_num0, uint slice_num1){ 
    //increase speed by setting to max duty cycle
    pwm_set_chan_level(slice_num0, PWM_CHAN_A, 4000);  // Start Channel A
    pwm_set_chan_level(slice_num1, PWM_CHAN_B, 4000);  // Start Channel B
    pwm_set_enabled(slice_num0, true);
    pwm_set_enabled(slice_num1, true);
}

// function to 180 degree turn
void turnAround180(uint slice_num0, uint slice_num1, uint BTN_PIN3, uint BTN_PIN4, bool right1, bool right2, bool right3, bool right4){ 
    // change direction of left wheel to turn 90 degrees
    gpio_set_pulls(BTN_PIN3, !right1, !right2);
    gpio_set_pulls(BTN_PIN4, !right3, !right4);
    pwm_set_chan_level(slice_num0, PWM_CHAN_A, 7000);  // Start Channel A
    pwm_set_chan_level(slice_num1, PWM_CHAN_B, 7000);  // Start Channel B
    pwm_set_enabled(slice_num0, true);
    pwm_set_enabled(slice_num1, true);
    sleep_ms(4000); // 2 seconds should be enough for 180 degrees turn, can adjust accordingly
    stopCar(slice_num0, slice_num1); // stop the car after turning 

}

// function to 180 degree turn
void forwardDirection(uint BTN_PIN1, uint BTN_PIN2, bool ex1, bool ex2, bool ex3, bool ex4){ 
    gpio_set_pulls(BTN_PIN1, !ex1, !ex2);
    gpio_set_pulls(BTN_PIN2, !ex3, !ex4);
}


// function to turn sharp left from stationary [REQUIRES TO STOP CAR FIRST]
void turnSharpLeft(uint slice_num0, uint slice_num1, uint BTN_PIN3, uint BTN_PIN4, bool right1, bool right2, bool right3, bool right4){ 
    // change direction of left wheel to turn 90 degrees
    gpio_set_pulls(BTN_PIN3, !right1, !right2);
    gpio_set_pulls(BTN_PIN4, !right3, !right4);

    pwm_set_chan_level(slice_num0, PWM_CHAN_A, 7000);  // Start Channel A
    pwm_set_chan_level(slice_num1, PWM_CHAN_B, 7000);  // Start Channel B
    pwm_set_enabled(slice_num0, true);
    pwm_set_enabled(slice_num1, true);
    sleep_ms(2000); // 2 seconds should be enough for 90 degrees turn, can adjust accordingly
    stopCar(slice_num0, slice_num1); // stop the car after turning 
}


// function to turn sharp right from stationary [REQUIRES TO STOP CAR FIRST]
void turnSharpRight(uint slice_num0, uint slice_num1, uint BTN_PIN1, uint BTN_PIN2, bool left1, bool left2, bool left3, bool left4){ 
    // change direction of right wheel to turn 90 degrees
    gpio_set_pulls(BTN_PIN1, !left1, !left2);
    gpio_set_pulls(BTN_PIN2, !left3, !left4);

    pwm_set_chan_level(slice_num0, PWM_CHAN_A, 12500/2);  // Start Channel A
    pwm_set_chan_level(slice_num1, PWM_CHAN_B, 12500/2);  // Start Channel B
    pwm_set_enabled(slice_num0, true);
    pwm_set_enabled(slice_num1, true);
    sleep_ms(2000); // 2 seconds should be enough for 90 degrees turn, can adjust accordingly

    // turn finish, now change back the direction of right wheel to face forward
    gpio_set_pulls(BTN_PIN1, !left1, !left2);
    gpio_set_pulls(BTN_PIN2, !left3, !left4);
    stopCar(slice_num0, slice_num1);
}





int main() {

    // set pins for 3.3V and GND
    const uint BTN_PIN1 = 18; // pin for channel A
    const uint BTN_PIN2 = 19; // pin for channel A
    const uint BTN_PIN3 = 20; // pin for channel B
    const uint BTN_PIN4 = 21; // pin for channel B

    // set initial directions for both wheels
    bool left1 = true, left2 = false;
    bool left3 = false, left4 = true;
    bool right1 = true, right2 = false;
    bool right3 = false, right4 = true;

    // set direction for the pins 
    gpio_set_dir(BTN_PIN1, GPIO_IN);
    gpio_set_pulls(BTN_PIN1, left1, left2);
    gpio_set_dir(BTN_PIN2, GPIO_IN);
    gpio_set_pulls(BTN_PIN2, left3, left4);

    gpio_set_dir(BTN_PIN3, GPIO_IN);
    gpio_set_pulls(BTN_PIN3, right1, right2);
    gpio_set_dir(BTN_PIN4, GPIO_IN);
    gpio_set_pulls(BTN_PIN4, right3, right4);

    // Tell GPIO 0 and 1 they are allocated to the PWM
    gpio_set_function(0, GPIO_FUNC_PWM);
    gpio_set_function(1, GPIO_FUNC_PWM);

    // Find out which PWM slice is connected to GPIO 0 (it's slice 0)
    uint slice_num0 = pwm_gpio_to_slice_num(0);
    pwm_set_clkdiv(slice_num0,100);
    // this code here means that it will run 12500 cycles per second
    pwm_set_wrap(slice_num0, 12500);

     // Find out which PWM slice is connected to GPIO 1 (it's slice 1)
    uint slice_num1 = pwm_gpio_to_slice_num(1);
    pwm_set_clkdiv(slice_num1,100);
    // this code here means that it will run 12500 cycles per second
    pwm_set_wrap(slice_num1, 12500);

    
    // call functions here if need to 
    //stopCar(slice_num0,slice_num1);
    //startCar(slice_num0,slice_num1);
    

}