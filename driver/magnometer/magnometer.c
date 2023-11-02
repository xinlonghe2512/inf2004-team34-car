#include "stdio.h"
#include "math.h"
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "stdint.h"

// Accelerometer registers
#define ACCEL_ADDR 0x19
#define CTRL_REG1_A 0x20
#define CTRL_REG4_A 0x23
#define ACCEL_X_LOW 0x28
#define ACCEL_X_HIGH 0x29
#define ACCEL_Y_LOW 0x2A
#define ACCEL_Y_HIGH 0x2B
#define ACCEL_Z_LOW 0x2C
#define ACCEL_Z_HIGH 0x2D

// Magnetometer registers
#define MAG_ADDR 0x1E
#define MR_REG_M 0x02
#define MAG_X_LOW 0x04
#define MAG_X_HIGH 0x03
#define MAG_Z_LOW 0x06
#define MAG_Z_HIGH 0x05
#define MAG_Y_LOW 0x08
#define MAG_Y_HIGH 0x07

#define SCL_PIN 5
#define SDA_PIN 4

#define CONVERSION 130

typedef struct 
{
    int16_t Q; // Process noise covariance (as int16_t)
    int16_t R; // Measurement noise covariance (as int16_t)
    int16_t x[3]; // Estimated state (3D vector)
    int16_t P[3]; // Estimation error covariance (3x3 matrix)
    int16_t K[3]; // Kalman gain (3D vector)
} KalmanFilter;

void initKalmanFilter(KalmanFilter* filter, int16_t Q, int16_t R) 
{
    filter->Q = Q;
    filter->R = R;
    for (int i = 0; i < 3; ++i) 
    {
        filter->x[i] = 0; // Initial state estimation (0 for each dimension)   
        filter->P[i] = 1; // Initial error covariance estimation
        filter->K[i] = 0; // Initial error covariance estimation
    }
}

void updateKalmanFilter(KalmanFilter* filter, int16_t measurements[3]) 
{
    int16_t pred_x[3];
    int16_t pred_p[3];
    // Prediction step
    for (int i = 0; i < 3; ++i) 
    {
        pred_x[i] = filter->x[i];
        pred_p[i] = filter->P[i] + filter->Q;
    }

    // Update step
    for (int i = 0; i < 3; ++i) 
    {
        filter->K[i] = pred_p[i] / pred_p[i] + filter->R;
        filter->x[i] = pred_x[i] + (filter->K[i] * (measurements[i] - pred_x[i]));
        filter->P[i] = (1 - filter->K[i]) * pred_p[i];
    }
}

void configureRegister(uint8_t addr, uint8_t reg, uint8_t value) 
{
    uint8_t data[2] = {reg, value};
    i2c_write_blocking(i2c0, addr, data, 2, false);
}

uint8_t readRegister(uint8_t addr, uint8_t reg) 
{
    uint8_t data;
    i2c_write_blocking(i2c0, addr, &reg, 1, true);
    i2c_read_blocking(i2c0, addr, &data, 1, false);
    return data;
}


void readAccelerometer(int16_t *x, int16_t *y, int16_t *z) 
{
    *x = (int16_t)((readRegister(ACCEL_ADDR, ACCEL_X_HIGH) << 8) | readRegister(ACCEL_ADDR, ACCEL_X_LOW));
    *y = (int16_t)((readRegister(ACCEL_ADDR, ACCEL_Y_HIGH) << 8) | readRegister(ACCEL_ADDR, ACCEL_Y_LOW));
    *z = (int16_t)((readRegister(ACCEL_ADDR, ACCEL_Z_HIGH) << 8) | readRegister(ACCEL_ADDR, ACCEL_Z_LOW));

    *x /= CONVERSION;
    *y /= CONVERSION;
    *z /= CONVERSION;
}

void readMagnetometer(int16_t *x, int16_t *y, int16_t *z) 
{
    *x = (int16_t)((readRegister(MAG_ADDR, MAG_X_HIGH) << 8) | readRegister(MAG_ADDR, MAG_X_LOW));
    *y = (int16_t)((readRegister(MAG_ADDR, MAG_Y_HIGH) << 8) | readRegister(MAG_ADDR, MAG_Y_LOW));
    *z = (int16_t)((readRegister(MAG_ADDR, MAG_Z_HIGH) << 8) | readRegister(MAG_ADDR, MAG_Z_LOW));
}

int main() {
    stdio_init_all();

    // Initialize GPIO
    i2c_init(i2c0, 100000);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(SCL_PIN);
    gpio_pull_up(SDA_PIN);

    // Initialize accelerometer settings
    configureRegister(ACCEL_ADDR, CTRL_REG1_A, 0x57);
    configureRegister(ACCEL_ADDR, CTRL_REG4_A, 0x30);

    // Initialize magnetometer settings
    configureRegister(MAG_ADDR, MR_REG_M, 0x00);

     // Initialize Kalman filters for accelerometer and magnetometer
    KalmanFilter accelFilter, magFilter;
    initKalmanFilter(&accelFilter, 0.01, 0.1);
    initKalmanFilter(&magFilter, 0.01, 0.1);

    while (1) {
        int16_t accelX, accelY, accelZ;
        readAccelerometer(&accelX, &accelY, &accelZ);

        int16_t MagX, MagY, MagZ;
        readMagnetometer(&MagX, &MagY, &MagZ);

        int16_t noisyAccelData[3] = {accelX, accelY, accelZ};
        updateKalmanFilter(&accelFilter, noisyAccelData);

        int16_t noisyMagData[3] = {MagX, MagY, MagZ};
        updateKalmanFilter(&magFilter, noisyMagData);

        float heading = atan2(magFilter.x[1], magFilter.x[0]);
        heading *= 180.0 / M_PI;

        if(heading < 0) {
            heading += 360;
        }

        printf("Accelerometer: x=%d, y=%d, z=%d\n", accelFilter.x[0], accelFilter.x[1], accelFilter.x[2]);
        printf("Magnetometer: x=%d, y=%d, z=%d, heading=%.2f deg\n\n", magFilter.x[0], magFilter.x[1], magFilter.x[2], heading);

        sleep_ms(1000);
    }

    return 0;
}