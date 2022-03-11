/* mbed Microcontroller Library
 * Copyright (c) 2019 ARM Limited
 * SPDX-License-Identifier: Apache-2.0
 */

#include "mbed.h"
#include "platform/mbed_thread.h"
#include "mpu6500_spi.h"                // IMU Library

// Blinking rate in milliseconds
#define BLINKING_RATE_MS                                                    1000


SPI spi(PA_12, PA_11, PA_1); // mosi, miso, sclk
DigitalOut cs(PB_0); // Chip Select Pin

mpu6500_spi mpu(spi, PB_0);


// -------------------------------
// Initialization of Values and Variables
// -------------------------------

// Sample time of main loops
float Ts = 0.007; // Around 143 Hz. Do not change. This is the lowest value possible if printing to a serial connection is needed.

// MPU 6050 Variables - Acceleration and Gyroscope Raw and Converted Data Variables
int16_t AccX_Raw, AccY_Raw, AccZ_Raw;
int16_t GyroX_Raw, GyroY_Raw, GyroZ_Raw;
double AccX_g, AccY_g, AccZ_g;
double GyroX_Degrees, GyroY_Degrees, GyroZ_Degrees, GyroZ_RadiansPerSecond;

// printf Variable
int k = 0;


// -------------------------------
//  Variables: Here define variables such as gains etc.
// -------------------------------

// Accelerometer and Gyroscope


// Cube Angle and Speed Variables
double Cuboid_Angle_Radians, Cuboid_Angle_Degrees;
double Cuboid_Angle_Speed_Degrees = 0.0;

// Low pass filter variables
float t = 0.5f;

// Flywheel Position and Velocity variables
double Velocity_Input_Voltage = 0.0f;
double Velocity, Velocity_Voltage, Velocity_rpm, Velocity_Voltage_Read;


int main()
{

    mpu.init_inav();
    //printf("mpu_init_result: %d\n", mpu_init_result);
    thread_sleep_for(500);
    
    bool mpu6050TestResult = mpu.testConnection();
    if(mpu6050TestResult) {
        printf("MPU6050 test passed \n\r");
    } else {
        printf("MPU6050 test failed \n\r");
    }	
    mpu.configuration();
    
    
    float Acc;
    // Initialise the digital pin LED1 as an output
    DigitalOut led(LED1);
    while (true) {
        //printf("Hello World!\n");
        thread_sleep_for(500);
        
        mpu.readAcc();
        Acc = mpu.accZ;
        //Acc = 1.234f;
        printf("Acc %0.6f\n", Acc);


    }

    /*
            while (true) {
                led = !led;
                thread_sleep_for(BLINKING_RATE_MS);
            }
    */

}
