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

int main()
{
    
    // Chip must be deselected
    cs = 1;

    // Setup the spi for 8 bit data, high steady state clock,
    // second edge capture, with a 1MHz clock rate
    spi.format(8, 0);
    spi.frequency(500000);

    // Select the device by seting chip select low
    cs = 0;

    // Send 0x8f, the command to read the WHOAMI register
    spi.write(0x75);

    // Send a dummy byte to receive the contents of the WHOAMI register
    int whoami = spi.write(0x01);
    printf("WHOAMI register = 0x%X\n", whoami);

    // Deselect the device
    cs = 1;
    
    
    
    mpu.init_inav();
    //printf("mpu_init_result: %d\n", mpu_init_result);
    thread_sleep_for(2000);
    
    // Works 
    unsigned int mpu_WhomAmI = mpu.whoami();
    printf("mpu_WhomAmI: %d\n", mpu_WhomAmI);
    thread_sleep_for(2000);

    // Send 0x8f, the command to read the WHOAMI register
    spi.write(0x75);
    whoami = spi.write(0x01);
    printf("WHOAMI register = 0x%X\n", whoami);

    
    int16_t Temperature;
    
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
        

        /*
        Temperature = mpu.read_temp();
        printf("Temperature %d\n", Temperature);
        */

    }
//Are commits working?
//printf("Just to check if commits work")


    /*
            while (true) {
                led = !led;
                thread_sleep_for(BLINKING_RATE_MS);
            }
    */

}
