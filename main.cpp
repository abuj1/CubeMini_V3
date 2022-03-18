/* mbed Microcontroller Library
 * Copyright (c) 2019 ARM Limited
 * SPDX-License-Identifier: Apache-2.0
 */

#include "mbed.h"
#include "platform/mbed_thread.h"
#include "mpu6500_spi.h"                // IMU Library
#include "IIR_filter.h"
#include "EncoderCounter.h"
#include "DiffCounter.h"
#include "LinearCharacteristics.h"
// Blinking rate in milliseconds
#define BLINKING_RATE_MS                                                    1000
// PI Values
#define PI 3.1415927f
#define pi 3.1415927f



SPI spi(PA_12, PA_11, PA_1); // mosi, miso, sclk
DigitalOut cs(PB_0); // Chip Select Pin

// -------------------------------
// Analog/Digitsl I/O Definitions
// -------------------------------
//AnalogIn Velocity_Voltage_Input(p5); // Velocity Input as Analogue Signal from Escon
//DigitalOut Pin_3V3(p30);             // Defining P30 as 3V3 Pin for internal use. Please do not modify.
InterruptIn Button(PA_10);        // User Button Interrput
DigitalOut Escon_Enable(PB_1);
//DigitalOut GLED(p21);
AnalogOut Voltage(PA_4);

// User Button
int Button_Status = 0;                  // User Button Status
Timer T_Button;                         // define timer for button


mpu6500_spi mpu(spi, PB_0);
float Acc = 0.0f;

// -------------------------------
// Initialization of Values and Variables
// -------------------------------

// Sample time of main loops
float Ts = 0.007; // Around 143 Hz. Do not change. This is the lowest value possible if printing to a serial connection is needed.

EncoderCounter Encoder_counter1(PA_8, PA_9);    // initialize counter on A: PA_8, B: PA_9
DiffCounter diff(0.01,Ts);              // discrete differentiate, based on encoder data

//LCs
LinearCharacteristics CurrentToVoltage(-15.0f, 15.0f, 0.0f, 5.0f);

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

// -------------------------------
//  User-Defined Functions
// -------------------------------

// Controller loop (via interrupt)
// void updateControllers(void);           // controller loop (via interrupt)

// Accelerometer and Gyroscope Filters
IIR_filter FilterAccX(t, Ts, 1.0f);
IIR_filter FilterAccY(t, Ts, 1.0f);
IIR_filter FilterGyro(t, Ts, t);


// Interrupts
Ticker  ControllerLoopTimer;            // Interrupt for control loop
EventQueue *queue = mbed_event_queue(); // event queue

Ticker flipper;
DigitalOut led1(LED1);

void flip()
{
    led1 = !led1;
}

// USer Button
void pressed(void);                     // user Button pressed
void released(void);                    // user Button released


// -------------------------------
//  Functions
// -------------------------------

// Timers
//Timer Loop;
Timer Code_Timer;

void processInUserContext() {
    //printf("printing in a user context. counter = %d\r\n", counter);
    printf("Acc %0.6f\n", Acc);
}

void updateControllers()
{
        //printf("Hello World!\n");
        Acc = 1.234f;
        //queue->call(&processInUserContext); // defer the execution to a different context
}

// Flywheel Position and Velocity variables 
long double counts, Counts_Revs, Counts_Radians, Velocity_E;

double Code_Time;

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
    
    led1 = 1;

    //ControllerLoopTimer.attach(&updateControllers, 1ms);
    //flipper.attach(&flip, 2000ms); // the address of the function to be attached (flip) and the interval (2 seconds)
    //queue->dispatch_forever();

    Button.fall(&pressed);          // attach key pressed function
    Button.rise(&released);         // attach key pressed function

    diff.reset(0.0f,0.0f);

    while (1) {

        //Code_Timer.start();
        counts = Encoder_counter1.read();   // get counts from Encoder
        Counts_Revs = (Encoder_counter1.read())*(-1.0f/2048.0f)*(1.0f/4.0f);
        Counts_Radians = (Encoder_counter1.read())*(-1.0f/2048.0f)*(1.0f/4.0f)*(2.0f*pi);
        Velocity_E = diff(counts)*9.5493;           // motor velocity // The 2048.0f in defined in the Diff library. Fix this!


        //mpu.readAcc();
        //Acc = mpu.accZ;
        AccX_Raw = mpu.readAcc_raw(0);
        AccY_Raw = mpu.readAcc_raw(1);
        AccZ_Raw = mpu.readAcc_raw(2);
        GyroX_Raw = mpu.readGyro_raw(0);
        GyroY_Raw = mpu.readGyro_raw(1);
        GyroZ_Raw = mpu.readGyro_raw(2);

        // Aquire Raw Acceleration and Gyro Data form the IMU
        //mpu.getMotion6(&AccX_Raw, &AccY_Raw, &AccZ_Raw, &GyroX_Raw, &GyroY_Raw, &GyroZ_Raw);


        // -------------- Convert Raw data to SI Units --------------------

        //Convert Acceleration Raw Data to (ms^-2) - (Settings of +/- 4g)
        AccX_g = AccX_Raw / 8192.0f;
        AccY_g = AccY_Raw / 8192.0f;
        AccZ_g = AccZ_Raw / 8192.0f;

        //Convert Gyroscope Raw Data to Degrees per second
        GyroX_Degrees = GyroX_Raw / 32.768f;  // (2^15/1000 = 32.768)
        GyroY_Degrees = GyroY_Raw / 32.768f;  // (2^15/1000 = 32.768)
        GyroZ_Degrees = GyroZ_Raw / 32.768f;  // (2^15/1000 = 32.768)

        //Convert Gyroscope Raw Data to Degrees per second
        GyroZ_RadiansPerSecond = (GyroZ_Raw / 32.768f)* pi/180.0f;

        // ----- Combine Accelerometer Data and Gyro Data to Get Angle ------

        Cuboid_Angle_Radians =  -1*atan2(-FilterAccX(AccX_g), FilterAccY(AccY_g)) + FilterGyro(GyroZ_RadiansPerSecond); // + 0.7854f
        Cuboid_Angle_Degrees = Cuboid_Angle_Radians*180.0f/pi;


        // Output
        Voltage.write(0.1f);

        thread_sleep_for(7);
            // Print Data // Printing Rate (in this case) is every Ts*100 = 0.007*100 = every 0.7 seconds
        //Code_Timer.stop();
        //Code_Time = Code_Timer.read();
        //Code_Timer.reset();
        if(++k >= 10) {
            k = 0;
            //printf("ACuboid_Angle_Degrees %0.6f\n", Cuboid_Angle_Degrees);
            printf("Velocity_E %1.5Lf, Angle %0.6f\n", Velocity_E, Cuboid_Angle_Degrees);
            }
    }

}

//******************************************************************************
//------------------ Control Loop (called via interrupt) -----------------------
//******************************************************************************

//******************************************************************************
//------------------ User functions like buttons handle etc. -------------------
//******************************************************************************

//...
// start timer as soon as Button is pressed
void pressed()
{
    T_Button.start();
}
// Falling edge of button: enable/disable controller
void released()
{
    // readout, stop and reset timer
    float ButtonTime = T_Button.read();
    T_Button.stop();
    T_Button.reset();
    if(ButtonTime > 0.05f) {
        /*
        Button_Status = Button_Status + 1;
        if (Button_Status > 3.0f) {
            Button_Status = 1.0;
            
        }
        */
        Escon_Enable = !Escon_Enable;
        //bool B2_Status = Escon_Enable.read();
        //pc.printf("Button Status: %i\r\n", Button_Status);
        //printf("Button Status: %d\r\n", B2_Status);
    }
}
