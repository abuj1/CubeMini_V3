#include "ControllerLoop.h"
#include "mbed.h"
#include "platform/mbed_thread.h"
#include "mpu6500_spi.h"                // IMU Library
#include "IIR_filter.h"
#include "ThreadFlag.h"

using namespace std;

// PI Values
#define PI 3.1415927f
#define pi 3.1415927f

SPI spi(PA_12, PA_11, PA_1); // mosi, miso, sclk
DigitalOut cs(PB_0); // Chip Select Pin

mpu6500_spi mpu(spi, PB_0);
float Acc = 0.0f;

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

// Accelerometer and Gyroscope Filters
IIR_filter FilterAccX(t, Ts, 1.0f);
IIR_filter FilterAccY(t, Ts, 1.0f);
IIR_filter FilterGyro(t, Ts, t);

// contructor for controller loop
ControllerLoop::ControllerLoop(float Ts) : thread(osPriorityHigh,4096)
{
    this->Ts = Ts;
    ti.reset();
    ti.start();
    }

// decontructor for controller loop
ControllerLoop::~ControllerLoop() {}

// ----------------------------------------------------------------------------
// this is the main loop called every Ts with high priority
void ControllerLoop::loop(void){
    while(1)
        {
        ThisThread::flags_wait_any(threadFlag);
        // THE LOOP ------------------------------------------------------------
        if(0)//!is_initialized)
            {
            find_index();
            if(0)//index1.positionAtIndexPulse != 0 && index2.positionAtIndexPulse != 0) 
                is_initialized=true;
            }
        else
            {
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

                Cuboid_Angle_Radians =  -1*atan2(-FilterAccX(AccX_g), FilterAccY(AccY_g)) + 0.7854f + FilterGyro(GyroZ_RadiansPerSecond);
                Cuboid_Angle_Degrees = Cuboid_Angle_Radians*180.0f/pi;

                thread_sleep_for(7);
                    // Print Data // Printing Rate (in this case) is every Ts*100 = 0.007*100 = every 0.7 seconds
                if(++k >= 10) {
                    k = 0;
                    printf("ACuboid_Angle_Degrees %0.6f\n", Cuboid_Angle_Degrees);
                    }

                    }
            
        }// endof the main loop
}

void ControllerLoop::sendSignal() {
    thread.flags_set(threadFlag);
}
void ControllerLoop::start_loop(void)
{
    thread.start(callback(this, &ControllerLoop::loop));
    ticker.attach(callback(this, &ControllerLoop::sendSignal), Ts);
}

float ControllerLoop::pos_cntrl(float d_phi)
{
   
   // write position controller here
   return 0.0;
    }

void ControllerLoop::init_controllers(void)
{
    // set values for your velocity and position controller here!
    
    
}
// find_index: move axis slowly to detect the zero-pulse
void ControllerLoop::find_index(void)
{
    // use a simple P-controller to get system spinning, add a constant current to overcome friction
    
}
    
void ControllerLoop::reset_pids(void)
{
    // reset all cntrls.
}