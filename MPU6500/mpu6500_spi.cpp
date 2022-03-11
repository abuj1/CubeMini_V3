 
#include <mbed.h>
#include "mpu6500_spi.h"
#include "mpu6000_registermap.h"
 
mpu6500_spi::mpu6500_spi(SPI& _spi, PinName _cs) : spi(_spi), cs(_cs) {}
 

bool mpu6500_spi::init_inav(){
 unsigned int response;
    spi.format(8,0);
    spi.frequency(1000000);
		ThisThread::sleep_for(chrono::milliseconds(150));
    write2spi(0x6B,0x80);
		ThisThread::sleep_for(chrono::milliseconds(150));
    write2spi(0x68,0x07);
		ThisThread::sleep_for(chrono::milliseconds(15));
    write2spi(0x6B,0x00);
		ThisThread::sleep_for(chrono::milliseconds(15));
    write2spi(0x6B,0x01);
		ThisThread::sleep_for(chrono::milliseconds(15));
    write2spi(0x1B,0x00);
		ThisThread::sleep_for(chrono::milliseconds(15));
    write2spi(0x1C,0x00);
		ThisThread::sleep_for(chrono::milliseconds(15));
    write2spi(0x1A,0x03);
		ThisThread::sleep_for(chrono::milliseconds(15));
    write2spi(0x19,0x00);
		ThisThread::sleep_for(chrono::milliseconds(15));
    write2spi(0x37,0 << 7 | 0 << 6 | 0 << 5 | 1 << 4 | 0 << 3 | 0 << 2 | 1 << 1 | 0 << 0);
		ThisThread::sleep_for(chrono::milliseconds(15));
    write2spi(0x38,1<<0);
		ThisThread::sleep_for(chrono::milliseconds(15));
	write2spi(MPUREG_CONFIG,BITS_DLPF_CFG_42HZ);
		set_acc_scale(BITS_FS_2G);
        set_gyro_scale(BITS_FS_250DPS);
		return true;
    
}



int mpu6500_spi::enableInterrupt()
{
    unsigned int response;

    //ENABLE INTERRUPTS
    select();
    response=spi.write(MPUREG_INT_ENABLE);
    response=spi.write(0x01);
    deselect();
    return 0;
}
 
unsigned int mpu6500_spi::set_acc_scale(int scale){
    unsigned int temp_scale;
    select();
    spi.write(MPUREG_ACCEL_CONFIG);
    spi.write(scale);  
    deselect();    
    switch (scale){
        case BITS_FS_2G:
            acc_divider = 1670.13251;
        break;
        case BITS_FS_4G:
            acc_divider = 835.06625;
        break;
        case BITS_FS_8G:
            acc_divider = 417.5331294;
        break;
        case BITS_FS_16G:
            acc_divider =  208.766564;
        break;   
    }
    ThisThread::sleep_for(chrono::milliseconds(10));
    select();
    temp_scale=spi.write(MPUREG_ACCEL_CONFIG|READ_FLAG);
    temp_scale=spi.write(0x00);  
    deselect();
    switch (temp_scale){
        case BITS_FS_2G:
            temp_scale=2;
        break;
        case BITS_FS_4G:
            temp_scale=4;
        break;
        case BITS_FS_8G:
            temp_scale=8;
        break;
        case BITS_FS_16G:
            temp_scale=16;
        break;   
    }
    return temp_scale;
}
 
unsigned int mpu6500_spi::set_gyro_scale(int scale){
    unsigned int temp_scale;
    select();
    spi.write(MPUREG_GYRO_CONFIG);
    spi.write(scale);  
    deselect();    
    switch (scale){
        case BITS_FS_250DPS:
            gyro_divider = 7509.87;
        break;
        case BITS_FS_500DPS:						// now it's not dps its RPS (rads per second)
            gyro_divider = 3754.94;			// corresponds 2^15/500*180/pi
        break;
        case BITS_FS_1000DPS:
            gyro_divider = 1877.468;
        break;
        case BITS_FS_2000DPS:
            gyro_divider = 938.73;
        break;   
    }
    ThisThread::sleep_for(chrono::milliseconds(10));
    select();
    temp_scale=spi.write(MPUREG_GYRO_CONFIG|READ_FLAG);
    temp_scale=spi.write(0x00);  
    deselect();
    switch (temp_scale){
        case BITS_FS_250DPS:
            temp_scale=250;
        break;
        case BITS_FS_500DPS:
            temp_scale=500;
        break;
        case BITS_FS_1000DPS:
            temp_scale=1000;
        break;
        case BITS_FS_2000DPS:
            temp_scale=2000;
        break;   
    }
    return temp_scale;
}
 
unsigned int mpu6500_spi::whoami(){
    unsigned int response;
    select();
    response=spi.write(MPUREG_WHOAMI|READ_FLAG);
    response=spi.write(0x00);
    deselect();
    return response;
}
 
float mpu6500_spi::readAcc(int axis){
    uint8_t responseH,responseL;
    int16_t bit_data;
    float data;
    select();
    switch (axis){
        case 0:
        responseH=spi.write(MPUREG_ACCEL_XOUT_H | READ_FLAG);
        break;
        case 1:
        responseH=spi.write(MPUREG_ACCEL_YOUT_H | READ_FLAG);
        break;
        case 2:
        responseH=spi.write(MPUREG_ACCEL_ZOUT_H | READ_FLAG);
        break;
    }
    responseH=spi.write(0x00);
    responseL=spi.write(0x00);
    bit_data=((int16_t)responseH<<8)|responseL;
    data=(float)bit_data;
    data=data/acc_divider;
    deselect();
    return data;
}
 int16_t mpu6500_spi::readAcc_raw(int axis){
    uint8_t responseH,responseL;
    int16_t bit_data;
    float data;
    select();
    switch (axis){
        case 0:
        responseH=spi.write(MPUREG_ACCEL_XOUT_H | READ_FLAG);
        break;
        case 1:
        responseH=spi.write(MPUREG_ACCEL_YOUT_H | READ_FLAG);
        break;
        case 2:
        responseH=spi.write(MPUREG_ACCEL_ZOUT_H | READ_FLAG);
        break;
    }
    responseH=spi.write(0x00);
    responseL=spi.write(0x00);
    bit_data=((int16_t)responseH<<8)|responseL;
    deselect();
    return bit_data;
}

void mpu6500_spi::readAcc(void){
    uint8_t responseHx,responseLx,responseHy,responseLy,responseHz,responseLz;
    int16_t bit_data;
    select();
    responseHx=spi.write(MPUREG_ACCEL_XOUT_H | READ_FLAG);
    responseHx=spi.write(0x00);
    responseLx=spi.write(0x00);
    responseHy=spi.write(0x00);
    responseLy=spi.write(0x00);
    responseHz=spi.write(0x00);
    responseLz=spi.write(0x00);
		deselect();
		wait_us(1);
    bit_data=((int16_t)responseHx<<8)|responseLx;
    accX = (float)bit_data/acc_divider;
    accX_raw = bit_data;
    bit_data=((int16_t)responseHy<<8)|responseLy;
    accY = (float)bit_data/acc_divider;
    accY_raw = bit_data;
    bit_data=((int16_t)responseHz<<8)|responseLz;
    accZ = (float)bit_data/acc_divider;
    accZ_raw = bit_data;
    
}
void mpu6500_spi::readAccTempGyro(void){
    uint8_t response[16];
    int16_t bit_data;
    select();
		response[0] = spi.write(MPUREG_ACCEL_XOUT_H | READ_FLAG);
		for(uint8_t k=0;k<14;k++)
        response[k]=spi.write(0x00);
    deselect();
		wait_us(1);
    bit_data=((int16_t)response[0]<<8)|response[1];
    accY = (float)bit_data/acc_divider;									// here is the rotation/alignment sensorframe 2 copter frame!!!
    bit_data=((int16_t)response[2]<<8)|response[3];
    accX = -(float)bit_data/acc_divider;
    bit_data=((int16_t)response[4]<<8)|response[5];
    accZ = (float)bit_data/acc_divider;
	//------------------------------
		bit_data=((int16_t)response[8]<<8)|response[9];
    gyroY = (float)bit_data/gyro_divider;
		bit_data=((int16_t)response[10]<<8)|response[11];
    gyroX = -(float)bit_data/gyro_divider;
		bit_data=((int16_t)response[12]<<8)|response[13];
    gyroZ = (float)bit_data/gyro_divider;
	
    
}
 
float mpu6500_spi::readGyro(int axis){
    uint8_t responseH,responseL;
    int16_t bit_data;
    float data;
    select();
    switch (axis){
        case 0:
        responseH=spi.write(MPUREG_GYRO_XOUT_H | READ_FLAG);
        break;
        case 1:
        responseH=spi.write(MPUREG_GYRO_YOUT_H | READ_FLAG);
        break;
        case 2:
        responseH=spi.write(MPUREG_GYRO_ZOUT_H | READ_FLAG);
        break;
    }
    responseH=spi.write(0x00);
    responseL=spi.write(0x00);
		deselect();
    bit_data=((int16_t)responseH<<8)|responseL;
    data=(float)bit_data;
    data=data/gyro_divider;
    
    return data;
}
void mpu6500_spi::readGyro(void){
    uint8_t responseHx,responseLx,responseHy,responseLy,responseHz,responseLz;
    int16_t bit_data;
    float data;
    select();
    responseHx=spi.write(MPUREG_GYRO_XOUT_H | READ_FLAG);
    responseHx=spi.write(0x00);
    responseLx=spi.write(0x00);
    responseHy=spi.write(0x00);
    responseLy=spi.write(0x00);
    responseHz=spi.write(0x00);
    responseLz=spi.write(0x00);
		deselect();
    bit_data=((int16_t)responseHx<<8)|responseLx;
    gyroX = (float)bit_data/gyro_divider;
    bit_data=((int16_t)responseHy<<8)|responseLy;
    gyroY = (float)bit_data/gyro_divider;
    bit_data=((int16_t)responseHz<<8)|responseLz;
    gyroZ = (float)bit_data/gyro_divider;
   
}
 
int16_t mpu6500_spi::read_temp(){
    uint8_t responseH,responseL;
    int16_t bit_data;
    float data;
    select();
    responseH=spi.write(MPUREG_TEMP_OUT_H | READ_FLAG);
    responseH=spi.write(0x00);
    responseL=spi.write(0x00);
    bit_data=((int16_t)responseH<<8)|responseL;
    /*data=(float)bit_data;
    data=(data/340)+36.53;
    deselect();
    return data;*/
	return bit_data;
}
 
int mpu6500_spi::calib_acc(int axis){
    uint8_t responseH,responseL,calib_data;
    int temp_scale;
    //READ CURRENT ACC SCALE
    select();
    responseH=spi.write(MPUREG_ACCEL_CONFIG|READ_FLAG);
    temp_scale=spi.write(0x00);  
    deselect();
    ThisThread::sleep_for(chrono::milliseconds(10));
    set_acc_scale(BITS_FS_8G);
    ThisThread::sleep_for(chrono::milliseconds(10));
    //ENABLE SELF TEST
    select();
    responseH=spi.write(MPUREG_ACCEL_CONFIG);
    temp_scale=spi.write(0x80>>axis);  
    deselect();
    ThisThread::sleep_for(chrono::milliseconds(10));
    select();
    responseH=spi.write(MPUREG_SELF_TEST_X|READ_FLAG);
    switch(axis){
        case 0:
            responseH=spi.write(0x00);
            responseL=spi.write(0x00);
            responseL=spi.write(0x00);
            responseL=spi.write(0x00);
            calib_data=((responseH&11100000)>>3)|((responseL&00110000)>>4);
        break;
        case 1:
            responseH=spi.write(0x00);
            responseH=spi.write(0x00);
            responseL=spi.write(0x00);
            responseL=spi.write(0x00);
            calib_data=((responseH&11100000)>>3)|((responseL&00001100)>>2);
        break;
        case 2:
            responseH=spi.write(0x00);
            responseH=spi.write(0x00);
            responseH=spi.write(0x00);
            responseL=spi.write(0x00);
            calib_data=((responseH&11100000)>>3)|((responseL&00000011));
        break;
    }
    deselect();
    ThisThread::sleep_for(chrono::milliseconds(10));
    set_acc_scale(temp_scale);
    return calib_data;
} 
 
void mpu6500_spi::select() {
    //Set CS low to start transmission (interrupts conversion)
    cs = 0;
}
void mpu6500_spi::deselect() {
    //Set CS high to stop transmission (restarts conversion)
    cs = 1;
}

void mpu6500_spi::write2spi(uint8_t reg,uint8_t val){
	  unsigned int response;
		cs = 0;
    response = spi.write(reg);
    response = spi.write(val);
    cs = 1;
}