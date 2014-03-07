#include "mbed.h"
//#include "nbprint.h"

#include "MPU6050/MPU6050.h"

DigitalOut red(LED1);
DigitalOut green(LED2);
DigitalOut blue(LED3);

//DigitalOut sda(PTD4);
//DigitalOut scl(PTA12);

//DigitalOut vcc(PTA5);
//DigitalOut gnd(PTA4);

MPU6050 gyro(PTB1,PTB2);

Serial pc(USBTX,USBRX);

int startgyro(){
	gyro.setI2CBypass(0);
	gyro.setSleepMode(0);
	pc.printf("Attempting to initialize a connection...");
	bool connected = 0;
	int i;
	while(!connected){
		connected = gyro.testConnection();
		if (i%1000 == 0){
			pc.printf(".");
			red = !red;
		}
		i++;
	}
	int *data;
	
	while(connected){
		gyro.getGyroRaw(data);
	
		pc.printf("Gyro data:\t %f \t %f \t %f",data[0],data[1],data[2]);
	}
	pc.printf("Connection failed! Terminating...");
	
	return 1;
}
