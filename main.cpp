

#include "mbed.h"
#include <stdio.h>
//#include <math.h>
//#include "linescan.h"

#define TURN_LEFT 350
#define TURN_RIGHT 350
#define TURN_FORWARD 1500
#define TURN_MAXTURN 300

#define NUM_SCANS 5
#define LINE_SCANNER_SCALE 0.075
#define PI 3.14159

#define PROPORTIONAL_PARAM 50
#define DIFFERENTIAL_PARAM .1

#define K_EXPOSURE (.005 / 32768.0)

using namespace std;

static const float eps = 1e-3;

Serial bt(PTC4,PTC3);
Serial pc(USBTX,USBRX);
//same pinout order right to left as on power board
DigitalOut brake(PTD4); // right most pin / 2 away from edge
PwmOut leftMotor(PTA4); //adjacent
PwmOut rightMotor(PTA5);  // skips 1

PwmOut servo(PTA13); //left most, and on other socket

PwmOut cam_ck(PTA1);
DigitalOut cam_si(PTD0);
AnalogIn cam_ao(PTB0);

InterruptIn vel_sensor(PTD3);

Ticker vel_sensing;
int vel_sensor_count = 0;
float vel_sensor_count_avg = 0;

float isGoingFwd = .1;

static float gaussian1d_kernel[7] = {  0.0702  ,  0.1311 ,   0.1907  ,  0.2161  ,  0.1907  ,  0.1311  ,  0.0702 };

void gaussian_filter(int data[], int output[]){
	// using sigma1 = 1, sigma2 = 2
	
	//int sig1 = 1;
	//int sig2 = 2;
	int i,l;
	float convsum;


	
		for (i = 0; i < 128; i++){
			convsum = 0;
		
			
				for (l = -3; l < 4; l++){
					int il = i + l;
					
					
					
					if (il < 0){
						il += 128;
					}
					else if (il > 127){
						il -= 128;
					}
					
					float elem = data[il];
					
					//inner most 2d convolution loop
					//printf("\t%d\t%d ", i + l, j + k);
					//convsum += elem * 3 * gaussian_kernel[1][l];   ///1.0f / 12.0f * exp(-0.5f * (float(k*k)  + float(l*l) / 4.0f)) * elem;
					convsum += elem * gaussian1d_kernel[l+3];
				}
			
			output[i] = int(convsum + 0.5);
			//printf("%d, ", output[i][j]);
		}
	


	return;

}


void differential(int data[], int diff[]){
	int i;
	
		
		for (i = 0; i < 128; i++){
			if (i < 4){
				diff[i] = 0;
			} else if (i > 124){
				diff[i] = 0;
			} else {
			//1st derivative - could be implemented in the filter stage
				diff[i] = data[i] - data[i-1];
			// 2nd derivative filter, 2nd order approximation, then 1st to compare
				//diff[i] =  data[(i - 1)] - 2 * data[i] + data[(i + 1)];
			}
		}
		//data[0] = 0;
		//data[127] = 0;
	return;
}

void findcenter(int data[], int ret[]){
	int j, htmp, h_i, l_i, ltmp, mean=0;
	float hv, lv;
	float var=0;
	//int *ret = new int[2]; //return center and certainty
	
	htmp = 0;
	ltmp = 66000;
	for(j = 2; j<126; j++){
		mean += data[j];
		if (data[j] >= htmp){
			htmp = data[j];
			h_i = j;
		}
		if(data[j] <= ltmp){
			ltmp = data[j];
			l_i = j;
		}
	}
	mean /= 124;
	//calculate variance
	//for (j=2; j<126;j++){
	//	var += (mean-data[j])*(mean-data[j]);
	//}
	//var /= 124;
	//var = sqrt(var); //stdev
	
	ret[0] = int((h_i + l_i)/2);
	
	int dif = abs(h_i - l_i);
	ret[1] = abs(10 - dif);
	
	//lv = abs(mean - data[l_i]) / var;
	//hv = abs(mean - data[h_i]) / var;
	//ret[1] = (lv+hv)/2;
	
	return;//n ret;
}


bool equalz(float a, float b){
	if (abs(a - b) < eps){
		return true;
	}
	else {
		return false;
	}
}


float find_angle(int max, float prev){
	float alpha, ret;


	//alpha = LINE_SCANNER_SCALE / d_x * m;
	alpha = 64 - max;

	
	//beta = LINE_SCANNER_SCALE / d_x * alpha;
	//angle = -atan(beta)/PI; 

	ret = (alpha/PROPORTIONAL_PARAM) + DIFFERENTIAL_PARAM*(ret-prev);   //-alpha/42 is good
	
	
	return ret;
}

void delay(int d){
	int i,j;
	for( i = 0; i<d; i++){
		for (j = 0; j<400; j++){}
	}
}

void initPWM(){
	//FIXED ///// FIX ME - something weird is going on with the pwms
	servo.period_ms(20); 
	leftMotor.period_ms(2);
	rightMotor.period_ms(2);
	
	cam_ck.period(0.00005);
	cam_ck.write(0.5);
	
}
	
float goForward(float speed){
	if (speed < 0){
		speed = 0;
	} 
	else if (speed > 1){
		speed = 1.0;
	}
	brake = 0;
	leftMotor.write(speed);
	rightMotor.write(speed);
	//brake = 0;
	return speed;
}

void leftMotorOn(){
	leftMotor.write(0.30f);
	rightMotor.write(0.0f);
	brake = 0;
}
void passiveBrake(){
	brake = 1;
}

// per should be percent to turn, between -1 and 1
void turn(float per){
	if (per > 1.0){
		per = 1.0;
	} else if (per < -1.0){
		per = -1.0;
	}
	int dev;
	if( per > 0 ){
			dev = per*TURN_RIGHT;
	} else{
		dev = per*TURN_LEFT;
	}
	servo.pulsewidth_us(TURN_FORWARD+dev);
}


void vel_sensor_counter(){
	vel_sensor_count+=4;
}



static float VEL_DESIRED_TICKS = .75;
#define VEL_KP 	.04
#define VEL_KD 	.02
#define VEL_P 	.15

#define TARGET_PWM	.1

void vel_sensing_tick()
{
	vel_sensor_count_avg = .5*vel_sensor_count_avg + .5*vel_sensor_count;
	//bt.printf("%f\n\r", vel_sensor_count_avg);
	
	float err = vel_sensor_count - VEL_DESIRED_TICKS;
	vel_sensor_count = 0;

	float err_avg = vel_sensor_count_avg - VEL_DESIRED_TICKS;

	float propcntl = TARGET_PWM - VEL_P * err;
	float adj = -VEL_KP * err;
	float adj_avg = VEL_KD * err_avg;
	
	float newGoingFwd = isGoingFwd+adj;
	//pc.printf("%f\n\r", adj);
	if (brake == 0){
		isGoingFwd = goForward(.8*propcntl+ .4*(isGoingFwd+adj - adj_avg) );
	}
}



int main() {
   // nbprint_setup();
		//bt.baud(9600);
//	while(1){
//				if (bt.readable()){
//					char msg = bt.getc();
//					if (msg){
//						break;
//					}
//				}
//	}			
	initPWM();
	
	//init velocity sensing
	vel_sensor.rise(&vel_sensor_counter);
	//vel_sensor.fall(&vel_sensor_counter);
	vel_sensing.attach(&vel_sensing_tick,0.02); //20ms
	
	
	isGoingFwd = goForward(0.1f);

	int linescan[128] = {};
	int altscan[128] = {};
	int linecenter=64;
	int tempcenter[2] = {};
		
	int i;

	float turnto = 0;

	float exposure_time = 0.005; //5ms to start
	int exposure = 0;
		

	while(1){
		
		cam_si = 1;
		wait(.00005);
		cam_si = 0; //flush camera buffer
		
	//	wait(.0009); //18 clock cycles extra before integration starts
		
//  first three lines will clear the buffer on the camera.
//	not exactly necesasry if using a ticker or some thread-like processing	
		wait(.010);
	//	wait(exposure_time); //10ms integration
		cam_si = 1;
		wait(.00005); // serial initiation pulse
		cam_si = 0;
		
		exposure = 0;
		// read from analog sensor
		for (i = 0; i<128;i++){
			int aout = cam_ao.read_u16();
			linescan[i] = aout;//.5*linescan[i] + aout;
			exposure += aout;
			//delay(10);
			//wait(0.0000125); //quarter period of the cameras clock (dunno, it works tho)
			wait(0.000012);
		}
		
		//auto exposure adjustment
		int exp_diff = 0x7FFF - exposure/128; //positive means too little exposure, neg is too much
		exposure_time *= 1 + 25*(K_EXPOSURE * exp_diff);
		if (exposure_time < .001){
			exposure_time = .001;
		} else if (exposure_time > .05){
			exposure_time = .05;
		}
		
		//bt.printf("%d\t%f\n\r",exp_diff, exposure_time);
		
		
		
		//gaussian_filter(linescan,altscan);

		differential(linescan,altscan);

		
		/*
		bt.printf("\r");
		for(i = 0; i<128;i+=2){
			if(linescan[i] > 1000){
				bt.printf("-");
			} else {
				bt.printf("_");
			}
		}*/
		
		
		findcenter(altscan,tempcenter);
		
		if(tempcenter[1] <= 10){ //this parameter will be finicky
			brake = 0;
			linecenter = tempcenter[0];
		} else {
			//brake = 1;
		}
		
	//	bt.printf("%d\n\r", tempcenter[1]);

//		pc.printf("%d\n\r", linecenter);

		turnto = find_angle(linecenter, turnto); //turnto in arguement is the term used for a differential
		//VEL_DESIRED_TICKS = 2.8 - 1.4 * fabs(turnto);
		
		if (turnto > 1){
			turnto = 1;
		} else if (turnto < -1) {
			turnto = -1;
		}
	  turn(turnto);

		
//		if (bt.readable()){
//			char msg = bt.getc();
//			if (msg == ' '){
//					brake=1;
//			} else if (msg == 'g'){
//				brake = 0;
//			}
//		}
	
		
	}

}

