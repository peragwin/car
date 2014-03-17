

#include "mbed.h"
#include <stdio.h>
//#include "linescan.h"

#define TURN_LEFT 350
#define TURN_RIGHT 350
#define TURN_FORWARD 1500
#define TURN_MAXTURN 300

#define NUM_SCANS 13
#define LINE_SCANNER_SCALE 1.0f/128.0f
#define PI 3.14159


using namespace std;

static const float eps = 1e-8;


Serial pc(USBTX,USBRX);
//same pinout order right to left as on power board
DigitalOut brake(PTD4); // right most pin / 2 away from edge
PwmOut leftMotor(PTA12); //adjacent
PwmOut rightMotor(PTA5);  // skips 1

PwmOut servo(PTA13); //left most, and on other socket

PwmOut cam_ck(PTC9);
DigitalOut cam_si(PTD0);
AnalogIn cam_ao(PTB0);


void gaussian_filter(int data[][NUM_SCANS], int output[][NUM_SCANS]){
	// using sigma1 = 1, sigma2 = 2
	
	int sig1 = 1;
	int sig2 = 2;
	int i, j,k,l;
	float convsum;


	for (j = 0; j < NUM_SCANS; j++){
		for (i = 0; i < 128; i++){
			convsum = 0;
		
			for (k = -2; k < 3; k++){
				for (l = -4; l < 5; l++){
					int il = i + l;
					int jk = j + k;
					
					if (jk < 0){
						jk += NUM_SCANS;
						
					}
					else if (jk > 12) {
						jk -= NUM_SCANS;
						
					}
					if (il < 0){
						il += 128;
					}
					else if (il > 127){
						il -= 128;
					}
					
					float elem = data[il][jk];
					
					//inner most 2d convolution loop
					//printf("\t%d\t%d ", i + l, j + k);
					convsum += 1.0f / 12.0f * exp(-0.5f * (float(k*k)  + float(l*l) / 4.0f)) * elem;
					if (convsum < 0){
						printf("wtf");
						convsum = 0;
					}
					
				}
			}
			output[i][j] = int(convsum + 0.5);
			//printf("%d, ", output[i][j]);
		}
	}


	return;

}

int **differential(int** data, int n){

	if (n == 0)
		return data;

	int i, j;
	int *ndifs;
	int **diffs;
	diffs = new int *[NUM_SCANS];
	for (i = 0; i < NUM_SCANS; i++){
		ndifs = new int[128];
		for (j = 0; j < 128; j++){
			ndifs[j] = 0;
		}
		ndifs[0] = data[0][i]-data[127][i]; //wrap around to the end for the first element so theres no untouched data
		for (j = 1; j < 128; j++){
			ndifs[j] = data[j][i] - data[j - 1][i];
		}

		diffs[i] = ndifs;
	}

	return differential(diffs, n - 1);

}

int *argmax(int data[][NUM_SCANS]){

	int i, j, tmp,tmp_i;
	int *max = new int[NUM_SCANS];

	for (i = 0; i < NUM_SCANS; i++){
		tmp = 0;
		tmp_i = 0;
		for (j = 0; j < 128; j++){
			if (data[j][i] >= tmp){
				tmp = data[j][i];
				tmp_i = j;
			}
		}
		max[i] = tmp_i;
	}
	return max;
}

int findmax(int data[][NUM_SCANS]){
	int i, j, mx;
	mx = 0;
	for (i = 0; i < 128; i++){
		for (j = 0; j < NUM_SCANS; j++){
			if (data[i][j] > mx){
				mx = data[i][j];
			}
		}
	}
	return mx;
}
int findmin(int data[][NUM_SCANS]){
	int i, j, mx;
	mx = 65536;
	for (i = 0; i < 128; i++){
		for (j = 0; j < NUM_SCANS; j++){
			if (data[i][j] < mx){
				mx = data[i][j];
			}
		}
	}
	return mx;
}

bool equalz(float a, float b){
	if (abs(a - b) < eps){
		return true;
	}
	else {
		return false;
	}
}

float * leastsq(int data[]){
	int i;
	float r, b, m;
	float meanx = 0;
	float meany = 0;
	float varx = 0, stdx =0;  
	float vary = 0, stdy =0;
	float sumxy = 0, sumy = 0, sumx = 0, sumyy = 0, sumxx = 0;

	float *ret;
	ret = new float[2];

	for (i = 0; i < NUM_SCANS; i++){
		meany += data[i] / float(NUM_SCANS);
		meanx += i / float(NUM_SCANS);
	}
	for (i = 0; i < NUM_SCANS; i++){
		varx += (meanx - i)*(meanx - i) / float(NUM_SCANS);
		vary += (meany - data[i])*(meany - data[i]) / float(NUM_SCANS);
		sumx += i;
		sumy += data[i];
		sumyy += data[i] * data[i];
		sumxx += i*i;
		sumxy += i*data[i];
	}
	stdx = sqrt(varx);
	stdy = sqrt(vary);
	r = (float(NUM_SCANS) * sumxy - sumx*sumy) / sqrt((float(NUM_SCANS) * sumxx - sumx*sumx)*(float(NUM_SCANS) * sumyy - sumy*sumy));

	m = r*stdy / stdx;
	b = meany - m*meanx;

	ret[0] = m;
	ret[1] = b;
	return ret;
}

float error_dist(float *y1, float *y2){
	int i;
	float ed = 0;
	for (i = 0; i < NUM_SCANS; i++){
		ed += abs(y1[i] - y2[i]);
	}
	return ed;
}

float error_dist(float *y1, int *y2){
	int i;
	float ed = 0;
	for (i = 0; i < NUM_SCANS; i++){
		ed += abs(y1[i] - y2[i]);
	}
	return ed;
}

float error_dist(int *y, float m, float b){
	int i;
	float ed = 0;
	for (i = 0; i < NUM_SCANS; i++){
		ed += abs(y[i] - m*i - b);
	}
	return ed;
}


float * _minim(int data[], float m, float b){
	float step_size = 0.5f;
	int loop = 2;
	int inc = -1;
	float err;
	float *ret = new float[2];
	float olderr = error_dist(data, m, b);
	//increase by stepsize until err > olderr, then decrease doing the same thing
	while (loop > 0){
		m += inc*step_size;
		err = error_dist(data, m, b);
		if (equalz(err, olderr))
			break;
		if (err > olderr){
			loop--;
			inc = -inc;
		}
		else {
			step_size *= .75;
		}
		olderr = err;
	}
	loop = 2;
	step_size = .5;
	inc = -1;
	while (loop > 0){
		b += inc*step_size;
		err = error_dist(data, m, b);
		if (equalz(err, olderr))
			break;
		if (err > olderr){ 
			loop--;
			inc = -inc;
		}
		else {
			step_size *= .75;
		}
		olderr = err;
	}
	ret[0] = m;
	ret[1] = b;
	return ret;
}

float *leastabs(int data[], float m, float b){
	int loop = 125; //some big enough number for convergence. 100ish or less should be fine with least squares as the initial guess
	float m_est;
	float b_est;
	float *newest;
	float *ret = new float[2];
	while (loop>0){
		m_est = m;
		b_est = b;
		newest = _minim(data, m, b);
		m = newest[0];
		b = newest[1];
		if (equalz(m_est, m) && equalz(b_est, b)){
			ret[0] = m;
			ret[1] = b;
			printf("yay, %d tries. %f, %f", loop, m, b);
			return ret;
		}
		else{
			loop--;
		}
	}
	ret[0] = m;
	ret[1] = b;
	return ret;
}

float find_angle(float m, float b, float d_x){
	float angle, alpha;

	angle = - LINE_SCANNER_SCALE / d_x * m;
	alpha = tan(-angle);

	return alpha/PI;
}




void initPWM(){
	// FIX ME - something weird is going on with the pwms
	servo.period_ms(20); 
	leftMotor.period_ms(20);
	rightMotor.period_ms(20);
	
	cam_ck.period(0.00005);
	cam_ck.write(0.5);
	
}
	
int goForward(float speed){
	brake = 0;
	leftMotor.write(speed);
	rightMotor.write(speed);
	//brake = 0;
	return 1;
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
	int dev = 0;
	if( per > 0 ){
			dev = per*TURN_RIGHT;
	} else{
		dev = per*TURN_LEFT;
	}
	servo.pulsewidth_us(TURN_FORWARD+dev);
}


int main() {
   // nbprint_setup();
		
	initPWM();
	
		//int isGoingFwd = goForward(0.30f); 
	int linescan[128];
	for (int i =0; i<128; i++){
			linescan[i] = 0;
	}
	int linemax = 0;
	float turnto;
		//Linescan camera(cam_si,cam_ck,cam_ao);
	while(1){
		wait(0.01); //10ms integration
		cam_si = 1;
		wait(.001); // serial initiation pulse
		cam_si = 0;
		// read from analog sensor
		for (int i = 0; i<128;i++){
			int aout = cam_ao.read_u16();
			//printf("pixel %d", aout);
			//pc.printf("hm %d \n", aout);
			linescan[i] = aout;//.5*linescan[i] + aout;
			if (linescan[i] >= linescan[linemax]){
				linemax = i;
			}
			wait(0.00005); //one period of the cameras clock
		}
		//pc.printf("hm --------------------------------------------");
		int max = linescan[linemax];
		int threshold = .75*max;
		
		for (int i = 0; i<128; i++){
			if (i+10 < 128 && linescan[i] < threshold && linescan[i+10] < threshold){
				//if (i+5 > 64){
				//	turnto = 1 - (i+5)/64.0;
				//}
				//else{
				//	turnto = (i+5-64)/64.0;
				//}
				turnto = (i+5-32)/32.0;
				turn(turnto);
				break;
			}
		} 
		//turnto = (linemax-127)/128.0;
		//turn(turnto);
		/*
		for (int i =0; i<128; i++){
			pc.printf("index %d : %d\n", i, linescan[i]);
		} */
	}
		/*while(1){
			turn(1.0f);
			int isGoingFwd = goForward(0.30f);
			wait(4);
			turn(-1.0f);
			//passiveBrake();
  		wait(4);
			passiveBrake();  
			turn(0.0f);
			wait(4);
		}*/
}

