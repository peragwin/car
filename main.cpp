

#include "mbed.h"
#include <stdio.h>
//#include "linescan.h"

#define TURN_LEFT 350
#define TURN_RIGHT 350
#define TURN_FORWARD 1500
#define TURN_MAXTURN 300

#define NUM_SCANS 5
#define LINE_SCANNER_SCALE 0.075
#define PI 3.14159


using namespace std;

static const float eps = 1e-3;


Serial pc(USBTX,USBRX);
//same pinout order right to left as on power board
DigitalOut brake(PTD4); // right most pin / 2 away from edge
PwmOut leftMotor(PTA4); //adjacent
PwmOut rightMotor(PTA5);  // skips 1

PwmOut servo(PTA13); //left most, and on other socket

PwmOut cam_ck(PTA1);
DigitalOut cam_si(PTD0);
AnalogIn cam_ao(PTB0);

static float gaussian_kernel[3][5] = { { 0.0370   , 0.0720  ,  0.0899  ,  0.0720   , 0.0370},
   { 0.0462  ,  0.0899  ,  0.1123 ,   0.0899  ,  0.0462 },
   { 0.0370  ,  0.0720 ,   0.0899  ,  0.0720 ,   0.0370 },};

static float gaussian1d_kernel[7] = {  0.0702  ,  0.1311 ,   0.1907  ,  0.2161  ,  0.1907  ,  0.1311  ,  0.0702 };

void gaussian_filter(int data[][NUM_SCANS], int output[][NUM_SCANS]){
	// using sigma1 = 1, sigma2 = 2
	
	//int sig1 = 1;
	//int sig2 = 2;
	int i, j,k,l;
	float convsum;


	for (j = 0; j < NUM_SCANS; j++){
		for (i = 0; i < 128; i++){
			convsum = 0;
		
			for (k = -1; k < 2; k++){
				for (l = -2; l < 3; l++){
					int il = i + l;
					int jk = j + k;
					
					if (jk < 0){
						jk += NUM_SCANS;
						
					}
					else if (jk >= NUM_SCANS) {
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
					convsum += elem * gaussian_kernel[k][l];   ///1.0f / 12.0f * exp(-0.5f * (float(k*k)  + float(l*l) / 4.0f)) * elem;
					
				}
			}
			output[i][j] = int(convsum + 0.5);
			//printf("%d, ", output[i][j]);
		}
	}


	return;

}
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

void differential(int data[][NUM_SCANS], int diff[][NUM_SCANS]){
	int i,j;
	for (j = 0; j < NUM_SCANS; j++){

		for (i = 0; i < 128; i++){
			if (i < 8){
				diff[i][j] = 0;
			} else if (i > 120){
				diff[i][j] = 0;
			} else {
			// 2nd derivative filter, 2nd order approximation, then 1st to compare
				diff[i][j] =  data[(i - 1)][j] - 2 * data[i][j] + data[(i + 1)][j];
			}
		}
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

void argmax(int data[][NUM_SCANS], int *max){

	int i, j, tmp,tmp_i;
	//int *max = new int[NUM_SCANS];

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
	return;
}

int argmax(int data[]){

	int j, tmp,tmp_i;
	//int *max = new int[NUM_SCANS];

		
		tmp = 0;
		tmp_i = 0;
		for (j = 2; j < 126; j++){
			if (data[j] >= tmp){
				tmp = data[j];
				tmp_i = j;
			}
		}
		return tmp_i;
	
	//return max;
}
int argmin(int data[]){

	int j, tmp,tmp_i;
	//int *max = new int[NUM_SCANS];

		
		tmp = 66000;
		tmp_i = 0;
		for (j = 2; j < 126; j++){
			if (data[j] <= tmp){
				tmp = data[j];
				tmp_i = j;
			}
		}
		return tmp_i;
	
	//return max;
}
void argmin(int data[][NUM_SCANS], int *min){

	int i,j, tmp,tmp_i;
	//int *max = new int[NUM_SCANS];

	for (i = 0; i < NUM_SCANS; i++){	
		tmp = 66000;
		tmp_i = 0;
		for (j = 2; j < 126; j++){
			if (data[j][i] <= tmp){
				tmp = data[j][i];
				tmp_i = j;
			}
		}
		min[i] = tmp_i;
	}
		return;
	
	//return max;
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

void leastsq(int data[], float *ret){
	int i;
	float r, b, m, tmp1, tmp2;
	float meanx = 0;
	float meany = 0;
	float varx = 0, stdx;  
	float vary = 0, stdy;
	float sumy = 0, sumyy = 0;
	float sumxy = 0;
	float sumx = 0;
	float sumxx = 0;

	//float *ret;
	//ret = new float[2];

	//for (i=1; i<= NUM_SCANS; i++){ 
		
	//}
	
	for (i = 1; i < NUM_SCANS; i++){
		meany += data[i-1] / float(NUM_SCANS);
		meanx += i / float(NUM_SCANS);
	}
	for (i = 1; i < NUM_SCANS; i++){
		varx += (meanx - i)*(meanx - i) / float(NUM_SCANS);
		vary += (meany - data[i-1])*(meany - data[i-1]) / float(NUM_SCANS);	
		sumy += data[i-1];
		sumyy += data[i-1] * data[i-1];
		sumx += i;
		sumxx += i*i;
		sumxy += i *data[i-1];
	}
	//pc.printf("%f\t%f\t%f\r",sumx, sumxx, sumxy);
	stdx = sqrt(varx);
	stdy = sqrt(vary);
	tmp1 = (NUM_SCANS * sumxy - sumx*sumy);
	tmp2 = (NUM_SCANS * sumxx - sumx*sumx)*(NUM_SCANS * sumyy - sumy*sumy);
	
	m = tmp1 / tmp2 * stdy / stdx;
	b = meany - m*meanx;

	ret[0] = m;
	ret[1] = b;
	//return ret;
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
			step_size *= .5;
			inc = -inc;
		}
		else {
			loop--;
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
				step_size *= .5;
			inc = -inc;
		}
		else {
			loop--;
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
	float angle, alpha, beta;

	alpha = 500*LINE_SCANNER_SCALE / d_x * m;
	beta = LINE_SCANNER_SCALE / d_x * (64 - (m*NUM_SCANS+b));
	angle = (atan(beta)); // +alpha

	return angle/PI;
}
float find_angle(int max, float d_x, float theta){
	float angle, alpha, beta, dalpha, ret;


	//alpha = LINE_SCANNER_SCALE / d_x * m;
	alpha = 64 - max;

	
	beta = LINE_SCANNER_SCALE / d_x * alpha;
	angle = -atan(beta)/PI; 

	ret = -(alpha/42);   //-alpha/42 is good

	
	return ret;
}



void initPWM(){
	// FIX ME - something weird is going on with the pwms
	servo.period_ms(20); 
	leftMotor.period_ms(2);
	rightMotor.period_ms(2);
	
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


int main() {
   // nbprint_setup();
		
	initPWM();
	
	//int isGoingFwd = goForward(0.25f);
	
	//int linescan[128][NUM_SCANS] = {};
	//int altscan[128][NUM_SCANS] = {};
	//int linemin[13], linemax[13], linepts[13];
	int linescan[128] = {};
	int altscan[128] = {};
	int linemax, linemin, linecenter;
		
	float lsqcoef[2], ladcoef[2];
	float *lsline;
	lsline = new float[NUM_SCANS];
	float *ladline;
	ladline = new float[NUM_SCANS];
	float *usecoef;
	float lserrdist, laderrdist, current_speed;
	int i;
	//for (int i =0; i<128; i++){
	//		linescan[i] = 0;
	//}
	//int linemax = 0;
	float turnto = 0;
	float dturn = 0;
	float pturn = 0;
	int linerow = 0;
		//Linescan camera(cam_si,cam_ck,cam_ao);
	while(1){
		//cam_si = 1;
		//wait(.00005); 
		//cam_si = 0;
		
//  first three lines will clear the buffer on the camera.
//	not exactly necesasry if using a ticker or some thread-like processing	
		wait(0.005); //10ms integration
		cam_si = 1;
		wait(.00005); // serial initiation pulse
		cam_si = 0;
		// read from analog sensor
		for (i = 0; i<128;i++){
			int aout = cam_ao.read_u16();
			//printf("pixel %d", aout);
			//pc.printf("hm %d \n", aout);
			linescan[i] = aout;//.5*linescan[i] + aout;
			//if (linescan[i] >= linescan[linemax]){
			//	linemax = i;
			//}
			wait(0.000012); //quarter period of the cameras clock (dunno, it works tho)
		}
		linerow++;
		linerow %= NUM_SCANS;
		/*pc.printf("\r");
		for(i = 0; i<128;i+=2){
			if(linescan[i] > .75*65000){
				pc.printf("-");
			} else {
				pc.printf("_");
			}
		}*/
		//turnto = linerow/13.0f;
		//turn(turnto);
		
		gaussian_filter(linescan,altscan);

		differential(altscan,linescan);

		
		
		/*pc.printf("\r");
		for(i = 0; i<128;i+=2){
			if(linescan[i] > 1000){
				pc.printf("-");
			} else {
				pc.printf("_");
			}
		}*/
		
		
		
		linemax = argmax(linescan);
		linemin = argmin(linescan);
		linecenter = (linemax + linemin)/2; 
		
		pc.printf("%d\n\r", linecenter);
		//for (i = 0; i< NUM_SCANS; i++){
		//	pc.printf("%f\r",lsqcoef[0]);
		//}
			//linemax = 0;
		//leastsq(linepts, lsqcoef);
		//pc.printf("%f\r",lsqcoef[0]);
		//ladcoef = leastabs(linepts,lsqcoef[0],lsqcoef[1]);
		
		current_speed = 1.0;
		//turnto += find_angle(lsqcoef[0], lsqcoef[1], current_speed);
		
		turnto = find_angle(linecenter, current_speed, turnto);
		
		
		if (turnto > 1){
			turnto = 1;
		} else if (turnto < -1) {
			turnto = -1;
		}
	  turn(turnto);

		
		
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

