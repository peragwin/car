#include "mbed.h"
#include "nbprint.h"

PwmOut leftMotor(PTA12);
PwmOut rightMotor(PTA4);
DigitalOut brake(PTD4);
PwmOut servo(PTA5);

#define TURN_LEFT 300
#define TURN_RIGHT 400
#define TURN_FORWARD 1500
#define TURN_MAXTURN 300


void initPWM(){
	leftMotor.period_ms(5);
	rightMotor.period_ms(5);
	servo.period_ms(20);
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
		
		while(1){
			turn(1.0f);
			int isGoingFwd = goForward(0.30f);
			wait(4);
			turn(-1.0f);
			//passiveBrake();
  		wait(4);
		}
}


