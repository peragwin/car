//creates helper functions for linescan camera

#include "mbed.h"
#include "linescan.h"
//create "default" constructors
//	appended to respective headers
//DigitalOut::DigitalOut(){}
//AnalogIn::AnalogIn(){}



Linescan::Linescan(DigitalOut si_pin,PwmOut ck_pin, AnalogIn ao_pin){
	
	//si = DigitalOut(si_pin);
	//ck = PwmOut(ck_pin);
	//ao = AnalogIn(ao_pin);
	ck.period(0.001);
	ck.write(0.5);
}


//tells the camera to take a pcituer
void Linescan::scan(){
	//command it to take a photo
	
}


//gets the current line from the scanner
int* Linescan::getScan() {
	
	//init timer
	Ticker timer;
	_idx = 0;
	timer.attach_us(this,&Linescan::a2dconvert,10);
	
	//take 128 adc conversions
		//set to the camera_scan array
	for (int j = 0; j < 128; j++){
		camera_avg[j] = camera_avg[j]/2 + camera_scan[j]/2;
	}
	
	//kill timer
	timer.detach();
	return camera_avg;
}

void Linescan::a2dconvert() {
	int analog = ao.read_u16();
	camera_scan[_idx] = analog;
	_idx++;
}

