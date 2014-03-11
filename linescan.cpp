//creates helper functions for linescan camera

#include "mbed.h"

//create "default" constructors
//	appended to respective headers
//DigitalOut::DigitalOut(){}
//AnalogIn::AnalogIn(){}

class Linescan {
	private:
		int _idx;
		DigitalOut si, ck;
		AnalogIn ao;
		volatile int camera_scan[128];
		void a2dconvert (void);
	public:
		Linescan (PinName,PinName,PinName);
		int camera_avg[128]; //array that is the average over some samples
		int* getScan (void);
		void scan (void);
};

Linescan::Linescan(PinName si_pin, PinName ck_pin, PinName ao_pin){
	
	si = DigitalOut(si_pin);
	ck = DigitalOut(ck_pin);
	ao = AnalogIn(ao_pin);
	
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

