
class Linescan {
	private:
		int _idx;
		DigitalOut si;
		AnalogIn ao;
		PwmOut ck;
		volatile int camera_scan[128];
		void a2dconvert (void);
	public:
		Linescan (DigitalOut,PwmOut,AnalogIn);
		int camera_avg[128]; //array that is the average over some samples
		int* getScan (void);
		void scan (void);
};

