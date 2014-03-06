static char oldchar = 'o';
	led1 = led2 = led3 = 1;
	brake = 1;
	dir = 1;
 // int count = 0;  
	pc.printf("Which direction? (l,r,f,b,s)\r");
	while(1) {
				//printf("hey guys!\n");
		
		
    
		
		
		
		char newchar = pc.getc();
		if(newchar != oldchar){
			pc.printf("Which direction? (l,r,f,b,s)\r");
			oldchar = newchar;
			switch(newchar){
				
				case 'f': //forward
					dir = 1;
					brake = 0;
					led1 = 0; led2 = 0; led3 = 1;
				
					//wait(3);
					break;
			case 'b':
				//reverse
				dir = 0;
				brake = 0;
        led2 = 0; led1 = 1; led3 = 0;
			
        //wait(3);ffb
				break;
			case 'l': //left
				led3 = 0; led2 = 1; led1 = 0;
				servo.pulsewidth_us(1200);
        //wait(3);
				break;
			case 'r':
				//passive break
				led3 = 0; led1 = 1; led2 = 1;
				servo.pulsewidth_us(1800);
				//wait(3);
				break;
			case 's':
				led3 = 1; led1 = 1; led2 = 0;
				servo.pulsewidth_us(1500);
				brake = 1;
				break;
			default:
				pc.printf("Wrong choice");
		}
	}
	}	