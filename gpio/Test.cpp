#include <iostream>
#include <string>
#include <unistd.h>
#include "gpio.h"

using namespace std;

unsigned int GPIOPIN50 = 57; // GPIO1_28 = (1x32) + 28 = 60
unsigned int GPIOPIN58 = 166; 
unsigned int GPIOPIN55 = 165;
unsigned int GPIOPIN52 = 164;

int main(int argc, char *argv[])
{
	cout << "Testing the GPIO Pins" << endl;
	gpio_export(GPIOPIN50); 				// The LED
	gpio_export(GPIOPIN58); 				// The push button switch
	gpio_export(GPIOPIN55);
	gpio_export(GPIOPIN52);
	gpio_set_dir(GPIOPIN50, OUTPUT_PIN); 			// The LED is an output
	gpio_set_dir(GPIOPIN58, OUTPUT_PIN); 			// The LED is an output
	gpio_set_dir(GPIOPIN55, OUTPUT_PIN);
	gpio_set_dir(GPIOPIN52, OUTPUT_PIN);
	
	// Flash the LED
	while(1){
		cout << "All HIGH\n" << endl;
		gpio_set_value(GPIOPIN50, HIGH);
		gpio_set_value(GPIOPIN58, HIGH);
		gpio_set_value(GPIOPIN55, HIGH);
		gpio_set_value(GPIOPIN52, HIGH);
		usleep(2000000);
		cout << "50 LOW 58 HIGH\n" << endl;
		gpio_set_value(GPIOPIN50, LOW);
		gpio_set_value(GPIOPIN58, HIGH);
		gpio_set_value(GPIOPIN55, LOW);
		gpio_set_value(GPIOPIN52, HIGH);
		usleep(2000000); 
		cout << "50 HIGH 58 LOW\n" << endl;
		gpio_set_value(GPIOPIN50, HIGH);
		gpio_set_value(GPIOPIN58, LOW);
		gpio_set_value(GPIOPIN55, HIGH);
		gpio_set_value(GPIOPIN52, LOW);
		usleep(2000000);
		cout << "REPEAT\n" << endl;	
		}
}
