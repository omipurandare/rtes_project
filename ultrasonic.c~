#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <errno.h>
#include "SimpleGPIO.h"
#include <time.h>

unsigned int GPIOPIN40 = 160;     //TRIG pin OUTPUT
unsigned int GPIOPIN43 = 161;     //ECHO pin INPUT

void delay_time(int nsec);

int main(void) {
  long duration, distance;
  struct timespec pulse_start, pulse_end;
  long  pulse_duration, pulse_width;
  gpio_export(GPIOPIN40);
  gpio_export(GPIOPIN43);
  gpio_set_dir(GPIOPIN40, OUTPUT_PIN);
  gpio_set_dir(GPIOPIN43, INPUT_PIN);
  printf("in main\n");
  //gpio_set_value(GPIOPIN40, HIGH);
  while (1) {
    unsigned int value;
    //while(1)
  //  {
        value = HIGH;
        gpio_set_value(GPIOPIN40, LOW);
	//usleep(10);
        delay_time(2000); //2us delay = 2000ns
        gpio_set_value(GPIOPIN40, HIGH);
	//usleep(10);
       	delay_time(10000); //10us delay = 10000ns
       gpio_set_value(GPIOPIN40, LOW);
   // }
//    printf("main while loop\n");

    clock_gettime(CLOCK_MONOTONIC, &pulse_start);
    while (value != LOW) {
     // printf("in loop\n");
      gpio_get_value(GPIOPIN43, &value);
    }
    clock_gettime(CLOCK_MONOTONIC, &pulse_end);

    pulse_duration = pulse_end.tv_nsec - pulse_start.tv_nsec;

    pulse_width = pulse_duration / 1000; //in microseconds
    //distance = pulse_width / 58 ; //distance in centimeters
    printf("%ld cm\n",pulse_width);
    usleep(500000);
  }
  return 0;
}

void delay_time(int nsec)
{
  struct timespec time_start, time_end,time_comp;
  clock_gettime(CLOCK_MONOTONIC, &time_start);
  time_end.tv_nsec = time_start.tv_nsec + nsec;
  while(1)
  {
	clock_gettime(CLOCK_MONOTONIC, &time_comp);
	if(time_comp.tv_nsec >= time_end.tv_nsec)
        	break;
	//printf("0\n");
//	else
//		printf("current time%ld\n", time_comp.tv_nsec);
  }

//  printf("start time:%ld\n",time_start.tv_nsec);
//  printf("start time:%ld\n",time_end.tv_nsec);

}
