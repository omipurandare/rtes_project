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

int main(int argc, char const *argv[]) {
  long duration, distance;
  struct timespec pulse_start, pulse_end, pulse_duration;
  gpio_export(GPIOPIN40);
  gpio_export(GPIOPIN43);

  gpio_set_dir(GPIOPIN40, INPUT_PIN);
  gpio_set_dir(GPIOPIN43, OUTPUT_PIN);

  while (1) {
    unsigned int value = HIGH;

    gpio_set_value(GPIOPIN40, LOW);
    delay_time(2000); //2us delay = 2000ns
    gpio_set_value(GPIOPIN40, HIGH);
    delay_time(10000); //10us delay = 10000ns
    gpio_set_value(GPIOPIN40, LOW);

    clock_gettime(CLOCK_MONOTONIC, &pulse_start);
    while (value != LOW) {
      gpio_get_value(GPIOPIN43, &value);
    }
    clock_gettime(CLOCK_MONOTONIC, &pulse_end);

    pulse_duration = pulse_end - pulse_start;

    pulse_width = pulse_duration.tv_nsec * 1000; //in microseconds
    distance = pulse_width / 58 ; //distance in centimeters
    printf("%d cm\n",distance);
  }
  return 0;
}

void delay_time(int nsec){

  struct timespec time_start, time_end;
  clock_gettime(CLOCK_MONOTONIC, &time_start);

  time_end = time_start;
  time_end.tv_nsec += nsec;

  //while (time_start.tv_nsec < time_end.tv_nsec);
}
