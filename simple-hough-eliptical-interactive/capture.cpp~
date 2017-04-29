/*
 *
 *  Example by Sam Siewert 
 *
 */
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <pthread.h>
#include <semaphore.h>
#include <sched.h>
#include <errno.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <string.h>
#include <unistd.h>
#include "SimpleGPIO.h"

using namespace cv;
using namespace std;

#define HRES 640
#define VRES 480

pthread_t CV_thread, motor_thread;
pthread_attr_t CV_attr, main_attr, motor_attr;

CvCapture* capture;
IplImage* frame;
Mat gray;
vector<Vec3f> circles;
int max_prio, min_prio;
int dev=0;

pthread_mutex_t vector_mutex;

struct sched_param CV_param;
struct sched_param main_param;
struct sched_param motor_param;


unsigned int GPIOPIN50 = 57; // GPIO1_28 = (1x32) + 28 = 60
unsigned int GPIOPIN58 = 166; 
unsigned int GPIOPIN55 = 165;
unsigned int GPIOPIN52 = 164;

void *CVThread(void *);
void *MotorThread(void *);
void motor_initialize(void);

int main( int argc, char** argv )
{
    cvNamedWindow("Capture Example", CV_WINDOW_AUTOSIZE);
    //CvCapture* capture = (CvCapture *)cvCreateCameraCapture(0);
    //CvCapture* capture = (CvCapture *)cvCreateCameraCapture(argv[1]);
	
    if(argc > 1)
    {
        sscanf(argv[1], "%d", &dev);
        printf("using %s\n", argv[1]);
    }
    else if(argc == 1)
        printf("using default\n");

    else
    {
        printf("usage: capture [dev]\n");
        exit(-1);
    }
	motor_initialize();
	

	pthread_mutex_init(&vector_mutex, NULL);

	pthread_attr_init(&CV_attr);
	pthread_attr_init(&motor_attr);
	pthread_attr_init(&main_attr);
	
	pthread_attr_setinheritsched(&CV_attr, PTHREAD_EXPLICIT_SCHED);
	pthread_attr_setschedpolicy(&CV_attr, SCHED_FIFO);
	
	pthread_attr_setinheritsched(&motor_attr, PTHREAD_EXPLICIT_SCHED);
	pthread_attr_setschedpolicy(&motor_attr, SCHED_FIFO);
	
	pthread_attr_setinheritsched(&main_attr, PTHREAD_EXPLICIT_SCHED);
	pthread_attr_setschedpolicy(&main_attr, SCHED_FIFO);
	
	max_prio = sched_get_priority_max(SCHED_FIFO);
	min_prio = sched_get_priority_min(SCHED_FIFO);
	
	main_param.sched_priority = max_prio;
	CV_param.sched_priority = max_prio - 1;
	motor_param.sched_priority = max_prio - 2;
	
	int rc = sched_setscheduler(getpid(), SCHED_FIFO, &main_param);

	if(rc)
	{
		perror(NULL);
		printf("Error setting scheduler: %d\n", rc);		
		exit(1);
	}

	pthread_attr_setschedparam(&CV_attr, &CV_param);
	pthread_attr_setschedparam(&motor_attr, &motor_param);
	pthread_attr_setschedparam(&main_attr, &main_param);


	if(pthread_create(&CV_thread, &CV_attr, CVThread, (void *)0))
	{
		printf("Error creating thread\n");
		exit(1);
	}
	
	if(pthread_create(&motor_thread, &motor_attr, MotorThread, (void *)0))
	{
		printf("Error creating  motor thread\n");
		exit(1);
	}

	pthread_join(CV_thread, NULL);
	pthread_join(motor_thread, NULL);
}

void *CVThread(void *)
{
    capture = (CvCapture *)cvCreateCameraCapture(0);
    cvSetCaptureProperty(capture, CV_CAP_PROP_FRAME_WIDTH, HRES);
    cvSetCaptureProperty(capture, CV_CAP_PROP_FRAME_HEIGHT, VRES);

    while(1)
    {
    	frame=cvQueryFrame(capture);

        Mat mat_frame(frame);
        cvtColor(mat_frame, gray, CV_BGR2GRAY);
        GaussianBlur(gray, gray, Size(9,9), 2, 2);
		
	pthread_mutex_lock(&vector_mutex);
        HoughCircles(gray, circles, CV_HOUGH_GRADIENT, 1, gray.rows/8, 100, 50, 30, 40);
//	pthread_mutex_unlock(&vector_mutex);	
//	printf("before while loop\n");
        for( size_t i = 0; i < circles.size(); i++ )
        {
//			pthread_mutex_lock(&vector_mutex);
			Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
			int radius = cvRound(circles[i][2]);

			printf("X axis is = %f\n", circles[i][0]);
			printf("Y axis is = %f\n", circles[i][1]);
			printf("Radius is = %f\n", circles[i][2]);

			if(circles[i][0] >= (float)300 && circles[i][0] <= (float)380)
			{
			printf("OKAY\n");
			}
			else
			{
			printf("NOT OKAY\n");
			}
//			pthread_mutex_unlock(&vector_mutex);
			// circle center
			circle( mat_frame, center, 3, Scalar(0,255,0), -1, 8, 0 );
			// circle outline
			circle( mat_frame, center, radius, Scalar(0,0,255), 3, 8, 0 );
        }
	pthread_mutex_unlock(&vector_mutex);
     
        if(!frame) break;

        cvShowImage("Capture Example", frame);
		
        char c = cvWaitKey(10);
        if( c == 'q' )	
	break;
	usleep(100);
   }
    cvReleaseCapture(&capture);
    cvDestroyWindow("Capture Example");	
}

void *MotorThread(void *)
{	
	while(1)
	{
		for( size_t i = 0; i < circles.size(); i++)
		{	
			if (circles[i][0] >= float(340) && circles[i][0] <= float(380))
			{
				gpio_set_value(GPIOPIN50, HIGH);
				gpio_set_value(GPIOPIN58, LOW);
			}
			else
			{
				gpio_set_value(GPIOPIN50, LOW);
				gpio_set_value(GPIOPIN58, LOW);
			}			
		}
	}
        //usleep(100);
}
void motor_initialize(void)
{
	cout << "Initialize the GPIO Pins" << endl;
	gpio_export(GPIOPIN50); 				// The LED
	gpio_export(GPIOPIN58); 				// The push button switch
	gpio_export(GPIOPIN55);
	gpio_export(GPIOPIN52);
	gpio_set_dir(GPIOPIN50, OUTPUT_PIN); 			// The LED is an output
	gpio_set_dir(GPIOPIN58, OUTPUT_PIN); 			// The LED is an output
	gpio_set_dir(GPIOPIN55, OUTPUT_PIN);
	gpio_set_dir(GPIOPIN52, OUTPUT_PIN);
}

