/************************************************************
 Real Time Embedded Systems[ECEN 5623]  
 Code by: Omkar Purandare , Sameer Vaze, Rishi Soni 
*************************************************************/

/******************Header files**********************************/
#include <unistd.h>
#include <stdio.h>
#include <iostream>
#include <pthread.h>
#include <semaphore.h>
#include <sched.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>
#include <termios.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <errno.h>
#include <time.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <string.h>
#include "SimpleGPIO.h"
/****************************************************************/



/**********************************Thread Globals**************************************/
using namespace cv;
using namespace std;
pthread_t CV_thread, motor_thread,ultrasonic_thread;
pthread_attr_t CV_attr, main_attr, motor_attr,ultrasonic_attr;
pthread_mutex_t vector_mutex;
pthread_mutex_t ultrasonic_mutex;

struct sched_param CV_param;
struct sched_param main_param;
struct sched_param motor_param;
struct sched_param ultrasonic_param;
struct timespec start = {0,0};
struct timespec end = {0,0};
struct timespec thread_dt = {0,0};
#define NSEC_PER_SEC (1000000000)
#define NSEC_PER_MSEC (1000000)
#define NSEC_PER_MICROSEC (1000)
long array1[1000];
double res[100];
cpu_set_t affinity;
/********************************************************************************/

/****************************Image Globals****************************************/
#define HRES 640
#define VRES 480
CvCapture* capture;
IplImage* frame;
Mat gray;
vector<Vec3f> circles;
int max_prio, min_prio;
int dev=0;
/**********************************************************************************/

/**************************Ultrasonic Globals************************************/
struct termios *configure;
char *device = "/dev/ttyACM0";
int fd;
uint8_t len, c;
/*********************************************************************************/


/***************************Motor Globals******************************************/
unsigned int GPIOPIN50 = 57; // GPIO1_28 = (1x32) + 28 = 60
unsigned int GPIOPIN58 = 166; 
unsigned int GPIOPIN55 = 165;
unsigned int GPIOPIN52 = 164;
/************************************************************************************/

/***********************Threads and function *************************************/
void *CVThread(void *);
void *MotorThread(void *);
void *UltrasonicThread(void *);
void motor_initialize(void);
void tty_config(struct termios *con, int descriptor);
void driver_initialize(void);
int delta_t(struct timespec *stop, struct timespec *start, struct timespec *delta_t);
void calc_avg(long *arr);
/*********************************************************************************/


/******************************************* Main **********************************************************/
int main( int argc, char** argv )
{
    cvNamedWindow("Capture Example", CV_WINDOW_AUTOSIZE);
	
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
	motor_initialize(); //function to initialize motor
	driver_initialize(); //function to initialize serial driver
	
	CPU_ZERO(&affinity);
	CPU_SET(0, &affinity);
	
	//Intializing the mutex
	pthread_mutex_init(&vector_mutex, NULL); 
	pthread_mutex_init(&ultrasonic_mutex, NULL);
	
	//intializing the attributes for all the threads
	pthread_attr_init(&CV_attr);
	pthread_attr_init(&motor_attr);
	pthread_attr_init(&ultrasonic_attr);
	pthread_attr_init(&main_attr);
	
	//setting the fifo scheduling for the CV thread and setting affinity to CPU0
	pthread_attr_setinheritsched(&CV_attr, PTHREAD_EXPLICIT_SCHED);
	pthread_attr_setschedpolicy(&CV_attr, SCHED_FIFO);
	pthread_attr_setaffinity_np(&CV_attr, sizeof(cpu_set_t), &affinity);

	//setting the fifo scheduling for the motor thread and setting affinity to CPU0
	pthread_attr_setinheritsched(&motor_attr, PTHREAD_EXPLICIT_SCHED);
	pthread_attr_setschedpolicy(&motor_attr, SCHED_FIFO);
	pthread_attr_setaffinity_np(&motor_attr, sizeof(cpu_set_t), &affinity);

	//setting the fifo scheduling for the ultrasonic thread and setting affinity to CPU0
	pthread_attr_setinheritsched(&ultrasonic_attr, PTHREAD_EXPLICIT_SCHED);
	pthread_attr_setschedpolicy(&ultrasonic_attr, SCHED_FIFO);
	pthread_attr_setaffinity_np(&ultrasonic_attr, sizeof(cpu_set_t), &affinity);

	//setting the fifo scheduling for the main thread and setting affinity to CPU0	
	pthread_attr_setinheritsched(&main_attr, PTHREAD_EXPLICIT_SCHED);
	pthread_attr_setschedpolicy(&main_attr, SCHED_FIFO);
	pthread_attr_setaffinity_np(&main_attr, sizeof(cpu_set_t), &affinity);	

	//Getting the priorities
	max_prio = sched_get_priority_max(SCHED_FIFO);
	min_prio = sched_get_priority_min(SCHED_FIFO);
	
	//Setting the priorities for all the threads
	main_param.sched_priority = max_prio;
	CV_param.sched_priority = max_prio - 3;
	motor_param.sched_priority = max_prio - 1;
	ultrasonic_param.sched_priority = max_prio - 2;
	
	
	int rc = sched_setscheduler(getpid(), SCHED_FIFO, &main_param);

	if(rc)
	{
		perror(NULL);
		printf("Error setting scheduler: %d\n", rc);		
		exit(1);
	}
	
	//Setting attributes
	pthread_attr_setschedparam(&CV_attr, &CV_param);
	pthread_attr_setschedparam(&motor_attr, &motor_param);
	pthread_attr_setschedparam(&ultrasonic_attr, &ultrasonic_param);
	pthread_attr_setschedparam(&main_attr, &main_param);

	//Creating thread CV
	if(pthread_create(&CV_thread, &CV_attr, CVThread, (void *)0))
	{
		printf("Error creating CV thread\n");
		perror(NULL);		
		exit(1);
	}
	
	//Creating thread motor
	if(pthread_create(&motor_thread, &motor_attr, MotorThread, (void *)0))
	{
		printf("Error creating  motor thread\n");
		perror(NULL);
		exit(1);
	}
	
	//Creating ultrasonic thread
	if(pthread_create(&ultrasonic_thread, &ultrasonic_attr, UltrasonicThread, (void *)0))
	{
		printf("Error creating  ultra thread\n");
		perror(NULL);
		exit(1);
	}
	
	//Waiting for all threads to join which will not happen because threads run in a infinite while loop
	pthread_join(ultrasonic_thread, NULL);
	pthread_join(CV_thread, NULL);
	pthread_join(motor_thread, NULL);
}
/***********************************************************************************************/


/***********************************CV Thread***********************************************/
void *CVThread(void *)
{
    //capture with defined resolution
    capture = (CvCapture *)cvCreateCameraCapture(0);
    cvSetCaptureProperty(capture, CV_CAP_PROP_FRAME_WIDTH, HRES);
    cvSetCaptureProperty(capture, CV_CAP_PROP_FRAME_HEIGHT, VRES);
    
    while(1)
    {    	
	frame=cvQueryFrame(capture); //Extracting frame from the capture
        Mat mat_frame(frame);
        cvtColor(mat_frame, gray, CV_BGR2GRAY);//Converting frame to gray scale
        GaussianBlur(gray, gray, Size(9,9), 2, 2); //Remove bluring
		
        HoughCircles(gray, circles, CV_HOUGH_GRADIENT, 1, gray.rows/8, 100, 50, 5, 40);//Detecting circles on screen 
	pthread_mutex_unlock(&vector_mutex);	

        for( size_t i = 0; i < circles.size(); i++ ) //storing the centre and radius of all the circles displayed on screen globally
        {
		pthread_mutex_lock(&vector_mutex);//locking mutex
		Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
		int radius = cvRound(circles[i][2]);

		// circle center
		circle( mat_frame, center, 3, Scalar(0,255,0), -1, 8, 0 );
		// circle outline
		circle( mat_frame, center, radius, Scalar(0,0,255), 3, 8, 0 );
        }
	pthread_mutex_unlock(&vector_mutex);//Mutex unlock
     
        if(!frame) break;

        cvShowImage("Capture Example", frame);//Show the image
		
        char c = cvWaitKey(10);
        if( c == 'q' )	
		break;//break if key is pressed
   }
    cvReleaseCapture(&capture);
    cvDestroyWindow("Capture Example");	//Destroy windows
}
/****************************************************************************************/



/***********************************Motor Thread***********************************************/
void *MotorThread(void *)
{	
	while(1)
	{	//lock camera and ultrasonic mutex
		pthread_mutex_lock(&vector_mutex);
		pthread_mutex_lock(&ultrasonic_mutex);
		if(circles.size() > 0)		
		{	
			if (circles[0][0] >= 480)
			{	
				//Left turn if circle detected in towards the left of the screen
				gpio_set_value(GPIOPIN50, HIGH);
				gpio_set_value(GPIOPIN58, LOW);
				gpio_set_value(GPIOPIN55, LOW);
				gpio_set_value(GPIOPIN52, LOW);

			}
			else if (circles[0][0] <= 160)
			{	
				//Right turn if circle detected is towards the right of the screen
				gpio_set_value(GPIOPIN50, LOW);
				gpio_set_value(GPIOPIN58, LOW);
				gpio_set_value(GPIOPIN55, HIGH);
				gpio_set_value(GPIOPIN52, LOW);
			}
			else
			{
				if(c>80)
				{	//straight motion if circle is in the middle and ultrasonic centre senses more distance
					gpio_set_value(GPIOPIN50, HIGH);
					gpio_set_value(GPIOPIN58, LOW);
					gpio_set_value(GPIOPIN55, HIGH);
					gpio_set_value(GPIOPIN52, LOW);
				}
				else
				{	//stoping condition
					gpio_set_value(GPIOPIN50, LOW);
					gpio_set_value(GPIOPIN58, LOW);
					gpio_set_value(GPIOPIN55, LOW);
					gpio_set_value(GPIOPIN52, LOW);
				}
			}

		}

		else
		{	//stoping condition
			gpio_set_value(GPIOPIN50, LOW);
			gpio_set_value(GPIOPIN58, LOW);
			gpio_set_value(GPIOPIN55, LOW);
			gpio_set_value(GPIOPIN52, LOW);
		}
		//unlock camera and ultrasonic mutex
		pthread_mutex_unlock(&vector_mutex);
		pthread_mutex_unlock(&ultrasonic_mutex);
		usleep(19000);//Sleeping the thread for 40ms
	}
        
}

/***********************Ultrasonic Thread****************************************/

void *UltrasonicThread(void *)
{
	while(1)
	{
		pthread_mutex_lock(&ultrasonic_mutex); //locking ultrasonic mutex
		while(read(fd, &c, 1) <= 0){} //waiting till a valid ultrasonic value is read
		printf("%d\n", c); //print the ultrasonic values
		fflush(stdout); 
		pthread_mutex_unlock(&ultrasonic_mutex); //unlocking ultrasonic mutex
		usleep(25000);//Sleeping thread for 25ms
	}
}
/*********************************************************************************/

/********************Intializing driver for reading ultrasonic via USB********************/
void driver_initialize(void)
{
	fd = open(device, O_RDWR | O_NOCTTY | O_NONBLOCK);				//tty file descriptor 
	if(fd == -1)
	{
		perror("ERROR opening file descriptor\n");
	}

	configure = (struct termios*)malloc(sizeof(struct termios));			//allocate memory for serial I/O
	tty_config(configure, fd);
}


void tty_config(struct termios *con, int descriptor)
{
	tcgetattr(descriptor, con); //get attrubutes of the file descriptor (tty port)
	con->c_iflag &= ~(IGNBRK | BRKINT | ICRNL | INLCR | PARMRK | INPCK | ISTRIP | IXON);	//clear all input flags
	con->c_oflag = 0;									//clear all output flags
	con->c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);				//clear all local flags
	con->c_cc[VMIN] = 1;									//min number of bytes for read operation
	con->c_cc[VTIME] = 0;									//timeout in deciseconds for read operation
	if(cfsetispeed(con, B9600) || cfsetospeed(con, B9600))					//set I/O baud rate as 9600
	{
		perror("ERROR in baud set\n");
	}
	if(tcsetattr(descriptor, TCSAFLUSH, con) < 0)						//if termios hasn't been configured correctly according to the attribute, error
	{
		perror("ERROR in set attr\n");
	}
}
/*********************************************************************************************/

/***********************Motor Intialization function**************************************/
void motor_initialize(void)
{
	cout << "Initialize the GPIO Pins" << endl;
	//Exporting GPIO pins
	gpio_export(GPIOPIN50); 				
	gpio_export(GPIOPIN58); 				
	gpio_export(GPIOPIN55);
	gpio_export(GPIOPIN52);
	//Setting GPIOs as output
	gpio_set_dir(GPIOPIN50, OUTPUT_PIN); 			
	gpio_set_dir(GPIOPIN58, OUTPUT_PIN); 			
	gpio_set_dir(GPIOPIN55, OUTPUT_PIN);
	gpio_set_dir(GPIOPIN52, OUTPUT_PIN);
}
/***************************************************************************************/
