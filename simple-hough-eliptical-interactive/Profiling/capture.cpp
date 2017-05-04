/*
 *
 *  Example by Sam Siewert 
 *
 */
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
#include <unistd.h>
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


struct timespec start = {0,0};
struct timespec end = {0,0};
struct timespec thread_dt = {0,0};
#define NSEC_PER_SEC (1000000000)
#define NSEC_PER_MSEC (1000000)
#define NSEC_PER_MICROSEC (1000)
long array1[1000];
double res[100];
cpu_set_t affinity;

/**********************************Thread Globals**************************************/
using namespace cv;
using namespace std;
pthread_t CV_thread, motor_thread,ultrasonic_thread;
pthread_attr_t CV_attr, main_attr, motor_attr,ultrasonic_attr;

pthread_mutex_t vector_mutex;
pthread_mutex_t ultrasonic_mutex;
sem_t sem_image, sem_ultra;
struct sched_param CV_param;
struct sched_param main_param;
struct sched_param motor_param;
struct sched_param ultrasonic_param;
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

int delta_t(struct timespec *stop, struct timespec *start, struct timespec *delta_t)
{
  int dt_sec=stop->tv_sec - start->tv_sec;
  int dt_nsec=stop->tv_nsec - start->tv_nsec;

  if(dt_sec >= 0)
  {
    if(dt_nsec >= 0)
    {
      delta_t->tv_sec=dt_sec;
      delta_t->tv_nsec=dt_nsec;
    }
    else
    {
      delta_t->tv_sec=dt_sec-1;
      delta_t->tv_nsec=NSEC_PER_SEC+dt_nsec;
    }
  }
  else
  {
    if(dt_nsec >= 0)
    {
      delta_t->tv_sec=dt_sec;
      delta_t->tv_nsec=dt_nsec;
    }
    else
    {
      delta_t->tv_sec=dt_sec-1;
      delta_t->tv_nsec=NSEC_PER_SEC+dt_nsec;
    }
  }

  return(1);
}

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
	driver_initialize();
	
	CPU_ZERO(&affinity);
	CPU_SET(0, &affinity);
	
	pthread_mutex_init(&vector_mutex, NULL);
	pthread_mutex_init(&ultrasonic_mutex, NULL);

	pthread_attr_init(&CV_attr);
	pthread_attr_init(&motor_attr);
	pthread_attr_init(&ultrasonic_attr);
	pthread_attr_init(&main_attr);
	
	pthread_attr_setinheritsched(&CV_attr, PTHREAD_EXPLICIT_SCHED);
	pthread_attr_setschedpolicy(&CV_attr, SCHED_FIFO);
	pthread_attr_setaffinity_np(&CV_attr, sizeof(cpu_set_t), &affinity);

	pthread_attr_setinheritsched(&motor_attr, PTHREAD_EXPLICIT_SCHED);
	pthread_attr_setschedpolicy(&motor_attr, SCHED_FIFO);
	pthread_attr_setaffinity_np(&motor_attr, sizeof(cpu_set_t), &affinity);

	pthread_attr_setinheritsched(&ultrasonic_attr, PTHREAD_EXPLICIT_SCHED);
	pthread_attr_setschedpolicy(&ultrasonic_attr, SCHED_FIFO);
	pthread_attr_setaffinity_np(&ultrasonic_attr, sizeof(cpu_set_t), &affinity);
	
	pthread_attr_setinheritsched(&main_attr, PTHREAD_EXPLICIT_SCHED);
	pthread_attr_setschedpolicy(&main_attr, SCHED_FIFO);
	pthread_attr_setaffinity_np(&main_attr, sizeof(cpu_set_t), &affinity);	

	max_prio = sched_get_priority_max(SCHED_FIFO);
	min_prio = sched_get_priority_min(SCHED_FIFO);
	
	main_param.sched_priority = max_prio;
	CV_param.sched_priority = max_prio - 3;
	motor_param.sched_priority = max_prio - 1;
	ultrasonic_param.sched_priority = max_prio - 2;
	
	sem_init(&sem_image, 0, 1);
	sem_init(&sem_ultra, 0, 0);

	int rc = sched_setscheduler(getpid(), SCHED_FIFO, &main_param);

	if(rc)
	{
		perror(NULL);
		printf("Error setting scheduler: %d\n", rc);		
		exit(1);
	}

	pthread_attr_setschedparam(&CV_attr, &CV_param);
	pthread_attr_setschedparam(&motor_attr, &motor_param);
	pthread_attr_setschedparam(&ultrasonic_attr, &ultrasonic_param);
	pthread_attr_setschedparam(&main_attr, &main_param);

	
	if(pthread_create(&CV_thread, &CV_attr, CVThread, (void *)0))
	{
		printf("Error creating CV thread\n");
		perror(NULL);		
		exit(1);
	}
	
	if(pthread_create(&motor_thread, &motor_attr, MotorThread, (void *)0))
	{
		printf("Error creating  motor thread\n");
		perror(NULL);
		exit(1);
	}
	
	if(pthread_create(&ultrasonic_thread, &ultrasonic_attr, UltrasonicThread, (void *)0))
	{
		printf("Error creating  ultra thread\n");
		perror(NULL);
		exit(1);
	}
	
	pthread_join(ultrasonic_thread, NULL);
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
	//sem_wait(&sem_image);
	//printf("Thread image\n");
    	frame=cvQueryFrame(capture);
        Mat mat_frame(frame);
        cvtColor(mat_frame, gray, CV_BGR2GRAY);
        GaussianBlur(gray, gray, Size(9,9), 2, 2);
		
	pthread_mutex_lock(&vector_mutex);
        HoughCircles(gray, circles, CV_HOUGH_GRADIENT, 1, gray.rows/8, 100, 50, 5, 40);
	pthread_mutex_unlock(&vector_mutex);	

        for( size_t i = 0; i < circles.size(); i++ )
        {
		pthread_mutex_lock(&vector_mutex);
		Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
		int radius = cvRound(circles[i][2]);

		//printf("X axis is = %f\n", circles[i][0]);
		//printf("Y axis is = %f\n", circles[i][1]);
		//printf("Radius is = %f\n", circles[i][2]);

		//if(circles[i][0] >= (float)280 && circles[i][0] <= (float)360)
		//{
		//printf("OKAY\n");
		//}
		//else
		//{
		//printf("NOT OKAY\n");
		//}

		//pthread_mutex_unlock(&vector_mutex);

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
	clock_gettime(CLOCK_REALTIME, &end);
	delta_t(&end, &start, &thread_dt);
	printf("Thread IMAGE completed at time stamp: %lf msec\n",(double)(thread_dt.tv_nsec / 1000000));
	//usleep(30000);
	//sem_post(&sem_ultra);
   }
    cvReleaseCapture(&capture);
    cvDestroyWindow("Capture Example");	
}




/* GPIOPIN 50 & 58 = Left wheel
   GPIOPIN 55 and 52 = Right wheel 
*/

void *MotorThread(void *)
{	
	while(1)
	{
		int count=0;
		if(count==1)
		
		//printf("motor thread\n");
		pthread_mutex_lock(&vector_mutex);
		pthread_mutex_lock(&ultrasonic_mutex);
		//for(size_t i = 0; i < circles.size(); i++)
		if(circles.size() > 0)		
		{	
			if (circles[0][0] >= 480)
			{	
				//Left turn 
				gpio_set_value(GPIOPIN50, HIGH);
				gpio_set_value(GPIOPIN58, LOW);
				gpio_set_value(GPIOPIN55, LOW);
				gpio_set_value(GPIOPIN52, LOW);

			}
			else if (circles[0][0] <= 160)
			{	
				//Right turn 
				gpio_set_value(GPIOPIN50, LOW);
				gpio_set_value(GPIOPIN58, LOW);
				gpio_set_value(GPIOPIN55, LOW);
				gpio_set_value(GPIOPIN52, HIGH);
			}
			else
			{
				if(c>80)
				{
					gpio_set_value(GPIOPIN50, HIGH);
					gpio_set_value(GPIOPIN58, LOW);
					gpio_set_value(GPIOPIN55, LOW);
					gpio_set_value(GPIOPIN52, HIGH);
				}
				else
				{
					gpio_set_value(GPIOPIN50, LOW);
					gpio_set_value(GPIOPIN58, LOW);
					gpio_set_value(GPIOPIN55, LOW);
					gpio_set_value(GPIOPIN52, LOW);
				}
			}

		}

		else
		{
			gpio_set_value(GPIOPIN50, LOW);
			gpio_set_value(GPIOPIN58, LOW);
			gpio_set_value(GPIOPIN55, LOW);
			gpio_set_value(GPIOPIN52, LOW);
		}
		/*else
		{
			if(c>110)
			{
				gpio_set_value(GPIOPIN50, HIGH);
				gpio_set_value(GPIOPIN58, LOW);
				gpio_set_value(GPIOPIN55, LOW);
				gpio_set_value(GPIOPIN52, HIGH);
			}
			else
			{
				gpio_set_value(GPIOPIN50, LOW);
				gpio_set_value(GPIOPIN58, LOW);
				gpio_set_value(GPIOPIN55, LOW);
				gpio_set_value(GPIOPIN52, LOW);
			}
		}*/
		pthread_mutex_unlock(&vector_mutex);
		pthread_mutex_unlock(&ultrasonic_mutex);
		clock_gettime(CLOCK_REALTIME, &end);
		delta_t(&end, &start, &thread_dt);
		printf("Thread MOTOR completed at time stamp: %lf msec\n",(double)(thread_dt.tv_nsec / 1000000));
		usleep(20000);
	}
        
}


void *UltrasonicThread(void *)
{
	while(1)
	{
		//sem_wait(&sem_ultra);
		//printf("Thread ultra\n");
		pthread_mutex_lock(&ultrasonic_mutex);
		while(read(fd, &c, 1) <= 0){}
		//printf("%d\n", c);
		fflush(stdout);
		pthread_mutex_unlock(&ultrasonic_mutex);
		//sem_post(&sem_image);
		clock_gettime(CLOCK_REALTIME, &end);
		delta_t(&end, &start, &thread_dt);
		printf("Thread ULTRASONIC completed at time stamp: %lf msec\n",(double)(thread_dt.tv_nsec / 1000000));
		usleep(50000);
	}
}


void driver_initialize(void)
{
	fd = open(device, O_RDWR | O_NOCTTY | O_NONBLOCK);
	if(fd == -1)
	{
		perror("ERROR opening file descriptor\n");
	}

	configure = (struct termios*)malloc(sizeof(struct termios));
	tty_config(configure, fd);
}


void tty_config(struct termios *con, int descriptor)
{
	tcgetattr(descriptor, con);
	con->c_iflag &= ~(IGNBRK | BRKINT | ICRNL | INLCR | PARMRK | INPCK | ISTRIP | IXON);
	con->c_oflag = 0;
	con->c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);
	con->c_cc[VMIN] = 1;
	con->c_cc[VTIME] = 0;
	if(cfsetispeed(con, B9600) || cfsetospeed(con, B9600))
	{
		perror("ERROR in baud set\n");
	}
	if(tcsetattr(descriptor, TCSAFLUSH, con) < 0)
	{
		perror("ERROR in set attr\n");
	}
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
