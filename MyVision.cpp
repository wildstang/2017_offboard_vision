#include <cv.h>
#include <cxcore.h>
#include <highgui.h>
#include <opencv2/opencv.hpp>
#include <stdio.h>
#include <iostream>
#include <math.h>


#include <time.h>
#include <stdlib.h>

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>

#include <sys/time.h>
using namespace cv;
using namespace std;

#define WS_USE_SOCKETS

static bool NODASHBOARD = false;
static int NUM_AVERAGES = 5;
static int BRIGHTNESS = 30;
static int CONTRAST = 5;
static int SATURATION = 200;

static double TimeDiffInSec(timespec *start, timespec *stop);
static void timespecDisplay(timespec *time);

void error(const char *msg)
{
    perror(msg);
    exit(0);
}

int m_H_MIN = 0;
int m_S_MIN = 0;
int m_V_MIN = 245;

int m_H_MAX = 0;
int m_S_MAX = 0;
int m_V_MAX = 255;

int offset = 0;
int thresholdX = 50;

int sockfd;

static int countP = 0;
static int cc = 0;

// exports for the filter
extern "C" {
    bool filter_init(const char * args, void** filter_ctx);
    void filter_process(void* filter_ctx, Mat &src, Mat &dst);
    void filter_free(void* filter_ctx);
}


/**
    Initializes the filter. If you return something, it will be passed to the
    filter_process function, and should be freed by the filter_free function
*/
bool filter_init(const char * args, void** filter_ctx) {
	//Set client
	//Set IP address

#ifdef WS_USE_SOCKETS
    int portno, n;

    struct sockaddr_in serv_addr;
    struct hostent *server;

    char buffer[256];
	
    portno = 8080;
    cout << "Opening socket" << endl;
    sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0)
    {
    	cout << "Could not open socket" << endl;
        error("ERROR opening socket");
    }

    cout << "Getting host name" << endl;
    server = gethostbyname("10.1.11.53");
    if (server == NULL) {
        cout << "ERROR, no such host" << endl;
        exit(0);
    }

    bzero((char *) &serv_addr, sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    bcopy((char *)server->h_addr, (char *)&serv_addr.sin_addr.s_addr, server->h_length);
    serv_addr.sin_port = htons(portno);

	//connection retry
    int	Count = 0;
    cout << "Attempting to connect: " << endl;
	cout << "." << std::flush;
	Count++;
	while (connect(sockfd,(struct sockaddr *)&serv_addr,sizeof(serv_addr)) < 0) {
		if ((Count % 1000) == 0) {
			cout << "." << std::flush;
		}
		Count++;
	}
    cout << endl;
    cout << "Connected" << endl;

    bzero(buffer,256);
    n = recv(sockfd,buffer,255, 0);
    cout << "Received data" << endl;
    if (n < 0)
    {
    	error("ERROR reading from socket");
    }

    printf("%s\n",buffer);

    // Parse config string
    char *token;

	/* get the first token */
	token = strtok(buffer, "|\n");

	int count = 0;
	/* walk through other tokens */
	while( token != NULL )
	{
		printf( " %s\n", token );
		if (count == 0)
		{
			m_H_MIN = atoi(token);
		}
		if (count == 1)
		{
			m_S_MIN = atoi(token);
		}
		if (count == 2)
		{
			m_V_MIN = atoi(token);
		}
		if (count == 3)
		{
			m_H_MAX = atoi(token);
		}
		if (count == 4)
		{
			m_S_MAX = atoi(token);
		}
		if (count == 5)
		{
			m_V_MAX = atoi(token);
		}
		if (count == 6)
		{
			offset = atoi(token);
		}
		if (count == 7)
		{
			thresholdX = atoi(token);
		}

		token = strtok(NULL, "|\n");
		count++;
	}

	printf("Hmin = %d\nHmax = %d\nSmin = %d\nSmax = %d\nVmin = %d\nVmax = %d\n", m_H_MIN, m_H_MAX, m_S_MIN, m_S_MAX, m_V_MIN, m_V_MAX);
#endif
	return true;
}

bool wayToSort(int i, int j) { return i > j; }

void ws_process(Mat& img) {
	
	Mat hsvMat(img.size(), img.type());
	Mat hsvOut;
	
	Rect oneRect;
	Rect theOtherRect;
	
	int leftBound;
	int rightBound;
	int unitSize = 1;
	int xCorrectionLevel;
	int avgX;
	int closest = 0;
	int secClose = 0;
	int leftSide;
	int rightSide;
	
	timespec tsStart;
	timespec tscvtColor;
	timespec tsInRange;
	timespec tsBlur;
	timespec tsEnd;
	clock_gettime(CLOCK_REALTIME, &tsStart);
	cvtColor(img, hsvMat, COLOR_BGR2HSV);
	
	inRange(hsvMat, Scalar(0, 0, 245), Scalar(0, 0, 255), hsvOut);
	Mat blurMat;
	double doubleRadius = 7;
	int radius = (int)(doubleRadius + 0.5);
	int kernelSize = 2 * radius + 1;
	blur(hsvOut, blurMat, Size(kernelSize, kernelSize));
	
	vector < vector<Point> > contours;
	vector < Vec4i > hierarchy;
	findContours(blurMat, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);
	vector <double> similarity(contours.size());
	
	
	if (contours.size() < 2) {
		//if not enough contours bypasses processing
		goto Exit;
	}
	
	for (int i = 0; i < contours.size(); i++) {
		Rect currentSize = boundingRect(contours[i]);

		int	Height , Width;
		//double	Height , Width;

		Height = currentSize.height;
		Width  = currentSize.width;
		//checks size of contours and reduces sample size
		if (((Height < 100) || (Width < 75)) || (Height > 500) || (Width > 700)) {
			similarity[i] = 10.;
		} else {
			similarity[i] = fabs(1-((currentSize.height / currentSize.width)* 0.4));
		}
	}
	for(int i = 0; i < contours.size(); i++){
		if(similarity[i] < similarity[closest]){
			closest = i;
		}
	}
	for(int i = 0; i < contours.size(); i++){
		if((similarity[i] < similarity[i-1]) && i != closest){
			secClose = i;
		}
	}
	
	oneRect = boundingRect(contours.at(closest));
	theOtherRect = boundingRect(contours.at(secClose));

	leftBound = ((400 + offset) - thresholdX);
	rightBound = ((400 + offset) + thresholdX);
	
	if((leftBound < 0) || (rightBound > 800)){
		leftBound = 350;
		rightBound = 450;
	}
	if(leftBound > 800 - rightBound){
		unitSize = leftBound / 10;
	} else {
		unitSize = (800 - rightBound) / 10;
	}

	line(img, Point(leftBound, 0), Point(leftBound, 600), Scalar(0,0,0));
	line(img, Point(rightBound, 0), Point(rightBound, 600), Scalar(0,0,0));
	/*for(int i = (leftBound + unitSize); i < 800; i = i + unitSize){
		line(img, Point(i, 275), Point(i, 325), Scalar(0,0,0));
	}
	for(int i = (rightBound - unitSize); i < 800; i = i - unitSize){
		line(img, Point(i, 275), Point(i, 325), Scalar(0,0,0));
	}*/
	
	avgX = ((oneRect.x + (oneRect.width/2)) + (theOtherRect.x + (theOtherRect.width/2)))/2;
	if(avgX > 0 && avgX < 800){
		if ((avgX >= leftBound) && (avgX <= rightBound)){
			rectangle(img, Point(oneRect.x, oneRect.y), Point(oneRect.x + oneRect.width, oneRect.y + oneRect.height), Scalar(0, 255, 0), 2);
			rectangle(img, Point(theOtherRect.x, theOtherRect.y), Point(theOtherRect.x + theOtherRect.width, theOtherRect.y + theOtherRect.height), Scalar(0, 255, 0), 2);
		}else{
			rectangle(img, Point(oneRect.x, oneRect.y), Point(oneRect.x + oneRect.width, oneRect.y + oneRect.height), Scalar(0, 0, 255), 2);
			rectangle(img, Point(theOtherRect.x, theOtherRect.y), Point(theOtherRect.x + theOtherRect.width, theOtherRect.y + theOtherRect.height), Scalar(0, 0, 255), 2);
		}
	}
	if((avgX >= leftBound) && (avgX <= rightBound)){
		xCorrectionLevel = 0;
	}else{
		leftSide = abs(avgX - leftBound);
		rightSide = abs(avgX - rightBound);
		if(rightSide < leftSide){
			xCorrectionLevel = (int) rightSide/unitSize;
		}else{
			xCorrectionLevel = (int) (rightSide/unitSize) * -1;
		}
	}
	char output[256];
	sprintf(output, "%d\n", xCorrectionLevel);
	send(sockfd, output, strlen(output)+1, 0);
	
	//clock calculations
	clock_gettime(CLOCK_REALTIME, &tsEnd);
	/*printf("Start2End=%7.6f cvtColor=%7.6f inRange=%7.6f Blur=%7.6f\n",
		   TimeDiffInSec(&tsStart, &tsEnd),
		   TimeDiffInSec(&tsStart, &tscvtColor),
		   TimeDiffInSec(&tscvtColor, &tsInRange),
		   TimeDiffInSec(&tsInRange, &tsBlur));*/
Exit:
	clock_gettime(CLOCK_REALTIME, &tsStart);							   
	Mat tmp;
	cv::resize(img, tmp, Size(320, 240));
	img = tmp;
}

void filter_process(void* filter_ctx, Mat &src, Mat &dst) {
	ws_process(src);
	
	dst = src;
}


void filter_free(void* filter_ctx) {

}

static double TimeDiffInSec(timespec *start, timespec *stop)
{
	double	TimeInSec;
	double	StartTimeInSec;
	double	EndTimeInSec;
	int		test;

	//cout << "Start: ";
	//timespecDisplay(start);
	//cout << endl;
    //
	//cout << "Stop:  ";
	//timespecDisplay(stop);
	//cout << endl;


	//StartTimeInSec 	= start->tv_sec + start->tv_nsec/1000000000.;
	//EndTimeInSec 	= stop->tv_sec + stop->tv_nsec/1000000000.;

	if ((stop->tv_nsec - start->tv_nsec) < 0) {
		test=0;

		TimeInSec = stop->tv_sec - start->tv_sec - 1.;
		TimeInSec += (stop->tv_nsec - start->tv_nsec + 1000000000)/1000000000.;

    } else {
		test=1;

		TimeInSec = stop->tv_sec - start->tv_sec;
		TimeInSec += (stop->tv_nsec - start->tv_nsec)/1000000000.;
	}

	//cout << "test=" << test << " TimeInSec = " << TimeInSec << " Calc 2 = " << EndTimeInSec - StartTimeInSec <<  endl;

	return TimeInSec;
}

static void timespecDisplay(timespec *time)
{
	cout << "time->tv_sec = " << time->tv_sec << " time->tv_nsec = " << time->tv_nsec;
}