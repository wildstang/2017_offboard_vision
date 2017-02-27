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
#define WS_USE_ORIGINAL_WS_PROCESS	// Uncomment to use the "empty" version. Useful for meauring frame rate without processing

static bool NODASHBOARD = false;
static int NUM_AVERAGES = 5;
static int BRIGHTNESS = 30;
static int CONTRAST = 5;
static int SATURATION = 200;

#define	STRIP_HEIGHT	5.0	// Strip Height in Inches
#define	STRIP_WIDTH		2.5	// Strip Width in Inches

static double TimeDiffInSec(timespec *start, timespec *stop);
static void timespecDisplay(timespec *time);
static int SocketReadln(int Socket, char *DestBuf, int MaxNumChars);

static timespec tsPrev;	// Previous Time


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
    server = gethostbyname("10.1.11.46");
    if (server == NULL) {
        cout << "ERROR, no such host" << endl;
        exit(0);
    }

    bzero((char *) &serv_addr, sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    bcopy((char *)server->h_addr, (char *)&serv_addr.sin_addr.s_addr, server->h_length);
    serv_addr.sin_port = htons(portno);

	//
	// P.Poppe 2/18/2017
	//	Wait for the connection...
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

	//
	// P.Poppe 2/18/2017
	// Watch out!!! recv() does not necessarily receive all the data that was sent...
	//
	//n = SocketReadln(sockfd, buffer, 255);
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

	clock_gettime(CLOCK_REALTIME, &tsPrev);

	return true;
}


bool wayToSort(int i, int j) { return i > j; }

#ifdef WS_USE_ORIGINAL_WS_PROCESS
void ws_process(Mat& img) {
	
	Mat hsvMat(img.size(), img.type());
	Mat hsvOut;
	Rect oneRect;
	Rect theOtherRect;
	int leftBound;
	int rightBound;
	int xCorrectionLevel;
	timespec tsStart;
	timespec tsEnd;


	clock_gettime(CLOCK_REALTIME, &tsStart);

	// 
	//  Blur the image
	// 
	Mat	blurInput = img;
	Mat blurOutput;
	double doubleRadius	= 5.0;
	int radius 			= (int)(doubleRadius + 0.5);
	int kernelSize 		= 2*radius + 1;
	blur(blurInput, blurOutput, Size(kernelSize, kernelSize));
	//GaussianBlur(hsvOut, blurMat, Size(0, 0), 3.0);

	// 
	//  Convert img(BGR) -> hsvMat(HSV) color space
	// 
	Mat	cnvInput = blurOutput;
	Mat	cnvOutput;
	cvtColor(cnvInput, cnvOutput, COLOR_BGR2HSV);

	// 
	// Find the pixels that are within the the specified range and place into hsvOut
	// 
	Mat	inRangeInput = cnvOutput;
	Mat inRangeOutput;
	//inRange(hsvMat, Scalar(0, 48, 0), Scalar(100, 180, 246), hsvOut);	// Poppe Basement
	//inRange(hsvMat, Scalar(11, 0, 0), Scalar(87, 150, 255), hsvOut);	// Classroom
	//inRange(hsvMat, Scalar(142, 0, 225), Scalar(180, 255, 255), hsvOut);	// Gym with Light TBD
	inRange(inRangeInput, Scalar(81, 0, 238), Scalar(180, 255, 255), inRangeOutput);	// Poppe Basement with Light

	// 
	// find the contours
	// 
	Mat	findRangeInput = inRangeOutput;
	vector < vector<Point> > contours;
	vector < Vec4i > hierarchy;
	findContours(findRangeInput, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);		// No Blur
	//findContours(blurMat, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);	// Using Blurred Image

	vector <double> similarity(contours.size());


	int closest = 0;
	int secClose = 0;

	//
	// If there are not at least 2 contours found, just return since there is nothing to do.
	//
	if (contours.size() <= 1) {
		// nothing to do so just return
		goto Exit;
	}


	//
	// Go through the contours looking calculating the aspect ratios which can be used to find the
	// rectangles that we are looking for.
	//
	for (int i = 0; i < contours.size(); i++) {
		Rect currentSize = boundingRect(contours[i]);

		double MeasuredHeight 	= currentSize.height;
		double MeasuredWidth 	= currentSize.width;

		if (((MeasuredHeight < 20) || (MeasuredWidth < 10)) || 
			 (MeasuredHeight > 500) || (MeasuredWidth > 700)) {
			similarity[i] = 10.;
		}
		else
		{
			// *************************************************************************************************
			// The similarity is calculated as follows:
			// 
			// 			 Measured Height   STRIP_WIDTH
			// fabs(1 -  --------------- x -----------  )
			//			 Measured Width	   STRIP_HEIGTH
			// 
			// This calculation should produce a range:
			// 			0 <= similarity <= 1.0
			// 
			// Essentially, this is trying to find the object with the closest aspect ration to the one we are trying to find.
			//
			// The closer this is to 0 the better the chance that we found the object that we are looking for!!!
			// *************************************************************************************************
			similarity[i] = fabs(1.0-((MeasuredHeight / MeasuredWidth)* STRIP_WIDTH/STRIP_HEIGHT));
		}
        
		//if (similarity[i] < 10.) {
		// 	// Display the "Potentially good" rectangles
		//	rectangle(img, Point(currentSize.x, currentSize.y), Point(currentSize.x + currentSize.width, currentSize.y + currentSize.height), Scalar(0, 255, 0), 2);
		//}

		//printf("i=%d similarity=%f Height=%d Width=%d\n" ,i, similarity[i], (int) MeasuredHeight, (int) MeasuredWidth);
	}

	// go through looking for the smallest which repesents the region of intrest that we are interested in
	closest = 0;
	for(int i = 0; i < contours.size(); i++){
		if(similarity[i] < similarity[closest]){
			closest = i;
		}
	}

	// Make sure to start the second closest someplace other than the lowest 
	// otherwise second closest might equal closest which we don't want!
	if (closest == 0) {
		secClose = 1;
	}
	else
		secClose = 0;

	// OK, go about finding the second closest contour which represents the aspect ration that we are looking for
	for(int i = 0; i < contours.size(); i++){
		if (i != closest) {
			if(similarity[i] < similarity[secClose]){
				secClose = i;
			}
		}
	}

	//printf("closest=%d secClose=%d\n", closest, secClose);
	{
		// Get the rectangles that represent the regions of intrest
		oneRect 		= boundingRect(contours.at(closest));
		theOtherRect 	= boundingRect(contours.at(secClose));

		// calculate and place some lines representing the "dead on" region
		leftBound = ((400 + offset) - thresholdX);
		rightBound = ((400 + offset) + thresholdX);

		if((leftBound < 0) || (rightBound > 800)){
			leftBound = 350;
			rightBound = 450;
		}

		int unitSize = 1;

		if(leftBound > 800 - rightBound){
			unitSize = leftBound / 10;
		} else {
			unitSize = (800 - rightBound) / 10;
		}

		// Display the "dead on" region as 2 vertical lines
		line(img, Point(leftBound, 0), Point(leftBound, 600), Scalar(0,0,0));
		line(img, Point(rightBound, 0), Point(rightBound, 600), Scalar(0,0,0));

		// 
		// Calculate the center line between to 2 rectanges that were found.
		//
		int 	avgX;
		int		iNumPixels 	= abs(theOtherRect.x-oneRect.x);

		if(oneRect.area() > 0 && theOtherRect.area() > 0){
			avgX = ((oneRect.x + (oneRect.width/2)) + (theOtherRect.x + (theOtherRect.width/2)))/2;

			// Put a line on the "calculated" center
			for (int i = -1; i <= 1; i++) {
				line(img, Point(avgX+i, 0), Point(avgX+i, 600), Scalar(255,0,0));
			}

			printf("avgX=%d oneRect.x=%d oneRect.width=%d theOtherRect.x=%d theOtherRect.width=%d iNumPixels=%d\n", 
				   avgX, 
				   oneRect.x,
				   oneRect.width,
				   theOtherRect.x,
				   theOtherRect.width,
				   iNumPixels
				   );

			if(avgX > 0 && avgX < 800){     
				if ((avgX >= leftBound) && (avgX <= rightBound)){
					rectangle(img, Point(oneRect.x, oneRect.y), Point(oneRect.x + oneRect.width, oneRect.y + oneRect.height), Scalar(0, 255, 0), 2);
					rectangle(img, Point(theOtherRect.x, theOtherRect.y), Point(theOtherRect.x + theOtherRect.width, theOtherRect.y + theOtherRect.height), Scalar(0, 255, 0), 2);
				}else{
					rectangle(img, Point(oneRect.x, oneRect.y), Point(oneRect.x + oneRect.width, oneRect.y + oneRect.height), Scalar(0, 0, 255), 2);
					rectangle(img, Point(theOtherRect.x, theOtherRect.y), Point(theOtherRect.x + theOtherRect.width, theOtherRect.y + theOtherRect.height), Scalar(0, 0, 255), 2);
				}
			}

			//
			// Calculate the "correction value that is to be sent back to the RoboRIO
			//
			if((avgX >= leftBound) && (avgX <= rightBound)){
				xCorrectionLevel = 0;
			}else{
				int leftSide = abs(avgX - leftSide);
				int rightSide = abs(avgX - rightSide);
				if(rightSide < leftSide){
					xCorrectionLevel = (int) rightSide/unitSize;
				}else{
					xCorrectionLevel = (int) (rightSide/unitSize) * -1;
				}
			}

			//
			// Send back any values to the RoboRIO
			//
			char output[256];
			static int	Parm1;
			int	Parm2	= 2;
			int	Parm3	= 3;			

			Parm1++;
			sprintf(output, "%d,%d,%d,%d\n", xCorrectionLevel, Parm1, Parm2, Parm3);
			send(sockfd, output, strlen(output)+1, 0);
		}
		else {
			printf("Why did we get here? oneRect.area()=%d theOtherRect.area()=%d\n", oneRect.area(), theOtherRect.area());
		}
	}

Exit:

	// just resize the final image the will be sent back to the browser
	Mat tmp;
	cv::resize(img, tmp, Size(320, 240));
	img = tmp;

	clock_gettime(CLOCK_REALTIME, &tsEnd);

	double Frame2FrameTimeInSec = TimeDiffInSec(&tsPrev, &tsStart);

	printf("F2F=%5.4f S2E=%5.4f fps=%5.3f\n\n",
		   Frame2FrameTimeInSec,
		   TimeDiffInSec(&tsStart, &tsEnd),
		   1./Frame2FrameTimeInSec);

	tsPrev = tsStart;
}
#else //WS_USE_ORIGINAL_WS_PROCESS
// Put together an empty process just for time measurement
void ws_process(Mat& img) {
	
	timespec tsStart;
	timespec tsEnd;

	clock_gettime(CLOCK_REALTIME, &tsStart);

Exit:

	Mat tmp;
	cv::resize(img, tmp, Size(400, 300));
	img = tmp;
	clock_gettime(CLOCK_REALTIME, &tsEnd);

	double Frame2FrameTimeInSec = TimeDiffInSec(&tsPrev, &tsStart);

	printf("F2F=%5.4f S2E=%5.4f fps=%5.3f\n",
		   Frame2FrameTimeInSec,
		   TimeDiffInSec(&tsStart, &tsEnd),
		   1./Frame2FrameTimeInSec);

	tsPrev = tsStart;
}
#endif //WS_USE_ORIGINAL_WS_PROCESS

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

static int SocketReadln(int Socket, char *DestBuf, int MaxNumChars)
{
	// This function reads the socket, filling it until either the MaxNumChars have been
	// read in or a linefeed ("\n") has been read in
	char	*BufPtr = DestBuf;
	char	c;
	int		NumRcvd;
	bool	Done = false;
	int		i = 0;

	while (Done == false) {
		NumRcvd = recv(sockfd, &c, 1, 0);
		if (NumRcvd <= 0) {
			i = NumRcvd;
			Done = true;
			continue;
		}
		else {
			BufPtr[i] = c;
			i++;
		}

		if ((c == '\n') || (i >= MaxNumChars-1)) {
			BufPtr[i] = 0;
			i++;
			Done = true;
		}
	}

	return (i);
}
