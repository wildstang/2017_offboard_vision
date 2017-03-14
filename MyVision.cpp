// This is the vision software that it use with the Raspberry Pi 3 for
// the 2017 FRC Steamworks Competition by:
// 
// TEAM:	111 Wildstang
// 
// This software is provided as-is and no guarantees are given that
// it even works!
// 
// Use at you own risk!!!
// 

// *******************************************
// Include files
// *******************************************
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
#include <semaphore.h>

#include <sys/time.h>

// *******************************************
// Namespaces... This is C++ after all...
// *******************************************
using namespace cv;
using namespace std;

// *******************************************
// Defines & Constants
// *******************************************
#define TRACE				// adds additional printf debug information
#define WS_USE_SOCKETS
//#define	ROBORIO_IP_ADDRESS	"10.1.11.38"	// VisionTest on PC via WiFi
//#define	ROBORIO_IP_ADDRESS	"10.1.11.46"		// VisionTest on PC via direct connect
#define	ROBORIO_IP_ADDRESS	"10.1.11.2"		// Actual RoboRIO
#define	ROBORIO_PORT_ADDRESS	5800			// Port Number on PC/RoboRIO

#define	STRIP_HEIGHT		5.0	// Strip Height in Inches
#define	STRIP_WIDTH			2.5	// Strip Width in Inches

#define IMAGE_HEIGHT_P		480 
#define	IMAGE_WIDTH_P		864
#define	IMAGE_CENTERLINE_P	(IMAGE_WIDTH_P/2)

// The calcluation for distance is as follows:
//
// distance = f(pixels) = DISTANCE_A + DISTANCE_B/pixels
// 
#define DISTANCE_A			-1.6454052
#define DISTANCE_B			5124.81531

// *******************************************
// Local Structures
// *******************************************

// *******************************************
// Local Variables
// *******************************************
static timespec tsPrev;	// Previous Time
static pthread_t tid;	// Thread Id for the processing Image Processing Thread (ws_process())

static int m_H_MIN 		= 0;
static int m_S_MIN 		= 0;
static int m_V_MIN 		= 245;
  
static int m_H_MAX 		= 0;
static int m_S_MAX 		= 0;
static int m_V_MAX 		= 255;

static int offset 		= 0;
static int thresholdX 	= 50;
static double blurRadius= 5.0;

static int sockfd;

static sem_t sem_ImageProcessed;
static sem_t sem_ImageAvailableToProcess;

static Mat SlaveProcessImage;	// Image passed between main thread and image processing thread.
static bool FirstTime = true;
static bool RunThread = true;

// *******************************************
// Local Prototypes
// *******************************************
static void* SlaveImgProcessThread(void *arg);
static void ws_process(Mat& img);
static double TimeDiffInSec(timespec *start, timespec *stop);
static void timespecDisplay(timespec *time);
static int SocketReadln(int Socket, char *DestBuf, int MaxNumChars);
static double Distance(int pixelWidth);
static void error(const char *msg);
static void ResizeImage(Mat &src, Mat &dst);
static bool ContourLocator(vector < vector<Point> > &contours, int &closest, int &secClose);


// exports for the filter
extern "C" {
    bool filter_init(const char * args, void** filter_ctx);
    void filter_process(void* filter_ctx, Mat &src, Mat &dst);
    void filter_free(void* filter_ctx);
}


// *******************************************
// Global Function Definitions
// *******************************************
/**
    Initializes the filter. If you return something, it will be passed to the
    filter_process function, and should be freed by the filter_free function
*/
extern bool filter_init(const char * args, void** filter_ctx) {
	//Set client
	//Set IP address

#ifdef WS_USE_SOCKETS
    int portno, n;

    struct sockaddr_in serv_addr;
    struct hostent *server;

    char buffer[256];
	
    portno = ROBORIO_PORT_ADDRESS;
    cout << "Opening socket" << endl;
    sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0)
    {
    	cout << "Could not open socket" << endl;
        error("ERROR opening socket");
    }

    cout << "Getting host name" << endl;
    server = gethostbyname(ROBORIO_IP_ADDRESS);
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
    cout << "Attempting to connect: " << ROBORIO_IP_ADDRESS << " Port: " << ROBORIO_PORT_ADDRESS << endl;
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
		if (count == 8)
		{
			blurRadius = atof(token);
		}

		token = strtok(NULL, "|\n");
		count++;
	}

	printf("Hmin = %d\nSmin = %d\nVmin = %d\nHmax = %d\nSmax = %d\nVmax = %d\nOffset = %d\nThreshold = %d\nBlur Radius = %f\n", m_H_MIN, m_S_MIN, m_V_MIN, m_H_MAX, m_S_MAX, m_V_MAX, offset, thresholdX, blurRadius);
#endif

	// Create the semaphores for signalling that to the slave thread that there is work to be done and
	// for the slave thread to signal when it has completed processing the image.
	sem_init(&sem_ImageProcessed, 0, 1); 			// set to initial value of 1 indicating that the "image" is ready to the main thread.
	sem_init(&sem_ImageAvailableToProcess, 0, 0); 	// set to initial value of 0 indicating that the "image" is not available to the slave thread

	// Spawn off the worker thread
	int err;
	err = pthread_create(&tid, NULL, &SlaveImgProcessThread, NULL);
	if (err != 0)
		printf("\ncan't create thread :[%s]", strerror(err));
	else
		printf("\n Thread created successfully\n");

	{
		char	SystemCmd[128];
		int		exposure_absolute = 100;

		// Turn off auto focus and set the exposure value on the camera
		//system("v4l2-ctl -d /dev/video0 -c focus_auto=1");

		// Turn off auto exposure and set the exposure value on the camera
		system("v4l2-ctl -d /dev/video0 -c exposure_auto=1");
		sprintf(SystemCmd, "v4l2-ctl -d /dev/video0 -c exposure_absolute=%d", exposure_absolute);
		system(SystemCmd);
	}

	clock_gettime(CLOCK_REALTIME, &tsPrev);


	return true;
}


extern void filter_process(void* filter_ctx, Mat &src, Mat &dst) {


	// Wait for the slave thread to indicate that it has completed processing the previous image
	sem_wait(&sem_ImageProcessed);

	if (FirstTime == true) {
		//
		// This first time through, there is no image to return so just resize the original image
		// to the same size that the other images will be and send that back.
		ResizeImage(src, dst);
		dst = src;
		FirstTime = false;
	}
	else {
		// OK, the slave thread has completed processing the image so copy to the destination
		dst = SlaveProcessImage.clone();
	}

	// Copy the source image to the SlaveProcessImage so that the slave gets the updated image
	SlaveProcessImage = src.clone();

	// Tell the slave process that another image is available for it to process.
	sem_post(&sem_ImageAvailableToProcess);
}


void filter_free(void* filter_ctx) {

}

// *******************************************
// Local Function Definitions
// *******************************************
static void* SlaveImgProcessThread(void *arg)
{
	timespec 	tsStart;
	timespec 	tsEnd;
	double 		Frame2FrameTimeInSec = 0.0;

	// This is the slave thread which is responsible for the following:
	// 
	// 1. Wait for an image
	// 2. Process the image via ws_process()
	// 3. Resize the image
	// 4. Let the main thread know that the image is ready to be sent out to the SmartDashboard
	// 

	printf("%s(): Started\n", __FUNCTION__);

	while (RunThread == true ) {

		// Wait for the main thread to tell this thread that an image is available to be processed
		sem_wait(&sem_ImageAvailableToProcess);

		clock_gettime(CLOCK_REALTIME, &tsStart);

		// Process the image
		ws_process(SlaveProcessImage);

		// Resize since the image here so it is done by the slave process thread
		ResizeImage(SlaveProcessImage, SlaveProcessImage);

		clock_gettime(CLOCK_REALTIME, &tsEnd);

		// Display the image processing time and frame rate
		Frame2FrameTimeInSec = TimeDiffInSec(&tsPrev, &tsStart);
		printf("F2F=%5.4f S2E=%5.4f fps=%5.3f\n\n",
			   Frame2FrameTimeInSec,
			   TimeDiffInSec(&tsStart, &tsEnd),
			   1./Frame2FrameTimeInSec);
		tsPrev = tsStart;

		// Let the main thread know that the image has been processed
		sem_post(&sem_ImageProcessed);
	}

	return NULL;
}

static void ws_process(Mat& img) {
	// The ws_process is responsible for processing the image and sending heading and distance information
	// back to the RoboRIO. This is done as a separate thread in parallel to the main thread which is collecting the
	// image frame from the camera and sending the output from to the RoboRIO.
	
	Mat hsvMat(img.size(), img.type());
	Mat hsvOut;
	
	Rect oneRect;
	Rect theOtherRect;
	int middleOfOneRect;
	int middleOfOtherRect;
	int distanceBetweenRects;
	
	int leftBound;
	int rightBound;
	int weightingBound;
	double xCorrectionLevel = 0.0;
	double DistanceFromWall = 0.0;
	
	//printf("img.cols=%d img.rows=%d\n", img.cols, img.rows); 

	// 
	//  Blur the image
	// 

	blurRadius = 7.0;


	Mat	blurInput = img;
	Mat blurOutput;
	int radius 			= (int)(blurRadius + 0.5);
	int kernelSize 		= 2*radius + 1;
	blur(blurInput, blurOutput, Size(kernelSize, kernelSize));
	//GaussianBlur(hsvOut, blurMat, Size(0, 0), 3.0);

	// 
	//  Convert img(BGR) -> hsvMat(HSV) color space
	// 
	//Mat	cnvInput = img;
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
	//inRange(inRangeInput, Scalar(81, 0, 238), Scalar(180, 255, 255), inRangeOutput);	// Poppe Basement with Light
	inRange (inRangeInput, Scalar(m_H_MIN, m_S_MIN, m_V_MIN), Scalar(m_H_MAX, m_S_MAX, m_V_MAX), inRangeOutput);

	// 
	// find the contours
	// 
	Mat	findRangeInput = inRangeOutput;
	vector < vector<Point> > contours;
	vector < Vec4i > hierarchy;
	findContours(findRangeInput, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);
	//for(int i = 0; i < contours.size())
	int closest = 0;
	int secClose = 0;
	bool shouldContinue = ContourLocator(contours, closest, secClose);
	cout<<"closest: "<<closest<<endl;
	cout<<"secClose: "<<secClose<<endl;
	if(!shouldContinue){
		goto Exit;
	}

	//printf("closest=%d secClose=%d\n", closest, secClose);

	// Get the rectangles that represent the regions of intrest
	oneRect 		= boundingRect(contours.at(closest));
	theOtherRect 	= boundingRect(contours.at(secClose));

	// Make sure the closest is on left and secClose is on the right
	if (oneRect.x > theOtherRect.x) {
		int TempValue;

		TempValue = secClose;
		secClose = closest;
		closest = TempValue;

		oneRect 		= boundingRect(contours.at(closest));
		theOtherRect 	= boundingRect(contours.at(secClose));
	}

	// calculate and place some lines representing the "dead on" region
	leftBound = ((IMAGE_CENTERLINE_P + offset) - thresholdX);
	rightBound = ((IMAGE_CENTERLINE_P + offset) + thresholdX);

	if((leftBound < 0) || (rightBound > IMAGE_WIDTH_P)){
		leftBound  = IMAGE_CENTERLINE_P - thresholdX;
		rightBound = IMAGE_CENTERLINE_P + thresholdX;
	}

	if(leftBound > IMAGE_WIDTH_P - rightBound){
		weightingBound = leftBound;
	} else {
		weightingBound = rightBound;
	}

	// Draw the left and right bounds indicating the "dead-on" zone
	line(img, Point(leftBound, 0), Point(leftBound, IMAGE_HEIGHT_P), Scalar(0,255,0), 2);
	line(img, Point(rightBound, 0), Point(rightBound, IMAGE_HEIGHT_P), Scalar(0,255,0), 2);
	

	// 
	// Calculate the center line between to 2 rectanges that were found.
	//
	int avgX;

	if(oneRect.area() > 0 && theOtherRect.area() > 0){
		middleOfOneRect = oneRect.x + (oneRect.width / 2);
		middleOfOtherRect = theOtherRect.x + (theOtherRect.width / 2);
		
		avgX = (middleOfOneRect + middleOfOtherRect)/2;
		distanceBetweenRects = abs(middleOfOneRect - middleOfOtherRect);
		int	iNumPixels 	= distanceBetweenRects;

		// Put a line on the "calculated" center
		line(img, Point(avgX, 0), Point(avgX, IMAGE_HEIGHT_P), Scalar(0, 0, 0), 3);
		
		// Put lines on diagonals of rectangles	
		line(img, Point(oneRect.x, oneRect.y), Point(oneRect.x + oneRect.width, oneRect.y + oneRect.height), Scalar(0, 0, 0), 2);
		line(img, Point(oneRect.x + oneRect.width, oneRect.y), Point(oneRect.x, oneRect.y + oneRect.height), Scalar(0, 0, 0), 2);
		line(img, Point(theOtherRect.x, theOtherRect.y), Point(theOtherRect.x + theOtherRect.width, theOtherRect.y + theOtherRect.height), Scalar(0, 0, 0), 2);
		line(img, Point(theOtherRect.x + theOtherRect.width, theOtherRect.y), Point(theOtherRect.x, theOtherRect.y + theOtherRect.height), Scalar(0, 0, 0), 2);
		
		printf("avgX=%d oneRect.x=%d oneRect.width=%d theOtherRect.x=%d theOtherRect.width=%d iNumPixels=%d\n", 
			   avgX, 
			   oneRect.x,
			   oneRect.width,
			   theOtherRect.x,
			   theOtherRect.width,
			   iNumPixels
			   );

		if(avgX > 0 && avgX < IMAGE_WIDTH_P){     
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
			int leftSide = abs(avgX - leftBound);
			int rightSide = abs(avgX - rightBound);
			if(rightSide < leftSide){
				xCorrectionLevel = ((double)rightSide/(double)weightingBound);
			}else{
				xCorrectionLevel = (((double)leftSide/(double)weightingBound) * -1);
			}
			printf("leftSide=%4d rightSide=%4d weightingBound=%4d xCorrectionLevel=%4.3f\n", leftSide, rightSide, weightingBound, xCorrectionLevel);
			//cout<<"leftside: "<<leftSide<<endl;
			//cout<<"rightside: "<<rightSide<<endl;
			//cout<<"correction: "<<xCorrectionLevel<<endl;
		}
		
		// Calculate the distance to the wall
		DistanceFromWall = Distance(iNumPixels);

	}
	else {
		printf("Why did we get here? oneRect.area()=%d theOtherRect.area()=%d\n", oneRect.area(), theOtherRect.area());
	}

Exit:
	{
		//
		// Send back any values to the RoboRIO
		//
		char output[256];
		int	Parm2	= 2;
		int	Parm3	= 3;
	
		sprintf(output, "%4.3f,%f,%d,%d\n", xCorrectionLevel, DistanceFromWall, Parm2, Parm3);
		//printf("%4.3f,%5.3f,%d,%d,%d\n", xCorrectionLevel, DistanceFromWall, Parm2, Parm3, weightingBound);
		printf("xCorrectionLevel=%4.3f, DistanceFromWall=%5.3f\n", xCorrectionLevel, DistanceFromWall);
		//cout<<"correction: "<<xCorrectionLevel<<endl;
		//printf("%d,%d,%d,%d\n", xCorrectionLevel, Parm1, Parm2, Parm3);
#ifdef WS_USE_SOCKETS
		send(sockfd, output, strlen(output)+1, 0);
#endif	//WS_USE_SOCKETS
	}

	return;
}

static bool ContourLocator(vector < vector<Point> > &contours, int &closest, int &secClose){
	
	vector <double> similarity(contours.size());
	vector <bool> UsableContour(contours.size());

	//
	// If there are not at least 2 contours found, just return since there is nothing to do.
	//
	if (contours.size() <= 1) {
		// nothing to do so just return
		return false;
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
			UsableContour[i] = false;
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
			// Essentially, this is trying to find the object with the closest aspect ratio to the one we are trying to find.
			//
			// The closer this is to 0 the better the chance that we found the object that we are looking for!!!
			// *************************************************************************************************
			similarity[i] = fabs(1.0-((MeasuredHeight / MeasuredWidth)* STRIP_WIDTH/STRIP_HEIGHT));
			if (similarity[i] > fabs(1.0- STRIP_WIDTH/STRIP_HEIGHT)) {
				UsableContour[i] = false;
			}
			else {
				UsableContour[i] = true;
			}
		}
        
		//if (similarity[i] < 10.) {
		// 	// Display the "Potentially good" rectangles
		//	rectangle(img, Point(currentSize.x, currentSize.y), Point(currentSize.x + currentSize.width, currentSize.y + currentSize.height), Scalar(0, 255, 0), 2);
		//}

		//printf("i=%d similarity=%f Height=%d Width=%d\n" ,i, similarity[i], (int) MeasuredHeight, (int) MeasuredWidth);
	}
	/*int i = 0;
	while(i < contours.size()){
		if(similarity[i] >= 9.9){
			contours.erase (contours.begin() + i);
			similarity.erase (similarity.begin() + i);
		} else {
			i++;
		}
	}*/

#ifdef TRACE
	for(int i = 0; i < contours.size() ; i++){
		Rect testRect;
		int	usable=-1;

		if (UsableContour[i])
			usable = 1;
		else
			usable = 0;

		testRect 	= boundingRect(contours.at(i));
		if (UsableContour[i] == true) {
			printf("BS similarity[i=%2d]=%6.3f Usable=%d x=%3d, w=%3d, y=%3d h=%3d a=%d\n", 
				   i, similarity[i], usable, testRect.x, testRect.width, testRect.y, testRect.height, testRect.area());
		}
	}
	printf("\n");
#endif //TRACE

	// Now lets sort them from best similarity (lowest number) to worst similarity (highest number)
	double 			tmpSimilarity;
	vector<Point>	tmpContour;
	bool			tmpUsableContour;

	for(int i = 0; i < contours.size() ; i++){
		for(int j = 0; j < contours.size()-i-1; j++){
			if (similarity[j] > similarity[j+1]) {

				tmpSimilarity 	= similarity[j];
				similarity[j] 	= similarity[j+1];
				similarity[j+1] = tmpSimilarity;

				tmpContour  	= contours[j];
				contours[j] 	= contours[j+1];
				contours[j+1] 	= tmpContour;

				tmpUsableContour 	= UsableContour[j];
				UsableContour[j] 	= UsableContour[j+1];
				UsableContour[j+1] 	= tmpUsableContour;
			}
		}
	}

#ifdef TRACE
	//for(int i = 0; i < contours.size() ; i++){
	//	Rect testRect;
	//	int	usable=-1;
	//	if (UsableContour[i]) {
	//		usable = 1;
	//	}
	//	else
	//		usable = 0;
    //
	//	testRect 	= boundingRect(contours.at(i));
	//	printf("AS similarity[i=%2d]=%6.3f Usable=%d x=%3d, w=%3d, y=%3d h=%3d a=%d\n", 
	//		   i, similarity[i], usable, testRect.x, testRect.width, testRect.y, testRect.height, testRect.area());
	//}
	//printf("\n");
#endif //TRACE

	int count = 0;
	for(int i = 0; i < UsableContour.size(); i++){
		if(UsableContour[i]){
			count++;
		}
	}

	if(count < 2){
		return false;
	}

	int	MinDiff = 100000000;
	int	testMinDiff;

	for(int i = 0; i < contours.size() ; i++){
		if (UsableContour[i] == false) {
			continue;
		}

		Rect testRect_i;
		testRect_i 	= boundingRect(contours.at(i));

		for(int j = 0; j < contours.size(); j++){
			if ((UsableContour[j] == false) || 
				(i == j)) {
				continue;
			}

			Rect testRect_j;
			testRect_j 	= boundingRect(contours.at(j));

			// This is a test to see if we are EXTREAMLY CLOSE...
			// If we are, both areas are > 400000 so call it good and exit the loops.
			if ((testRect_i.area() > 40000) && 
				(testRect_j.area() > 40000)) {
				closest = i;
				secClose = j;
				i = contours.size();
				j = contours.size();
				printf("EXTREME CLOSEUP!!!\n");
				continue;
			}

			// This test looks for the 2 rectangles that are the most similar in terms of size.
			testMinDiff = abs(testRect_i.y - testRect_j.y) +
				          abs(testRect_i.height - testRect_j.height) +
				          //abs(testRect_i.x - testRect_j.x) +
				          abs(testRect_i.width - testRect_j.width);
			if (testMinDiff < MinDiff) {
				closest = i;
				secClose = j;
				MinDiff = testMinDiff;
			}
		}
	}

#ifdef TRACE
	for(int i = 0; i < contours.size() ; i++){
		Rect testRect;
		testRect 	= boundingRect(contours.at(i));
		if (UsableContour[i] == true) {
			printf("similarity[i=%2d]=%6.3f x=%3d, w=%3d, y=%3d h=%3d a=%d\n", 
				   i, similarity[i], testRect.x, testRect.width, testRect.y, testRect.height, testRect.area());
		}
	}
#endif //TRACE

	return true;
}

static double Distance(int pixelWidth){
	// uses inverse function to approximate distance
	// has really good correlation
	// A and B change based on data collected
	//////////
	//						B
	// distance = A + -------------
	//					pixelWidth
	//
	// A and B defined above
	//////////
	
	return (DISTANCE_A + (DISTANCE_B / pixelWidth));
}

static void ResizeImage(Mat &src, Mat &dst)
{
	Mat tmp;

	cv::resize(src, tmp, Size(299, 166));
	dst = tmp;
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


static void error(const char *msg)
{
    perror(msg);
    exit(0);
}

static bool wayToSort(int i, int j) { return i > j; }


