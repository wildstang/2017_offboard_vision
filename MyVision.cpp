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

#include <unistd.h>	// for sleep()

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
//#define TRACE				// adds additional printf debug information
//#define OVERRIDE_PARAMETER_FILE_READ
//#define DRAW_ALL_CONTOURS
//#define HIGH_CAMERA		// Uncomment if camera is in the high position
//#define MEASURE_TIME		// Uncomment if we want to measure the time for the various processing steps

//#define READ_IMAGE_FILES						// Uncomment if we want to read recorded images and process them 

//#define USE_BLUR								// Uncomment if we want to use the OpenCV Blur (which is slow

#define WS_USE_SOCKETS
//#define	ROBORIO_IP_ADDRESS	"10.1.11.38"	// VisionTest on PC via WiFi
//#define	ROBORIO_IP_ADDRESS	"10.1.11.46"		// VisionTest on PC via direct connect
//#define	ROBORIO_IP_ADDRESS	"10.1.11.222"		// Paul Test Setup
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
#ifdef HIGH_CAMERA
#define DISTANCE_A			-3
#define DISTANCE_B			3917.36
#else // Camera at low position
#define DISTANCE_A			-2.03785849
#define DISTANCE_B			4363.144182
#endif //HIGH_CAMERA

// *******************************************
// Local Structures
// *******************************************
typedef struct {
	char	Directory[256];
	int 	SetNum;
	int 	ImageNumMin;
	int 	ImageNumMax;
} FILE_SET_ENTRY;

// *******************************************
// Local Variables
// *******************************************
static char HomeDir[] 			= "/home/pi/vision/test/";
static char ImageNameFormat[]	= "%03d-Image-%04d.jpg";

static FILE_SET_ENTRY FileSetList[] = {
	// Directory,									SetNum,	ImageNumMin,	ImageNumMax,	
	{ "",											0,		0,				0,				},	// 0
	{ "ImageRec-UIC-Match-01",						0,		2,				45,				},	// 1
	{ "ImageRec-UIC-Match-02",						0,		1,				36,				},	// 2
	{ "ImageRec-UIC-Match-03",						0,		2,				27,				},	// 3
	{ "ImageRec-UIC-Match-04",						0,		2,				27,				},	// 4
	{ "ImageRec-UIC-Match-05",						0,		1,				27,				},	// 5
	{ "ImageRec-UIC-Match-06",						0,  	1,				31,				},	// 6
	{ "ImageRec-UIC-Match-07(Hit Hard-Lost Comms)",	0,		1,				26,				},	// 7
	{ "ImageRec-UIC-Match-08",						1,  	1,				26,				},	// 8
	{ "ImageRec-UIC-Match-09",						0,  	1,				32,				},	// 9
	{ "ImageRec-2017-04-11_broken",					0,  	1,				29,				},	// 10
	{ "ImageRec-2017-04-11_broken",					1,  	1,				29,				},	// 11
	{ "ImageRec-2017-04-11_broken",					2,  	1,				37,				},	// 12
	{ "ImageRec-2017-04-11_broken",					4,  	1,				25,				},	// 13
};

#define PLAYBACK_MATCH	13	// Selected from FileSetList[] above

static char *FileSetName	= FileSetList[PLAYBACK_MATCH].Directory;
static int SetNum_Read		= FileSetList[PLAYBACK_MATCH].SetNum;
static int ImageNumMin_Read	= FileSetList[PLAYBACK_MATCH].ImageNumMin;
static int ImageNumMax_Read	= FileSetList[PLAYBACK_MATCH].ImageNumMax;
static int ImageNum_Read	= FileSetList[PLAYBACK_MATCH].ImageNumMin - 1;	


static timespec tsPrev;					// Previous Time
static pthread_t tid_KeyboardControl;	// Thread Id for KeyboardControlThread()
static pthread_t tid_SocketRead;		// Thread Id for SocketReadThread()
static pthread_t tid_SlaveSaveImage;	// Thread Id for SaveImagesThread()
static pthread_t tid_SlaveImgProc;		// Thread Id for processing Image Processing Thread (ws_process())
static pthread_t tid_SocketConnect;		// Thread Id for thread responsible for connecting to the visionServer/RoboRio

static bool InExtremCloseupMode 	= false;

// These are the parameters that are used for thresholding
static int m_H_MIN 			= 0;
static int m_S_MIN 			= 0;
static int m_V_MIN 			= 0;
static int m_H_MAX 			= 0;
static int m_S_MAX 			= 0;
static int m_V_MAX 			= 0;

// The thresholding parameters above are set to this if we detect that we are in "InExtremCloseupMode"
static int m_H_MIN_close 	= 70;
static int m_H_MAX_close 	= 180;
static int m_S_MIN_close 	= 0;
static int m_S_MAX_close 	= 255;
static int m_V_MIN_close 	= 172;
static int m_V_MAX_close 	= 255;

// The thresholding parameters above are set to this if we detect that we are NOT in "InExtremCloseupMode"
static int m_H_MIN_far 		= 73;
static int m_H_MAX_far 		= 180;
static int m_S_MIN_far 		= 0;
static int m_S_MAX_far 		= 255;
static int m_V_MIN_far 		= 172;
static int m_V_MAX_far 		= 255;

static int offset 		= 0;
static int thresholdX 	= 50;
static double blurRadius= 5.0;

static int	SetNum				= 0;
static int	ImageNum			= 0;
static int	SnapshotImageNum	= 0;

static int sockfd;

static sem_t sem_SocketConnect;
static sem_t sem_SocketRead;
static sem_t sem_ReadImageFile;
static sem_t sem_SaveImage;
static sem_t sem_ImageSaved;
static sem_t sem_ImageProcessed;
static sem_t sem_ImageAvailableToProcess;

static Mat SlaveProcessImage;	// Image passed between main thread and image processing thread.
static Mat SlaveSaveImage;		// Image passed between main thread and SaveImageThread
static bool FirstTime = true;
static bool RunThread = true;
static bool SocketConnected = false;
static bool ImageRecording	= false;
static bool SnapshotImage	= false;

static char filename_log[] 		= "/home/pi/vision/test/ImageRec/%03d-Log.txt";
static char filename_image[]	= "/home/pi/vision/test/ImageRec/%03d-Image-%04d.jpg";
static char filename_snapshot[]	= "/home/pi/vision/test/ImageSnapshot/Snapshot-%04d.jpg";
static FILE *fp_log				= NULL;

// *******************************************
// Local Prototypes
// *******************************************
static void* SocketConnectionThread(void *arg);
static void* SocketReadThread(void *arg);
static void* KeyboardControlThread(void *arg);
static void* SaveImagesThread(void *arg);
static void* SlaveImgProcessThread(void *arg);

static void ws_process(Mat& img);
static double TimeDiffInSec(timespec *start, timespec *stop);
static void timespecDisplay(timespec *time);
static int SocketReadln(int Socket, char *DestBuf, int MaxNumChars);
static double Distance(int pixelWidth);
static void error(const char *msg);
static void ResizeImage(Mat &src, Mat &dst);
static bool ContourLocatorExtremeCloseupMode(vector < vector<Point> > &contours, int &closest, int &secClose);
static bool ContourLocatorFarMode(vector < vector<Point> > &contours, int &closest, int &secClose);
static bool ExtremeCloseupModeGet(vector < vector<Point> > &contours);
static void ThresholdExtremeCloseupSet(void);
static void ThresholdFarSet(void);

static void ImageRecordBegin(void);
static void ImageRecordEnd(void);

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
	// This is the main initialization function that is called by the mjpg-streamer. 
	// This function MUST exist even if it does nothing.

	// Create the semaphores for signalling that to the slave thread that there is work to be done and
	// for the slave thread to signal when it has completed processing the image.
	sem_init(&sem_SocketConnect, 0, 1); 			// set to initial value of 1 indicating that the Socket Connection Thread should attempt to connect 
	sem_init(&sem_SocketRead, 0, 0); 				// set to initial value of 0 indicating that the Socket Read Thread should wait until the Socket Connection Thread has connected

	sem_init(&sem_ImageSaved, 0, 1); 				// set to initial value of 1 indicating that the "image" is ready to the main thread.
	sem_init(&sem_SaveImage,  0, 0); 				// set to initial value of 0 indicating that the "image" is not available to the slave thread

	sem_init(&sem_ReadImageFile, 0, 0); 			// set to initial value of 0 indicating that we are waiting for a keyboard forward or reverse

	sem_init(&sem_ImageProcessed, 0, 1); 			// set to initial value of 1 indicating that the "image" is ready to the main thread.
	sem_init(&sem_ImageAvailableToProcess, 0, 0); 	// set to initial value of 0 indicating that the "image" is not available to the slave thread

	// Spawn off the worker thread
	int err;
	err = pthread_create(&tid_SocketConnect, NULL, &SocketConnectionThread, NULL);
	if (err != 0)
		printf("\ncan't create thread SocketConnectionThread() :[%s]", strerror(err));
	else
		printf("\n SocketConnectionThread(): Thread created successfully\n");

	err = pthread_create(&tid_SocketRead, NULL, &SocketReadThread, NULL);
	if (err != 0)
		printf("\ncan't create thread SocketReadThread() :[%s]", strerror(err));
	else
		printf("\n SocketReadThread(): Thread created successfully\n");

	err = pthread_create(&tid_SlaveImgProc, NULL, &SlaveImgProcessThread, NULL);
	if (err != 0)
		printf("\ncan't create thread SlaveImgProcessThread() :[%s]", strerror(err));
	else
		printf("\n SlaveImgProcessThread(): Thread created successfully\n");

	err = pthread_create(&tid_SlaveSaveImage, NULL, &SaveImagesThread, NULL);
	if (err != 0)
		printf("\ncan't create thread SaveImagesThread() :[%s]", strerror(err));
	else
		printf("\n SaveImagesThread(): Thread created successfully\n");

	clock_gettime(CLOCK_REALTIME, &tsPrev);


	err = pthread_create(&tid_KeyboardControl, NULL, &KeyboardControlThread, NULL);
	if (err != 0)
		printf("\ncan't create thread KeyboardControlThread() :[%s]", strerror(err));
	else
		printf("\n KeyboardControlThread(): Thread created successfully\n");

	clock_gettime(CLOCK_REALTIME, &tsPrev);



	return true;
}


extern void filter_process(void* filter_ctx, Mat &src, Mat &dst) {
	// This is the main fil;ter function that is called once per imaged. It is called by the mjpg-streamer. 
	// This function MUST exist even if it does nothing.


	// Wait for the slave thread to indicate that it has completed processing the previous image
	sem_wait(&sem_ImageProcessed);
	sem_wait(&sem_ImageSaved);

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
	SlaveProcessImage 	= src.clone();
	SlaveSaveImage 		= src.clone();

	// Tell the slave process that another image is available for it to process.
	ImageNum++;
	sem_post(&sem_ImageAvailableToProcess);
	sem_post(&sem_SaveImage);
}


void filter_free(void* filter_ctx) {
	// This is the main cleanup/termination  function that is called by the mjpg-streamer. 
	// This function MUST exist even if it does nothing.

}

// *******************************************
// Local Function Definitions
// *******************************************
static void* SocketConnectionThread(void *arg)
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
		sem_wait(&sem_SocketConnect);
		
		//Set client
		//Set IP address

#ifdef WS_USE_SOCKETS
		int portno, n;
	
		struct sockaddr_in serv_addr;
		struct hostent *server;
	
		char buffer[256];
	
		printf("CV_MAJOR_VERSION=%d CV_MINOR_VERSION=%d\n", CV_MAJOR_VERSION, CV_MINOR_VERSION);
		
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
		// For the moment, sleep for a bit assuming that the server eill send the relevent information right away.
		sleep(2);
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

		// Let the SocketReadThread() know that it is safe to begin waiting for commands
		sem_post(&sem_SocketRead);

		SocketConnected = true;
#endif
	}

	return NULL;
}

static void* SocketReadThread(void *arg)
{
	// This is the slave thread which is responsible for the following:
	// 
	// 1. Wait for an image
	// 2. Process the image via ws_process()
	// 3. Resize the image
	// 4. Let the main thread know that the image is ready to be sent out to the SmartDashboard
	// 
	int 	n;
	char 	ch;

	printf("%s(): Started\n", __FUNCTION__);

	// Wait for the main thread to tell this thread that an image is available to be processed
	sem_wait(&sem_SocketRead);

	while (RunThread == true ) {

		n = recv(sockfd, &ch, 1, 0);
		cout << "Received data" << endl;
		if (n < 0)
		{
			ImageRecordEnd();
			sem_wait(&sem_SocketRead);
		}

		switch (ch) {
			case 'b':
			case 'B':
				ImageRecordBegin();
				break;

			case 'e':
			case 'E':
				ImageRecordEnd();
				break;

			case 's':
			case 'S':
				SnapshotImage = true;
				break;
		}
	}

	return NULL;
}

static void* SaveImagesThread(void *arg)
{
	bool		SiRunThread = true;
	vector<int> compression_params;
	char		buf[256];

	// This is the slave thread which is responsible for the following:
	// 
	// 1. Wait for an image
	// 2. Save the image to disk
	// 4. Let the main thread know that it is ready to accept another image
	// 

	// Setup the JPEG image compression settings
	compression_params.push_back(CV_IMWRITE_JPEG_QUALITY);
	compression_params.push_back(100);

	printf("%s(): Started\n", __FUNCTION__);

	while (SiRunThread == true ) {

		// Wait for the main thread to tell this thread that an image is available to be saved
		sem_wait(&sem_SaveImage);

		if (ImageRecording == true) {

			sprintf(buf, filename_image, SetNum, ImageNum);

			string filename = buf;

			try {
				imwrite( filename, SlaveSaveImage, compression_params);
				//printf("Image saved: %s\n", buf);
			} 

			catch ( runtime_error& ex ) {
				fprintf(stderr, "Exception converting image to JPEG format: %s\n", ex.what());
			}
		}

		if (SnapshotImage == true) {
			SnapshotImage = false;

			sprintf(buf, filename_snapshot, SnapshotImageNum);

			string filename = buf;

			try {
				imwrite( filename, SlaveSaveImage, compression_params);
				printf("Image saved: %s\n", buf);
			} 

			catch ( runtime_error& ex ) {
				fprintf(stderr, "Exception converting image to JPEG format: %s\n", ex.what());
			}

			SnapshotImageNum++;
		}

		// Let the main thread know that the image has been processed
		sem_post(&sem_ImageSaved);
	}

	return NULL;
}

static void* KeyboardControlThread(void *arg)
{
	bool		KcRunThread = true;

	// This is the slave thread which is responsible for the following:
	// 
	// 1. Gets a keyboard press
	// 2. Sets the parameters as necessary
	// 

	printf("%s(): Started\n", __FUNCTION__);

	while (KcRunThread == true ) {
        //int ch = std::cin.get();
		char ch;

		ch = getchar();

		switch (ch) {
			case 'b':
			case 'B':
				ImageRecordBegin();
				break;
	
			case 'e':
			case 'E':
				ImageRecordEnd();
				break;

			case 's':
			case 'S':
				SnapshotImage = true;
				break;

			case 'f':
			case 'F':
				ImageNum_Read++;
				if (ImageNum_Read > ImageNumMax_Read) {
					ImageNum_Read = ImageNumMin_Read;
				}
				sem_post(&sem_ReadImageFile);
				break;
			case 'r':
			case 'R':
				ImageNum_Read--;
				if (ImageNum_Read < ImageNumMin_Read) {
					ImageNum_Read = ImageNumMax_Read;
				}
				sem_post(&sem_ReadImageFile);
				break;
		}
	}

	return NULL;
}

static void* SlaveImgProcessThread(void *arg)
{
	timespec 	tsStart;
	timespec 	tsEnd;
	double 		Frame2FrameTimeInSec = 0.0;
	char		TrueStr[] 	= "True";
	char		FalseStr[] 	= "False";
	char		*ConnectedStr;
	char		*InExtremeCloseupModeStr;
	char		*ImageRecordingStr;

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

		if (SocketConnected == true)
			ConnectedStr = TrueStr;
		else
			ConnectedStr = FalseStr;

		if (InExtremCloseupMode == true)
			InExtremeCloseupModeStr = TrueStr;
		else
			InExtremeCloseupModeStr = FalseStr;

		if (ImageRecording == true)
			ImageRecordingStr = TrueStr;
		else
			ImageRecordingStr = FalseStr;

		printf("F2F=%5.4f S2E=%5.4f fps=%6.3f Connected=%s ExtremeCloseup=%s ImageRecording=%s\n\n",
			   Frame2FrameTimeInSec,
			   TimeDiffInSec(&tsStart, &tsEnd),
			   1./Frame2FrameTimeInSec, 
			   ConnectedStr,
			   InExtremeCloseupModeStr,
			   ImageRecordingStr);
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
	int boundingEdgeOneRect;
	int boundingEdgeOtherRect;
	int distanceBetweenRects;
	
	int leftBound;
	int rightBound;
	int weightingBound;
	double xCorrectionLevel = 0.0;
	double DistanceFromWall = 0.0;

#ifdef MEASURE_TIME
	timespec 	ltsStart;
	timespec 	ltsEnd;
	double 		TotalProcessTimeInSec = 0.0;

	timespec 	ltsBlurStart;
	timespec 	ltsBlurEnd;
	double 		BlurProcessTimeInSec = 0.0;

	timespec 	ltsImgConvertStart;
	timespec 	ltsImgConvertEnd;
	double 		ImgConvertProcessTimeInSec = 0.0;

	timespec 	ltsInRangeStart;
	timespec 	ltsInRangeEnd;
	double 		InRangeProcessTimeInSec = 0.0;

	timespec 	ltsFindContoursStart;
	timespec 	ltsFindContoursEnd;
	double 		FindContoursProcessTimeInSec = 0.0;

	timespec 	ltsFindRectangleStart;
	timespec 	ltsFindRectangleEnd;
	double 		FindRectanglesProcessTimeInSec = 0.0;

	timespec 	ltsProcessRectangleStart;
	timespec 	ltsProcessRectangleEnd;
	double 		ProcessRectanglesProcessTimeInSec = 0.0;
#endif //MEASURE_TIME

#ifdef READ_IMAGE_FILES
	{
		char		FilenameFormat[256];
		char		cFilename[256];
        Mat image;

		printf("Waiting for 'f' or 'r' <enter>\n");
		sem_wait(&sem_ReadImageFile);

		sprintf(FilenameFormat, "%s/%s/%s", HomeDir, FileSetName, ImageNameFormat);

		sprintf(cFilename, FilenameFormat, SetNum_Read, ImageNum_Read);
		//printf("FilenameFormat=%s\n", FilenameFormat);
		//printf("Filename      =%s\n", cFilename);

		printf("Reading file: %s\n", cFilename);

		string filename = cFilename;
		image = imread( filename, IMREAD_COLOR );
		if(image.empty())
		{
			std::cerr << "Cannot read image file: " << filename << std::endl;
			std::cerr << "Using Image from camera" << std::endl;
		}
		else
			img = image.clone();
	}
#endif //READ_IMAGE_FILES

#ifdef OVERRIDE_PARAMETER_FILE_READ
	{
		int	iblurRadius = 0;
		char *filename = "/home/pi/Override.txt";
		FILE *fp = fopen(filename, "r");
		if (fp == NULL) {
			printf("ERROR: Unable to open file: %s\n", filename);
			goto Continue;
		}
	
		fscanf(fp, "%d %d", &m_H_MIN, &m_H_MAX);
		fscanf(fp, "%d %d", &m_S_MIN, &m_S_MAX);
		fscanf(fp, "%d %d", &m_V_MIN, &m_V_MAX);
		fscanf(fp, "%d", &iblurRadius);
	
		blurRadius = iblurRadius;
	
		fclose (fp);
	}

Continue:
	printf("\n");
	printf("m_H_MIN=%3d, m_H_MAX=%3d\n", m_H_MIN, m_H_MAX);
	printf("m_S_MIN=%3d, m_S_MAX=%3d\n", m_S_MIN, m_S_MAX);
	printf("m_V_MIN=%3d, m_V_MAX=%3d\n", m_V_MIN, m_V_MAX);
	printf("blurRadius=%4.3f\n", blurRadius);
	printf("\n");
#endif //OVERRIDE_PARAMETER_FILE_READ
	
#ifdef MEASURE_TIME
	clock_gettime(CLOCK_REALTIME, &ltsStart);
#endif //MEASURE_TIME

	// hsvMat is a copy of the origianl image that we process an "blur". 
	// We really don't want this image to be sent back to the SmartDashboard though
	hsvMat = img.clone();

	//printf("img.cols=%d img.rows=%d\n", img.cols, img.rows); 

	// 
	//  Blur the image
	// 
#ifdef MEASURE_TIME
	clock_gettime(CLOCK_REALTIME, &ltsBlurStart);
#endif //MEASURE_TIME

#ifdef USE_BLUR
	//blurRadius = 7.0;
	Mat	blurInput = hsvMat;
	Mat blurOutput;
	int radius 			= (int)(blurRadius + 0.5);
	int kernelSize 		= 2*radius + 1;
	blur(blurInput, blurOutput, Size(kernelSize, kernelSize));
	//GaussianBlur(hsvOut, blurMat, Size(0, 0), 3.0);
#endif
#ifdef MEASURE_TIME
	clock_gettime(CLOCK_REALTIME, &ltsBlurEnd);
#endif //MEASURE_TIME

	// 
	//  Convert img(BGR) -> hsvMat(HSV) color space
	// 
#ifdef MEASURE_TIME
	clock_gettime(CLOCK_REALTIME, &ltsImgConvertStart);
#endif //MEASURE_TIME
#ifdef USE_BLUR
	Mat	cnvInput = blurOutput;
#else // USE_BLUR
	Mat	cnvInput = hsvMat;
#endif // USE_BLUR
	Mat	cnvOutput;
	cvtColor(cnvInput, cnvOutput, COLOR_BGR2HSV);

#ifdef MEASURE_TIME
	clock_gettime(CLOCK_REALTIME, &ltsImgConvertEnd);
#endif //MEASURE_TIME
	// 
	// Find the pixels that are within the the specified range and place into hsvOut
	// 
#ifdef MEASURE_TIME
	clock_gettime(CLOCK_REALTIME, &ltsInRangeStart);
#endif //MEASURE_TIME
	Mat	inRangeInput = cnvOutput;
	Mat inRangeOutput;
	//inRange(hsvMat, Scalar(0, 48, 0), Scalar(100, 180, 246), hsvOut);	// Poppe Basement
	//inRange(hsvMat, Scalar(11, 0, 0), Scalar(87, 150, 255), hsvOut);	// Classroom
	//inRange(hsvMat, Scalar(142, 0, 225), Scalar(180, 255, 255), hsvOut);	// Gym with Light TBD
	//inRange(inRangeInput, Scalar(81, 0, 238), Scalar(180, 255, 255), inRangeOutput);	// Poppe Basement with Light
	inRange (inRangeInput, Scalar(m_H_MIN, m_S_MIN, m_V_MIN), Scalar(m_H_MAX, m_S_MAX, m_V_MAX), inRangeOutput);
#ifdef MEASURE_TIME
	clock_gettime(CLOCK_REALTIME, &ltsInRangeEnd);

	clock_gettime(CLOCK_REALTIME, &ltsFindContoursStart);
#endif //MEASURE_TIME
	// 
	// find the contours
	// 
	Mat	findRangeInput = inRangeOutput;
	vector < vector<Point> > contours;
	vector < Vec4i > hierarchy;
	findContours(findRangeInput, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);
#ifdef DRAW_ALL_CONTOURS
	drawContours(img, contours, -1, (255,255,255), 3);
#endif //DRAW_ALL_CONTOURS
#ifdef MEASURE_TIME
	clock_gettime(CLOCK_REALTIME, &ltsFindContoursEnd);
#endif //MEASURE_TIME

	// find the "best" 2 rectangles that match what what we are looking for
	int closest = 0;
	int secClose = 0;
	bool shouldContinue 		= false;

#ifdef MEASURE_TIME
	clock_gettime(CLOCK_REALTIME, &ltsFindRectangleStart);
#endif //MEASURE_TIME
	InExtremCloseupMode = ExtremeCloseupModeGet(contours);

	if (InExtremCloseupMode == true) {
		// set the thresholds for the next go around
		//ThresholdExtremeCloseupSet();

		// now find the ROIs that we are interested in
		shouldContinue = ContourLocatorExtremeCloseupMode(contours, closest, secClose);
	}
	else {
		// set the thresholds for the next go around
		//ThresholdFarSet();

		// now find the ROIs that we are interested in
		shouldContinue = ContourLocatorFarMode(contours, closest, secClose);
	}
#ifdef MEASURE_TIME
	clock_gettime(CLOCK_REALTIME, &ltsFindRectangleEnd);
#endif //MEASURE_TIME

	if (SocketConnected != true) {
		// Make a BIG X indicating that no communication with the RoboRIO is happening
		// There is nothing else to do so just exit.
		line(img, Point(0, 0), Point(img.cols, img.rows), Scalar(0,0,255), 2);
		line(img, Point(0, img.rows), Point(img.cols, 0), Scalar(0,0,255), 2);
		goto Exit;
	}

#ifdef READ_IMAGE_FILES
	{
		char	TmpBuf[256];

		sprintf(TmpBuf, "Set: %03d Image: %03d", SetNum_Read, ImageNum_Read);
        
		string	TmpStr = TmpBuf;

		// put the image number on the image displayed
		cv::putText(img, 
					TmpStr,
					Point(5, IMAGE_HEIGHT_P-20),
					cv::FONT_HERSHEY_SIMPLEX, 		// Font
					1.0, 							// Scale. 2.0 = 2x bigger
					Scalar(0,255,255), 				// Color
					1, 								// Thickness
					CV_AA); 						// Anti-alias
	}
#endif //READ_IMAGE_FILES

	if(!shouldContinue){
		//ThresholdFarSet();
		goto Exit;
	}

	//cout<<"closest: "<<closest<<endl;
	//cout<<"secClose: "<<secClose<<endl;

	printf("closest=%d secClose=%d\n", closest, secClose);

#ifdef MEASURE_TIME
	clock_gettime(CLOCK_REALTIME, &ltsProcessRectangleStart);
#endif //MEASURE_TIME

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
		boundingEdgeOneRect = oneRect.x + oneRect.width;
		boundingEdgeOtherRect = theOtherRect.x;
		
		avgX = (boundingEdgeOneRect + boundingEdgeOtherRect)/2;
		distanceBetweenRects = abs(boundingEdgeOneRect - boundingEdgeOtherRect);
		int	iNumPixels 	= distanceBetweenRects;

		// Put a line on the "calculated" center
		line(img, Point(avgX, 0), Point(avgX, IMAGE_HEIGHT_P), Scalar(0, 255, 255), 3);
		//line(img, Point(avgX, 0), Point(avgX, IMAGE_HEIGHT_P), Scalar(0, 0, 0), 3);
		
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
#ifdef MEASURE_TIME
	clock_gettime(CLOCK_REALTIME, &ltsProcessRectangleEnd);
#endif //MEASURE_TIME

Exit:

#ifdef MEASURE_TIME
	clock_gettime(CLOCK_REALTIME, &ltsEnd);
	TotalProcessTimeInSec 		      = TimeDiffInSec(&ltsStart, &ltsEnd);
	BlurProcessTimeInSec  		      = TimeDiffInSec(&ltsBlurStart, &ltsBlurEnd);
	ImgConvertProcessTimeInSec	      = TimeDiffInSec(&ltsImgConvertStart, &ltsImgConvertEnd);
	InRangeProcessTimeInSec		      = TimeDiffInSec(&ltsInRangeStart, &ltsInRangeEnd);
	FindContoursProcessTimeInSec      = TimeDiffInSec(&ltsFindContoursStart, &ltsFindContoursEnd);
	FindRectanglesProcessTimeInSec    = TimeDiffInSec(&ltsFindRectangleStart, &ltsFindRectangleEnd);
	ProcessRectanglesProcessTimeInSec = TimeDiffInSec(&ltsProcessRectangleStart, &ltsProcessRectangleEnd);

	printf("---------------------------------------\n");
	printf("BlurProcessTimeInSec              = %4.3f\n", BlurProcessTimeInSec);
	printf("ImgConvertProcessTimeInSec        = %4.3f\n", ImgConvertProcessTimeInSec);
	printf("InRangeProcessTimeInSec           = %4.3f\n", InRangeProcessTimeInSec);
	printf("FindContoursProcessTimeInSec      = %4.3f\n", FindContoursProcessTimeInSec);
	printf("FindRectanglesProcessTimeInSec    = %4.3f\n", FindRectanglesProcessTimeInSec);
	printf("ProcessRectanglesProcessTimeInSec = %4.3f\n", ProcessRectanglesProcessTimeInSec);
	printf("TotalProcessTimeInSec             = %4.3f\n", TotalProcessTimeInSec);
	printf("---------------------------------------\n");
#endif //MEASURE_TIME

	if (SocketConnected == true) {
		//
		// Send back any values to the RoboRIO
		//
		char output[256];
		int	Parm2	= 2;
		int	Parm3	= 3;

		sprintf(output, "%4.3f,%f,%d,%d\n", xCorrectionLevel, DistanceFromWall, Parm2, Parm3);
		//printf("%4.3f,%5.3f,%d,%d,%d\n", xCorrectionLevel, DistanceFromWall, Parm2, Parm3, weightingBound);
		printf("xCorrectionLevel=%4.3f, DistanceFromWall=%5.3f\n", xCorrectionLevel, DistanceFromWall);
		if ((ImageRecording == true)  && (fp_log != NULL)) {
			fprintf(fp_log, "ImageNum=%04d xCorrectionLevel=%4.3f, DistanceFromWall=%5.3f\n", ImageNum, xCorrectionLevel, DistanceFromWall);
		}
		//cout<<"correction: "<<xCorrectionLevel<<endl;
		//printf("%d,%d,%d,%d\n", xCorrectionLevel, Parm1, Parm2, Parm3);
#ifdef WS_USE_SOCKETS
		int	ret;
		//printf("%s", output);
		ret = send(sockfd, output, strlen(output)+1, 0);
		if (ret < 0) {
			SocketConnected = false;

			// Tell the slave process that another image is available for it to process.
			sem_post(&sem_SocketConnect);
		}
#endif	//WS_USE_SOCKETS
	}

	return;
}

static bool ContourLocatorExtremeCloseupMode(vector < vector<Point> > &contours, int &closest, int &secClose){
	
	vector <double> similarity(contours.size());
	vector <bool> UsableContour(contours.size());
	int ImageCenterLine = IMAGE_CENTERLINE_P + offset;

	//
	// If there are not at least 2 contours found, just return since there is nothing to do.
	//
	if (contours.size() <= 1) {
		// nothing to do so just return
		return false;
	}

	//
	// Go through the contours calculating the aspect ratios which can be used to find the
	// rectangles that we are looking for.
	//
	for (int i = 0; i < contours.size(); i++) {
		Rect currentSize = boundingRect(contours[i]);
		int rectangleCenterLine = currentSize.x + currentSize.width/2;

		double MeasuredHeight 	= currentSize.height;
		double MeasuredWidth 	= currentSize.width;

		if (((MeasuredHeight < 20) || (MeasuredWidth < 27)) /*|| 
			(MeasuredHeight > 500) || (MeasuredWidth > 700) */) {
			similarity[i] = 10.;
			UsableContour[i] = false;
		}
		else if ((currentSize.area() < 7000) || (currentSize.area() > 50000)) {
			// this is reflection of some form so ignore
			similarity[i] = 15.;
			UsableContour[i] = false;
		}
		else if (abs(rectangleCenterLine-ImageCenterLine) < 50) {
			// This is a blob so ignore
#ifdef TRACE
			printf("i=%d x=%3d, w=%3d, y=%3d h=%3d a=%d rectangleCenterLine=%d ImageCenterLine=%d\n", 
				   i, currentSize.x, currentSize .width, currentSize.y, currentSize.height, currentSize.area(),
				   rectangleCenterLine, ImageCenterLine);
#endif //TRACE

			similarity[i] = 20.;
			UsableContour[i] = false;
			//printf("i=%d UsableContour[i%d] = false\n", i, i);
		}
		else {
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
			//UsableContour[i] = true;

			if (similarity[i] < 1.) {
				UsableContour[i] = true;
			}
			else {
				UsableContour[i] = false;
			}
 //   		if (similarity[i] > fabs(1.0- STRIP_WIDTH/STRIP_HEIGHT)) {
 //   			UsableContour[i] = false;
 //   		}
 //   		else {
 //   			UsableContour[i] = true;
//			}
		}
        
		//if (similarity[i] < 10.) {
		// 	// Display the "Potentially good" rectangles
		//	rectangle(img, Point(currentSize.x, currentSize.y), Point(currentSize.x + currentSize.width, currentSize.y + currentSize.height), Scalar(0, 255, 0), 2);
		//}

		//printf("i=%d similarity=%f Height=%d Width=%d\n" ,i, similarity[i], (int) MeasuredHeight, (int) MeasuredWidth);
	}

#ifdef TRACE
	for(int i = 0; i < contours.size() ; i++){
		Rect testRect;
		int	usable=-1;

		if (UsableContour[i])
		{
			usable = 1;
		}
		else
		{
			usable = 0;
		}

		testRect 	= boundingRect(contours.at(i));
		//if (UsableContour[i] == true) 
		{
			printf("BSE similarity[i=%2d]=%6.3f Usable=%d x=%3d, w=%3d, y=%3d h=%3d a=%d\n", 
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

			// This test looks for the 2 rectangles that are the most similar in terms of size.
			testMinDiff = abs(testRect_i.y - testRect_j.y) +
				          abs(testRect_i.height - testRect_j.height) +
				          //abs(testRect_i.x - testRect_j.x) +
				          abs(testRect_i.width - testRect_j.width) +
						  //abs(testRect_i.area() - testRect_j.area());
						  abs(testRect_i.area() - testRect_i.area());
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

static bool ContourLocatorFarMode(vector < vector<Point> > &contours, int &closest, int &secClose){
	
	vector <double> similarity(contours.size());
	vector <bool> UsableContour(contours.size());
	int ImageCenterLine = IMAGE_CENTERLINE_P + offset;

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
		int rectangleCenterLine = currentSize.x + currentSize.width/2;

		double MeasuredHeight 	= currentSize.height;
		double MeasuredWidth 	= currentSize.width;

		if (((MeasuredHeight < 25) || (MeasuredWidth < 10)) /*|| 
			(MeasuredHeight > 500) || (MeasuredWidth > 700) */) {
			similarity[i] = 50.;
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
			//UsableContour[i] = true;
			//if (similarity[i] > fabs(1.0- STRIP_WIDTH/STRIP_HEIGHT)) {
			if (similarity[i] < 1.) {
				UsableContour[i] = true;
			}
			else {
				UsableContour[i] = false;
			}
		}
        
		//if (similarity[i] < 10.) {
		// 	// Display the "Potentially good" rectangles
		//	rectangle(img, Point(currentSize.x, currentSize.y), Point(currentSize.x + currentSize.width, currentSize.y + currentSize.height), Scalar(0, 255, 0), 2);
		//}

		//printf("i=%d similarity=%f Height=%d Width=%d\n" ,i, similarity[i], (int) MeasuredHeight, (int) MeasuredWidth);
	}

#ifdef TRACE
	for(int i = 0; i < contours.size() ; i++){
		Rect testRect;
		int	usable=-1;

		if (UsableContour[i])
		{
			usable = 1;
		}
		else
		{
			usable = 0;
		}

		testRect 	= boundingRect(contours.at(i));
		//if (UsableContour[i] == true) 
		{
			printf("BSF similarity[i=%2d]=%6.3f Usable=%d x=%3d, w=%3d, y=%3d h=%3d a=%d\n", 
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
			//AreaRatio	= testRect_i.area()/testRect_i.area();

			// This test looks for the 2 rectangles that are the most similar in terms of size.
			testMinDiff = abs(testRect_i.y - testRect_j.y) +
				          abs(testRect_i.height - testRect_j.height) +
				          //abs(testRect_i.x - testRect_j.x) +
				          abs(testRect_i.width - testRect_j.width) 
						  //+ abs(testRect_i.area() - testRect_i.area())
							;
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

static bool ExtremeCloseupModeGet(vector < vector<Point> > &contours)
{
	bool 	InExtremCloseupMode = false;
	int		Count = 0;

	for (int i = 0; i < contours.size(); i++) {
		Rect currentSize = boundingRect(contours[i]);

		if (currentSize.area() > 17000) {
			Count++;
		}
	}

	if (Count >= 1) {
		InExtremCloseupMode = true;
	}

	return InExtremCloseupMode;
}

static void ThresholdExtremeCloseupSet(void)
{
	m_H_MIN = m_H_MIN_close;
	m_H_MAX = m_H_MAX_close;
	m_S_MIN = m_S_MIN_close;
	m_S_MAX = m_S_MAX_close;
	m_V_MIN = m_V_MIN_close;
	m_V_MAX = m_V_MAX_close;
}

static void ThresholdFarSet(void)
{
	m_H_MIN = m_H_MIN_far;
	m_H_MAX = m_H_MAX_far;
	m_S_MIN = m_S_MIN_far;
	m_S_MAX = m_S_MAX_far;
	m_V_MIN = m_V_MIN_far;
	m_V_MAX = m_V_MAX_far;
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

static void ImageRecordBegin(void)
{
	if (ImageRecording == false) {
		char filename[256];

		ImageNum = 0;

		sprintf(filename, filename_log, SetNum);

		fp_log = fopen(filename, "w");
		if (fp_log == NULL) {
			printf("ERROR: Unable to open file: %s\n", filename);
		}

		ImageRecording = true;
	}
}

static void ImageRecordEnd(void)
{
	if (ImageRecording == true) {
		ImageRecording = false;

		SetNum++;
		if (fp_log != NULL) {
			fclose (fp_log);
			fp_log = NULL;
		}
	}
}

