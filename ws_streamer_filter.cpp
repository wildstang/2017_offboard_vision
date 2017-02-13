/**

  Filter for OpenCV processing as a plugin for mjpg-streamer.
*/

#include <iostream>
#include <stdio.h>
#include <time.h>
#include <stdlib.h>
#include <opencv2/opencv.hpp>

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>

void error(char *msg)
{
    perror(msg);
    exit(0);
}

using namespace cv;
using namespace std;

#define WS_USE_SOCKETS

int m_H_MIN = 50;
int m_S_MIN = 30;
int m_V_MIN = 0;

int m_H_MAX = 120;
int m_S_MAX = 255;
int m_V_MAX = 255;

int sockfd;


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
	// Set client
	// Set IP address

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
    server = gethostbyname("10.1.11.6");
    if (server == NULL) {
        cout << "ERROR, no such host" << endl;
        exit(0);
    }

    bzero((char *) &serv_addr, sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    bcopy((char *)server->h_addr, (char *)&serv_addr.sin_addr.s_addr, server->h_length);
    serv_addr.sin_port = htons(portno);

    cout << "Attempting to connect" << endl;
    if (connect(sockfd,(struct sockaddr *)&serv_addr,sizeof(serv_addr)) < 0)
    {
    	cout << "Error connecting" << endl;
    	error("ERROR connecting");
    }

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

		token = strtok(NULL, "|\n");
		count++;
	}


	printf("Hmin = %d\nHmax = %d\nSmin = %d\nSmax = %d\nVmin = %d\nVmax = %d\n", m_H_MIN, m_H_MAX, m_S_MIN, m_S_MAX, m_V_MIN, m_V_MAX);
#endif
	return true;


}

int m_goalX;
int m_goalY;
int m_deltaX = 5;
int m_deltaY = 5;

int m_currentX;
int m_currentY;



void ws_process(Mat &imgbgr) {

	// Make an empty matrix with our image for hsv
	Mat hsvMat = Mat::zeros(imgbgr.size(), imgbgr.type());//(imgbgr.size(), imgbgr.type());

	cvtColor(imgbgr, hsvMat, COLOR_BGR2HSV);

	double find_rectangles_time ;
	double draw_largest_time ;

	Mat threshMat; // = new Mat(); // Will be the image matrix for the binary
	// values (black and white image)

	Mat lower;
	Mat upper;

	m_goalX = imgbgr.cols / 2;
	m_goalY = imgbgr.rows / 2;

	inRange(hsvMat, Scalar(m_H_MIN, m_S_MIN, m_V_MIN), Scalar(m_H_MAX, m_S_MAX, m_V_MAX), threshMat);

	// Mat threshold_output;
	vector < Vec4i > hierarchy;
	vector < vector<Point> > contours;

	findContours(threshMat, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

	int targetBottom = 0;
	int targetCenter = 0;

      if (contours.size() > 0)
      {
         Rect target = boundingRect(contours[0]);
         double c_width = target.width; // get width of the contour
         double c_height = target.height; // get width of the contour
         int biggestContourIndex = 0;

         for (int i = 1; i < contours.size() - 1; i++)
         {
            // Iterate through all the contours
            Rect newRect = boundingRect(contours[i]);

            double new_c_width = newRect.width;
            double new_c_height = newRect.height;

            if (new_c_width > c_width && new_c_height > c_height)
            {
               // find the contour with the biggest width (Probably the target)
               target = newRect;
               biggestContourIndex = i;
               c_width = new_c_width;
               c_height = new_c_height;
            }
         }

      	targetBottom = target.y;
        targetCenter = target.x + (target.width / 2);

        Scalar crosshairColor = Scalar(0, 0, 0);
        // Draw the goal crosshairs
        line(imgbgr, Point(m_goalX, 0), Point(m_goalX, imgbgr.rows), crosshairColor, 2);
        line(imgbgr, Point(0, m_goalY), Point(imgbgr.cols, m_goalY), crosshairColor, 2);

        //  Draw current target object center
        m_currentX = target.x + (target.width / 2);
        m_currentY = target.y + (target.height / 2);
        bool ontarget = false;

        Scalar targetColor = Scalar(0, 0, 255);

        if ((abs(m_currentX - m_goalX) <= m_deltaX) && (abs(m_currentY - m_goalY) <= m_deltaY))
        {
           targetColor = Scalar(0, 255, 0);
           ontarget = true;
        }
        else if ((abs(m_currentX - m_goalX) <= (3 * m_deltaX)) && (abs(m_currentY - m_goalY) <= (3 * m_deltaY)))
        {
           targetColor = Scalar(0, 100, 255);
        }

        line(imgbgr, Point(m_currentX, 0), Point(m_currentX, imgbgr.rows), targetColor, 2);
        line(imgbgr, Point(0, m_currentY), Point(imgbgr.cols, m_currentY), targetColor, 2);

        if (ontarget)
        {
           circle(imgbgr, Point(m_currentX, m_currentY), 5, Scalar(0, 255, 0), 2);
           circle(imgbgr, Point(m_currentX, m_currentY), 15, Scalar(0, 255, 0), 2);
           circle(imgbgr, Point(m_currentX, m_currentY), 25, Scalar(0, 255, 0), 2);
        }

		// Just Draw Blue for now
         rectangle(imgbgr, Point(target.x, target.y), Point(target.x + target.width, target.y + target.height), Scalar(0, 255, 0), 2);
      }

         hsvMat.release();
         threshMat.release();

         Mat tmp;
         cv::resize(imgbgr, tmp, Size(320, 240));

         imgbgr = tmp;

#ifdef WS_USE_SOCKETS
         int n;
         char buffer[256];
         bzero(buffer,256);
         sprintf(buffer, "%d\n", m_currentX);
         n = send(sockfd,buffer,strlen(buffer), MSG_DONTWAIT);
         if (n < 0)
         {
        	 error("ERROR writing to socket");
         }
#endif

	return;
}

/**
    Called by the OpenCV plugin upon each frame
*/
void filter_process(void* filter_ctx, Mat &src, Mat &dst) {
    // TODO insert your filter code here
	clock_t startTime, endTime;

	startTime = clock();

	ws_process(src);

	endTime = clock();
	double time_spent = (double) (endTime - startTime) / CLOCKS_PER_SEC;

	fprintf(stdout, "t=%f\n", time_spent);
    dst = src;
}

/**
    Called when the input plugin is cleaning up
*/
void filter_free(void* filter_ctx) {
    // empty
}


