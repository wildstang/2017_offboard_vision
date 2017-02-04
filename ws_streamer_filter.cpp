/**

  Filter for OpenCV processing as a plugin for mjpg-streamer.
*/

#include <iostream>
#include <stdio.h>
#include <time.h>

#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

#ifdef USE_NT_CORE
#include "ntcore.h"
#include "tables/ITableListener.h"
#include "networktables/NetworkTable.h"
#endif


RNG rng(12345);

#ifdef USE_NT_CORE
std::shared_ptr<NetworkTable> table;
#endif

static bool NODASHBOARD = false;

int m_H_MIN = 80;
int m_S_MIN = 45;
int m_V_MIN = 0;

int m_H_MAX = 120;
int m_S_MAX = 255;
int m_V_MAX = 255;



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
	// Do network tables things
	// Set client
	// Set IP address

#ifdef USE_NT_CORE
	NetworkTable::SetClientMode();
	NetworkTable::SetTeam(111);
	NetworkTable::SetIPAddress("10.1.11.2");
	table = NetworkTable::GetTable("remoteIO");

	if (!NODASHBOARD)
	{
		m_H_MIN = (int) table->GetNumber("Minimum Hue", m_H_MIN);
		m_H_MAX = (int) table->GetNumber("Maximum Hue", m_H_MAX);
		m_S_MIN = (int) table->GetNumber("Minimum Saturation", m_S_MIN);
		m_S_MAX = (int) table->GetNumber("Maximum Saturation", m_S_MAX);
		m_V_MIN = (int) table->GetNumber("Minimum Value", m_V_MIN);
		m_V_MAX = (int) table->GetNumber("Maximum Value", m_V_MAX);
	}
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

//		if (countP % (fps / 2) == 0)
//		{
//			cout << "---------------------------------------------------------" << endl;
			// Write image to file on disk
//			char fn[100];
//			snprintf(fn, sizeof fn, "images/image%02d.jpg", countP);
//			imwrite(fn, imgbgr);
//		}

         hsvMat.release();
         threshMat.release();

         Mat tmp;
         cv::resize(imgbgr, tmp, Size(320, 240));

         imgbgr = tmp;
#ifdef USE_NT_CORE
	table->PutNumber("Target Bottom", targetBottom);
	table->PutNumber("Target Center", targetCenter);
//      ((RemoteAnalogOutput) Core.getOutputManager().getOutput(BBBOutputs.TARGET_BOTTOM.getName())).setValue(targetBottom);
//      ((RemoteAnalogOutput) Core.getOutputManager().getOutput(BBBOutputs.TARGET_CENTER.getName())).setValue(targetCenter);
//      ((RemoteAnalogOutput) Core.getOutputManager().getOutput(BBBOutputs.VISION_ANGLE.getName())).setValue(currentAverageAngle);
//      ((RemoteAnalogOutput) Core.getOutputManager().getOutput(BBBOutputs.VISION_DISTANCE.getName())).setValue(currentAverageDistance);
//   }
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

	fprintf(stdout, "%f\n", time_spent);
//    dst = src;
}

/**
    Called when the input plugin is cleaning up
*/
void filter_free(void* filter_ctx) {
    // empty
}


