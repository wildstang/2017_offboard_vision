#include <cv.h>
#include <cxcore.h>
#include <highgui.h>
#include <opencv2/opencv.hpp>
#include <stdio.h>
#include <iostream>
#include <math.h>

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
static int NUM_AVERAGES = 5;
static int BRIGHTNESS = 30;
static int CONTRAST = 5;
static int SATURATION = 200;

int m_H_MIN = 80;
int m_S_MIN = 45;
int m_V_MIN = 0;

int m_H_MAX = 120;
int m_S_MAX = 255;
int m_V_MAX = 255;


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
		m_H_MIN = (int) table.getNumber("Minimum Hue", H_MIN);
		m_H_MAX = (int) table.getNumber("Maximum Hue", H_MAX);
		m_S_MIN = (int) table.getNumber("Minimum Saturation", S_MIN);
		m_S_MAX = (int) table.getNumber("Maximum Saturation", S_MAX);
		m_V_MIN = (int) table.getNumber("Minimum Value", V_MIN);
		m_V_MAX = (int) table.getNumber("Maximum Value", V_MAX);
	}
#endif

	return true;
}


bool wayToSort(int i, int j) { return i > j; }

void ws_process(Mat& img) {
	Mat hsvMat(img.size(), img.type());
	Mat hsvOut;
	cvtColor(img, hsvMat, COLOR_BGR2HSV);
	inRange(hsvMat, Scalar(0, 0, 245), Scalar(0, 0, 255), hsvOut);
	
	Mat blurMat;
	GaussianBlur(hsvOut, blurMat, Size(0, 0), 3.0);
	
	vector < vector<Point> > contours;
	vector < Vec4i > hierarchy;
	findContours(blurMat, contours, hierarchy, RETR_TREE,
			CHAIN_APPROX_SIMPLE);
	vector <double> similarity(contours.size());
	int closest = 0;

	int secClose = 0;
	for (int i = 0; i < contours.size(); i++) {
		Rect currentSize = boundingRect(contours[i]);
		similarity[i] = fabs(1-((currentSize.height / currentSize.width)* 0.4));
	}
	if(contours.size() > 0){
		for(int i = 1; i < contours.size(); i++){
			if(similarity[i] < similarity[i-1]){
				closest = i;
			}
		}
	}
	if(contours.size() > 1){
		for(int i = 1; i < contours.size(); i++){
			if((similarity[i] < similarity[i-1]) && i != closest){
				secClose = i;
			}
		}
	}
	Rect oneRect;
	Rect theOtherRect;
	if(contours.size() > 0){
		oneRect = boundingRect(contours.at(closest));
		rectangle(img, Point(oneRect.x, oneRect.y),
			Point(oneRect.x + oneRect.width,
					oneRect.y + oneRect.height), Scalar(0, 255, 0),
			2);
	}
	if(contours.size() > 1){
		theOtherRect = boundingRect(contours.at(secClose));
		rectangle(img, Point(theOtherRect.x, theOtherRect.y),
			Point(theOtherRect.x + theOtherRect.width,
					theOtherRect.y + theOtherRect.height), Scalar(0, 255, 0),
			2);
	}
	if(oneRect.)
	
	//Mat tmp;
	//cv::resize(img, tmp, Size(320, 240));
	//img = tmp;
}

void filter_process(void* filter_ctx, Mat &src, Mat &dst) {
	ws_process(src);
	
	dst = src;
}


void filter_free(void* filter_ctx) {

}