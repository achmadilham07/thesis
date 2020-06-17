#pragma once
#include <iostream>
#include <string>
#include "opencv2/core/core.hpp"
#include "opencv2/core/utility.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <vector>
#include <fstream>
#include <ctime>
#include <conio.h>

#include <opencv2/opencv.hpp>
#include <windows.h>

using namespace cv;
using namespace std;
using namespace cv::ml;

#define WINDOW_NAME "WINDOW"
#define TRAFFIC_VIDEO_FILE "E:/dataset/768x576_2.avi"
#define	IMAGE_SIZE Size(72,96) 

#ifndef __linux
#include <io.h> 
#define access _access_s
#else
#include <unistd.h>
#include <memory>
#endif

class hog
{
	cv::Mat img, draw;
	Ptr<SVM> svm;
	HOGDescriptor hogdesc;
	vector< Rect > locations;
	vector< double > hog_detector;
	double threshold;
	double resized;
	vector< double > foundWeights;

	//processing time check
	unsigned long AAtime = 0, BBtime = 0;
	double s_time = 0, fps_time;

public:

	hog(cv::String trained_svm);

	void get_svm_detector(const Ptr<SVM>& svm, vector< double > & hog_detector);

	void draw_locations(cv::Mat & img, const vector< Rect > & locations, const Scalar & color);

	bool file_exists(const string &file);

	void resizeloc(vector< Rect > & locations);

	void computeHOG();

	vector< Rect > get_locations();

	double get_s_time();
	double get_fps_time();
	void set_threshold(double thrshld);
	void set_resized(double rsize);
	void set_img(cv::Mat img);
};

hog::hog(cv::String trained_svm)
{
	hogdesc.winSize = IMAGE_SIZE;
	if (file_exists(trained_svm)) {
		int found = trained_svm.find("hog");
		if (found != string::npos) {
			hogdesc.load(trained_svm);
		}
		else {
			svm = StatModel::load<SVM>(trained_svm);
			get_svm_detector(svm, hog_detector);
			hogdesc.setSVMDetector(hog_detector);
		}
	}
	else {
		printf("No file traied_svm");
		_getch();
		exit(1);
	}
}

void hog::get_svm_detector(const Ptr<SVM>& svm, vector<double>& hog_detector)
{
	// get the support vectors
	cv::Mat sv = svm->getSupportVectors();
	const int sv_total = sv.rows;
	// get the decision function
	cv::Mat alpha, svidx;
	double rho = svm->getDecisionFunction(0, alpha, svidx);

	CV_Assert(alpha.total() == 1 && svidx.total() == 1 && sv_total == 1);
	CV_Assert((alpha.type() == CV_64F && alpha.at<double>(0) == 1.) ||
		(alpha.type() == CV_32F && alpha.at<double>(0) == 1.f));
	CV_Assert(sv.type() == CV_32F);
	hog_detector.clear();

	hog_detector.resize(sv.cols + 1);
	memcpy(&hog_detector[0], sv.ptr(), sv.cols * sizeof(hog_detector[0]));
	hog_detector[sv.cols] = (double)-rho;
}

void hog::draw_locations(cv::Mat & img, const vector<Rect>& locations, const Scalar & color)
{
	if (!locations.empty())
	{
		for (size_t j = 0; j < locations.size(); j++)
		{
			//char textOnImg[15];
			//sprintf_s(textOnImg, "%d. ", j);
			//sprintf_s(textOnImg, "%d. %.2f ", j, foundWeights[j]);
			rectangle(img, locations[j], color, 2);
			//putText(img,
			//	textOnImg,
			//	Point(locations[j].x + 2, locations[j].y + 11), // Coordinates
			//	FONT_HERSHEY_PLAIN, // Font
			//	1.1, // Scale. 2.0 = 2x bigger
			//	Scalar(76, 0, 153), // BGR Color
			//	2, // Line Thickness (Optional)
			//	CV_AA); // Anti-alias (Optional)
		}
	}
}

bool hog::file_exists(const string & file)
{
	return access(file.c_str(), 0) == 0;
}

void hog::resizeloc(vector<Rect> & locations)
{
	vector< Rect > relocations = locations;
	locations.clear();
	if (!relocations.empty())
	{
		for (size_t i = 0; i < relocations.size(); ++i)
		{
			relocations[i].x = relocations[i].x / resized;
			relocations[i].y = relocations[i].y / resized;
			relocations[i].width = relocations[i].width / resized;
			relocations[i].height = relocations[i].height / resized;
			//cout << relocations[i] << endl;

			locations.push_back(relocations[i]);
		}
	}

}

void hog::computeHOG() {
	draw = img.clone();

	//to calculate fps
	AAtime = getTickCount();

	//convert color to grey
	cv::cvtColor(img, img, cv::COLOR_BGR2GRAY);
	equalizeHist(img, img);

	//resize frame
	cv::resize(img, img, Size(img.cols * resized, img.rows * resized));

	//detect an object using HOG Descriptor
	locations.clear();
	hogdesc.detectMultiScale(img, locations, foundWeights, threshold);

	//resize back the location of object
	resizeloc(locations);

	//to calculate fps
	BBtime = getTickCount();
	s_time = (BBtime - AAtime) / getTickFrequency();
	fps_time = 1 / s_time;
}

vector< Rect > hog::get_locations() {
	return locations;
}

double hog::get_s_time() {
	return s_time * 1000;
}

double hog::get_fps_time() {
	return fps_time;
}

void hog::set_threshold(double thrshld) {
	threshold = thrshld;
}

void hog::set_resized(double rsize) {
	resized = rsize;
}

void hog::set_img(cv::Mat image) {
	img = image;
}
