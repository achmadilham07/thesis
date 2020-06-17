#pragma once
#include "opencv2/objdetect.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include <conio.h>

using namespace cv;
using namespace std;

class haar
{
	CascadeClassifier upperbody;
	Mat img;
	vector<Rect> upper;
	Mat frame_gray;

	//processing time check
	unsigned long AAtime = 0, BBtime = 0;
	double s_time, fps_time;

public:

	haar(String trained_svm);
	void computeHAAR();
	void set_img(Mat image);
	void draw_locations(cv::Mat& img, const Scalar& color);
	vector< Rect > get_locations();
	double get_s_time();
	double get_fps_time();

};

haar::haar(String trained_svm)
{
	if (!upperbody.load(trained_svm)) {
		printf("--(!)Error loading face cascade\n");
		_getch();
		exit(1);
	};
}

void haar::computeHAAR()
{
	//to calculate fps
	AAtime = getTickCount();

	cvtColor(img, frame_gray, COLOR_BGR2GRAY);
	equalizeHist(frame_gray, frame_gray);
	//-- Detect upper
	upperbody.detectMultiScale(frame_gray, upper);// , 1.1, 2, 0 | CASCADE_SCALE_IMAGE);

	//to calculate fps
	BBtime = getTickCount();
	s_time = (BBtime - AAtime) / getTickFrequency();
	fps_time = 1 / s_time;
}

void haar::draw_locations(cv::Mat& img, const Scalar& color)
{
	if (!upper.empty())
	{
		for (size_t j = 0; j < upper.size(); j++)
		{
			rectangle(img, upper[j], color, 2);
		}
	}
}

vector< Rect > haar::get_locations() {
	return upper;
}

void haar::set_img(cv::Mat image) {
	img = image;
}

double haar::get_s_time() {
	return s_time * 1000;
}

double haar::get_fps_time() {
	return fps_time;
}