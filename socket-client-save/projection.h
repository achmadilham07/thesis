#pragma once
#include <iostream>
#include <string>
#include <vector>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

class projection
{
	double h;	// dianggap 2x dari bounding box
	double fc;			// in pixel
	double yc;			// in centimeter
	double uc;
	double v0, ud, hd, vd;
	double x, y, z;

	//processing time check
	unsigned long AAtime = 0, BBtime = 0;
	double s_time, fps_time;

public:
	void compute_projection(Rect locations, bool report);
	void set_loc_y(Rect loc);
	void set_loc_w(Rect loc);
	void set_loc_h(Rect loc);

	void set_X();
	void set_Y();
	void set_Z();
	void reportAll();

	void drawVP(cv::Mat &imgfile, double X, double Y);
	void set_h(double times_of_height);
	void set_fc(double focal_length);
	void set_yc(double camera_height);
	void set_uc(double horizon_central_point);
	void set_v0(double horizon_position);

	double get_coord_x();
	double get_coord_y();
	double get_coord_z();

	double get_s_time();
	double get_fps_time();
};

void projection::compute_projection(Rect locations, bool report)
{
	//to calculate fps
	AAtime = getTickCount();

	set_loc_w(locations);
	set_loc_h(locations);
	set_loc_y(locations);

	set_X();
	set_Y();
	set_Z();

	//to calculate fps
	BBtime = getTickCount();
	s_time = (BBtime - AAtime) / getTickFrequency();
	fps_time = 1 / s_time;

	if (report)
		reportAll();
}

void projection::set_loc_y(Rect loc) {
	vd = loc.y + hd;
}

void projection::set_loc_w(Rect loc) {
	ud = loc.x + 0.5 * loc.width;
}

void projection::set_loc_h(Rect loc) {
	hd = h * loc.height;
}

void projection::set_X() {
	x = (yc * (ud - uc)) / (vd - v0);
}

void projection::set_Y() {
	y = (fc * yc) / (vd - v0);
}

void projection::set_Z() {
	z = (hd * yc) / (vd - v0);
}

void projection::reportAll() {
	cout << "\tx: " << x;
	cout << "\ty: " << y;
	cout << "\tz: " << z << endl;
}

void projection::drawVP(cv::Mat &imgfile, double X, double Y) {
	cv::line(imgfile, cv::Point(0, Y), cv::Point(imgfile.cols, Y), cv::Scalar(100, 100, 255), 5);
	cv::line(imgfile, cv::Point(X, 0), cv::Point(X, imgfile.rows), cv::Scalar(100, 100, 255), 5);
}

void projection::set_h(double times_of_height) {
	h = times_of_height;
}


void projection::set_fc(double focal_length) {
	fc = focal_length;
}

void projection::set_yc(double camera_height) {
	yc = camera_height;
}

void projection::set_uc(double horizon_central_point) {
	uc = horizon_central_point;
}

void projection::set_v0(double horizon_position) {
	v0 = horizon_position;
}

double projection::get_coord_x() {
	return x / 1000;
}

double projection::get_coord_y() {
	return y / 1000;
}

double projection::get_coord_z() {
	return z / 1000;
}

double projection::get_s_time() {
	return s_time * 1000;
}

double projection::get_fps_time() {
	return fps_time;
}