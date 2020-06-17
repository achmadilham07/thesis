#pragma once
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/tracking.hpp>
#include <iostream>
#include <vector>

using namespace std;
using namespace cv;

class kalman
{
private:
	unsigned int type = CV_32F;
	int statePos1Size = 6;
	int measPos1Size = 4;
	int contrPos1Size = 0;
	int statePos2Size = 4;
	int measPos2Size = 2;
	Mat statePos1, measPos1, statePos2, measPos2;
	KalmanFilter kfPos1, kfPos2;
	Mat estimated;

public:
	bool found1 = false, found2 = false;
	void kalmanPos1_init();
	void kalmanPos2_init();
	void setA1(double dT);
	void setA2(double dT);
	void predictPos1();
	void predictPos2();
	Rect getRect();
	Point getPoint();
	void updatePos1(Rect bbox);
	void updatePos2(double x, double y);
	void drawRect(Mat &img, Rect bbox, int i);
	double getPos2x();
	double getPos2y();
};

void kalman::kalmanPos1_init() {
	kfPos1.init(statePos1Size, measPos1Size, contrPos1Size, type);
	statePos1.create(statePos1Size, 1, type);	// [ x, y, vx, vy, w, h ]
	measPos1.create(measPos1Size, 1, type);		// [ zx, zy, zw, zh ]

	// Transition State Matrix A
	// [ 1  0  dT   0  0  0 ]
	// [ 0  1   0  dT  0  0 ]
	// [ 0  0   1   0  0  0 ]
	// [ 0  0   0   1  0  0 ]
	// [ 0  0   0   0  1  0 ]
	// [ 0  0   0   0  0  1 ]
	setIdentity(kfPos1.transitionMatrix);

	// Measure Matrix H
	// [ 1 0 0 0 0 0 ]
	// [ 0 1 0 0 0 0 ]
	// [ 0 0 0 0 1 0 ]
	// [ 0 0 0 0 0 1 ]
	kfPos1.measurementMatrix = Mat::zeros(measPos1Size, statePos1Size, type);
	kfPos1.measurementMatrix.at<float>(0) = 1.0f;
	kfPos1.measurementMatrix.at<float>(7) = 1.0f;
	kfPos1.measurementMatrix.at<float>(16) = 1.0f;
	kfPos1.measurementMatrix.at<float>(23) = 1.0f;

	// Process Noise Covariance Matrix Q
	// [ Ex   0     0    0   0   0 ]
	// [ 0   Ey     0    0   0   0 ]
	// [ 0    0   Evx    0   0   0 ]
	// [ 0    0     0  Evy   0   0 ]
	// [ 0    0     0    0  Ew   0 ]
	// [ 0    0     0    0   0  Eh ]
	/*kfPos1.processNoiseCov.at<float>(0) = 1e-2;
	kfPos1.processNoiseCov.at<float>(7) = 1e-2;
	kfPos1.processNoiseCov.at<float>(14) = 5.0f;
	kfPos1.processNoiseCov.at<float>(21) = 5.0f;
	kfPos1.processNoiseCov.at<float>(28) = 1e-2;
	kfPos1.processNoiseCov.at<float>(35) = 1e-2;*/
	kfPos1.processNoiseCov.at<float>(0) = 1e-3;//4
	kfPos1.processNoiseCov.at<float>(7) = 1e-3;
	kfPos1.processNoiseCov.at<float>(14) = 1e-4;//6
	kfPos1.processNoiseCov.at<float>(21) = 1e-4;
	kfPos1.processNoiseCov.at<float>(28) = 1e-2;//3
	kfPos1.processNoiseCov.at<float>(35) = 1e-2;

	// Measure Noise Covariance Matrix R
	setIdentity(kfPos1.measurementNoiseCov, Scalar(1e-1));
}

void kalman::setA1(double dT) {
	kfPos1.transitionMatrix.at<float>(2) = dT;
	kfPos1.transitionMatrix.at<float>(9) = dT;
}

// if bounding box exist
void kalman::predictPos1() {
	statePos1 = kfPos1.predict();

}

Rect kalman::getRect() {
	Rect predRect;
	predRect.width = statePos1.at<float>(4);
	predRect.height = statePos1.at<float>(5);
	predRect.x = statePos1.at<float>(0) - predRect.width / 2;
	predRect.y = statePos1.at<float>(1) - predRect.height / 2;

	return predRect;
}

Point kalman::getPoint() {
	Point center;
	center.x = statePos1.at<float>(0);
	center.y = statePos1.at<float>(1);

	return center;
}

void kalman::updatePos1(Rect bbox) {
	measPos1.at<float>(0) = bbox.x + bbox.width / 2;
	measPos1.at<float>(1) = bbox.y + bbox.height / 2;
	measPos1.at<float>(2) = (float)bbox.width;
	measPos1.at<float>(3) = (float)bbox.height;

	if (!found1) { // First detection !!
		kfPos1.errorCovPre.at<float>(0) = 1;
		kfPos1.errorCovPre.at<float>(7) = 1;
		kfPos1.errorCovPre.at<float>(14) = 1;
		kfPos1.errorCovPre.at<float>(21) = 1;
		kfPos1.errorCovPre.at<float>(28) = 1;
		kfPos1.errorCovPre.at<float>(35) = 1;

		statePos1.at<float>(0) = measPos1.at<float>(0);
		statePos1.at<float>(1) = measPos1.at<float>(1);
		statePos1.at<float>(2) = 0;
		statePos1.at<float>(3) = 0;
		statePos1.at<float>(4) = measPos1.at<float>(2);
		statePos1.at<float>(5) = measPos1.at<float>(3);

		kfPos1.statePost = statePos1;

		found1 = true;
	}
	else {
		kfPos1.correct(measPos1);
	}
}

void kalman::drawRect(Mat &img, Rect bbox, int i = 0) {
	switch (i) {
	case 0:
		rectangle(img, bbox, CV_RGB(255, 105, 180), 2);
		break;
	case 1:
		rectangle(img, bbox, CV_RGB(30, 144, 255), 2);
		break;
	default:
		rectangle(img, bbox, CV_RGB(255, 105, 180), 2);
		break;
	}

}

void kalman::kalmanPos2_init() {
	kfPos2.init(statePos2Size, measPos2Size, contrPos1Size, type);
	statePos2.create(statePos2Size, 1, type);	// [ x, y, vx, vy ]
	measPos2.create(measPos2Size, 1, type);		// [ zx, zy ]

	/*
	// Transition State Matrix A
	// [ 1  0  dT   0 ]
	// [ 0  1   0  dT ]
	// [ 0  0   1   0 ]
	// [ 0  0   0   1 ]
	setIdentity(kfPos2.transitionMatrix);

	// Measure Matrix H
	// [ 1 0 0 0 ]
	// [ 0 1 0 0 ]
	kfPos2.measurementMatrix = Mat::ones(measPos2Size, statePos2Size, type);
	kfPos2.measurementMatrix.at<float>(0) = 1.0f;
	kfPos2.measurementMatrix.at<float>(5) = 1.0f;

	// Process Noise Covariance Matrix Q
	// [ Ex   0     0    0 ]
	// [ 0   Ey     0    0 ]
	// [ 0    0   Evx    0 ]
	// [ 0    0     0  Evy ]
	kfPos2.processNoiseCov.at<float>(0) = 1e-2;
	kfPos2.processNoiseCov.at<float>(5) = 1e-2;
	kfPos2.processNoiseCov.at<float>(10) = 1e-2;
	kfPos2.processNoiseCov.at<float>(15) = 1e-2;
	kfPos2.processNoiseCov.at<float>(0) = 1e-4;
	kfPos2.processNoiseCov.at<float>(5) = 1e-4;
	kfPos2.processNoiseCov.at<float>(10) = 1e-6;
	kfPos2.processNoiseCov.at<float>(15) = 1e-6;

	// Measure Noise Covariance Matrix R
	setIdentity(kfPos2.measurementNoiseCov, Scalar(1e-1));

	//KalmanFilter KF(4, 2, 0);
	//kfPos2.init(statePos2Size, measPos2Size, contrPos1Size, type);
	*/

	kfPos2.transitionMatrix = (Mat_<float>(4, 4) << 1, 0, 1, 0, 0, 1, 0, 1, 0, 0, 1, 0, 0, 0, 0, 1);
	measPos2.setTo(Scalar(0));

	// init...
	kfPos2.statePre.at<float>(0) = 0;
	kfPos2.statePre.at<float>(1) = 0;
	kfPos2.statePre.at<float>(2) = 0;
	kfPos2.statePre.at<float>(3) = 0;

	// process noise covariance matrix (Q)
	kfPos2.processNoiseCov.at<float>(0) = 1e-1;//-3
	kfPos2.processNoiseCov.at<float>(5) = 1e-1;
	kfPos2.processNoiseCov.at<float>(10) = 1e-8;//-6
	kfPos2.processNoiseCov.at<float>(15) = 1e-8;

	setIdentity(kfPos2.measurementMatrix);

	// measurement noise covariance matrix (R)
	setIdentity(kfPos1.measurementNoiseCov, Scalar(1e10)); //-1

	setIdentity(kfPos2.errorCovPost, Scalar::all(.1));

}

void kalman::setA2(double dT) {
	kfPos2.transitionMatrix.at<float>(2) = dT;
	kfPos2.transitionMatrix.at<float>(7) = dT;
}

void kalman::predictPos2() {
	statePos2 = kfPos2.predict();
}

void kalman::updatePos2(double x, double y) {
	measPos2.at<float>(0) = x;
	measPos2.at<float>(1) = y;

	if (!found2) {
		kfPos2.errorCovPre.at<float>(0) = 1;
		kfPos2.errorCovPre.at<float>(5) = 1;
		kfPos2.errorCovPre.at<float>(10) = 1;
		kfPos2.errorCovPre.at<float>(15) = 1;

		statePos2.at<float>(0) = measPos2.at<float>(0);
		statePos2.at<float>(1) = measPos2.at<float>(1);
		statePos2.at<float>(2) = 0;
		statePos2.at<float>(3) = 0;

		kfPos2.statePost = statePos2;

		found2 = true;
	}
	else {
		kfPos2.correct(measPos2);
	}
}

double kalman::getPos2x() {
	return (double)statePos2.at<float>(0);
}

double kalman::getPos2y() {
	return (double)statePos2.at<float>(1);
}