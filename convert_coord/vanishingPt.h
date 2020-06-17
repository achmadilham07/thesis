#pragma once
#include <iostream>
#include <armadillo>
#include <string>
#include <opencv2/opencv.hpp>
#include <vector>

using namespace std;
using namespace arma;

class vanishingPt
{
	cv::Mat image, img, gray;
	cv::Mat frame, frame_ori;
	cv::Mat drawnLines;
	vector< vector<int> > points;
	mat A, b, prevRes;
	mat Atemp, btemp, res, aug, error, soln;
	double s_time, fps_time;
	unsigned long AAtime = 0, BBtime = 0;

	//ofstream out1, out2;
	float epsilon;

	//store slope (m) and y-intercept (c) of each lines
	float m, c;

	//store minimum length for lines to be considered while estimating vanishing point
	int minlength;

	//temporary vector for intermediate storage
	vector<int> temp;

	//store (x1, y1) and (x2, y2) endpoints for each line segment
	vector<cv::Vec4i> lines_std;

	//to store intermediate errors
	double temperr;

public:
	//constructor to set video/webcam and find vanishing point
	void vanishingPt_(cv::Mat imgfile, bool);

	void init(cv::Mat image, mat prevRes);

	void makeLines(int flag);

	//estimate the vanishing point
	void eval();

	//function to calculate and return the intersection point
	mat calc(mat A, mat b);

	float get_solnX();

	float get_solnY();

	cv::Mat get_frame();

	void drawVP(cv::Mat& imgfile);

	double get_s_time();

	double get_fps_time();

	void saveImg(cv::String& imgfile);
};

void vanishingPt::vanishingPt_(cv::Mat imgfile, bool showImages)
{
	// to calculate fps
	AAtime = cv::getTickCount();

	int flag = 0;

	frame_ori = imgfile.clone();
	frame = imgfile.clone();

	image = cv::Mat(cv::Size(frame.cols, frame.rows), CV_8UC1, 0.0);

	// define minimum length requirement for any line
	minlength = image.cols * image.cols * 0.001;

	// convert frame into gray
	cv::cvtColor(frame, image, cv::COLOR_BGR2GRAY);

	//resize frame to 480x320
	cv::resize(image, image, cv::Size(image.cols * 0.5, image.rows * 0.5));

	//equalize histogram
	cv::equalizeHist(image, image);

	//initialize the line segment matrix in format y = m*x + c	
	init(image, prevRes);

	//draw lines on image and display
	makeLines(flag);

	//approximate vanishing point
	eval();

	//to calculate fps
	BBtime = cv::getTickCount();
	s_time = (BBtime - AAtime) / cv::getTickFrequency();
	fps_time = 1 / s_time;

	if (showImages) {
		drawVP(frame);
		cv::namedWindow("win", CV_WINDOW_NORMAL);
		cv::namedWindow("Lines", CV_WINDOW_NORMAL);
		cv::namedWindow("Ori", CV_WINDOW_NORMAL);

		cv::imshow("win", image);
		cv::imshow("Lines", drawnLines);
		cv::imshow("Ori", frame);
	}
}

void vanishingPt::drawVP(cv::Mat& imgfile) {
	cv::circle(imgfile, cv::Point(soln(0, 0) * 2, soln(1, 0) * 2), 25, cv::Scalar(0, 180, 255), 3);
	//cv::line(imgfile, cv::Point(0, soln(1, 0) * 2), cv::Point(imgfile.cols, soln(1, 0) * 2), cv::Scalar(10, 0, 255), 2);
	cv::line(imgfile, cv::Point(soln(0, 0) * 2, soln(1, 0) * 2), cv::Point(soln(0, 0) * 2, (soln(1, 0) * 2) - 20), cv::Scalar(10, 0, 255), 2);
	cv::line(imgfile, cv::Point(soln(0, 0) * 2, soln(1, 0) * 2), cv::Point(soln(0, 0) * 2, (soln(1, 0) * 2) + 20), cv::Scalar(10, 0, 255), 2);

	cv::line(imgfile, cv::Point(soln(0, 0) * 2, soln(1, 0) * 2), cv::Point((soln(0, 0) * 2) - 20, soln(1, 0) * 2), cv::Scalar(10, 0, 255), 2);
	cv::line(imgfile, cv::Point(soln(0, 0) * 2, soln(1, 0) * 2), cv::Point((soln(0, 0) * 2) + 20, soln(1, 0) * 2), cv::Scalar(10, 0, 255), 2);
}

void vanishingPt::saveImg(cv::String& imgfile) {
	cv::String charImg;
	charImg = imgfile + "_asli.png";
	cout << charImg << endl;
	imwrite(charImg, frame_ori);

	charImg = imgfile + "_grey.png";
	cout << charImg << endl;
	imwrite(charImg, image);

	charImg = imgfile + "_lines.png";
	imwrite(charImg, drawnLines);

	charImg = imgfile + "_output.png";
	imwrite(charImg, frame);
}

mat vanishingPt::calc(mat A, mat b)
{
	mat x = zeros<mat>(2, 1);
	solve(x, A, b);
	return x;
}

void vanishingPt::eval()
{
	//stores the estimated co-ordinates of the vanishing point with respect to the image
	soln = zeros<mat>(2, 1);

	//initialize error
	double err = 9999999999;

	//calculate point of intersection of every pair of lines and
	//find the sum of distance from all other lines
	//select the point which has the minimum sum of distance
	for (int i = 0; i < points.size(); i++)
	{
		for (int j = 0; j < points.size(); j++)
		{
			if (i >= j)
				continue;

			//armadillo vector
			uvec indices;

			//store indices of lines to be used for calculation
			indices << i << j;

			//extract the rows with indices specified in uvec indices
			//stores the ith and jth row of matrices A and b into Atemp and btemp respectively
			//hence creating a 2x2 matrix for calculating point of intersection
			Atemp = A.rows(indices);
			btemp = b.rows(indices);

			//if lines are parallel then skip
			if (arma::rank(Atemp) != 2)
				continue;

			//solves for 'x' in A*x = b
			res = calc(Atemp, btemp);

			if (res.n_rows == 0 || res.n_cols == 0)
				continue;

			// calculate error assuming perfect intersection is 
			error = A * res - b;

			//reduce size of error
			error = error / 1000;

			// to store intermediate error values
			temperr = 0;
			//summation of errors
			for (int i = 0; i < error.n_rows; i++)
				temperr += (error(i, 0) * error(i, 0)) / 1000;

			//scale errors to prevent any overflows
			temperr /= 1000000;

			//if current error is smaller than previous min error then update the solution (point)
			if (err > temperr)
			{
				soln = res;
				err = temperr;
			}
		}
	}

	//cout << "\n\nResult:\n" << soln(0, 0) << "," << soln(1, 0) << "\nError:" << err;

	// draw a circle to visualize the approximate vanishing point
	//if (soln(0, 0) > 0 && soln(0, 0) < image.cols && soln(1, 0) > 0 && soln(1, 0) < image.rows) {
	//	cv::circle(image, cv::Point(soln(0, 0), soln(1, 0)), 25, cv::Scalar(0, 180, 255), 10);
	//}

	//flush the vector
	points.clear();

	//toDo: use previous frame's result to reduce calculations and stabilize the region of vanishing point
	prevRes = soln;
}

void vanishingPt::makeLines(int flag)
{
	// to solve Ax = b for x
	A = zeros<mat>(points.size(), 2);
	b = zeros<mat>(points.size(), 1);

	//convert given end-points of line segment into a*x + b*y = c format for calculations
	//do for each line segment detected
	for (int i = 0; i < points.size(); i++)
	{
		A(i, 0) = -(points[i][3] - points[i][1]);			//-(y2-y1)
		A(i, 1) = (points[i][2] - points[i][0]);				//x2-x1
		b(i, 0) = A(i, 0) * points[i][0] + A(i, 1) * points[i][1];	//-(y2-y1)*x1 + (x2-x1)*y1
	}
}

void vanishingPt::init(cv::Mat image, mat prevRes)
{
	//create OpenCV object for line segment detection
	cv::Ptr<cv::LineSegmentDetector> ls = cv::createLineSegmentDetector(cv::LSD_REFINE_STD);

	//initialize
	lines_std.clear();

	//detect lines in image and store in linse_std
	//store (x1, y1) and (x2, y2) endpoints for each line segment
	ls->detect(image, lines_std);

	// Show found lines
	cv::Mat drawlines(image);
	drawnLines = drawlines.clone();

	for (int i = 0; i < lines_std.size(); i++)
	{
		//ignore if almost vertical
		if (abs(lines_std[i][0] - lines_std[i][2]) < 10 || abs(lines_std[i][1] - lines_std[i][3]) < 10) //check if almost vertical
			continue;
		//ignore shorter lines (x1-x2)^2 + (y2-y1)^2 < minlength
		if (((lines_std[i][0] - lines_std[i][2]) * (lines_std[i][0] - lines_std[i][2]) + (lines_std[i][1] - lines_std[i][3]) * (lines_std[i][1] - lines_std[i][3])) < minlength)
			continue;

		//store valid lines' endpoints for calculations
		for (int j = 0; j < 4; j++)
		{
			temp.push_back(lines_std[i][j]);
		}

		points.push_back(temp);
		temp.clear();
	}
	ls->drawSegments(drawnLines, lines_std);

	//cout<<"Detected:"<<lines_std.size()<<endl;
	//cout<<"Filtered:"<<points.size()<<endl;
}

float vanishingPt::get_solnX() {
	return soln(0, 0) * 2;
}

float vanishingPt::get_solnY() {
	return soln(1, 0) * 2;
}

cv::Mat vanishingPt::get_frame() {
	cv::resize(frame, frame, cv::Size(frame.cols * 1, frame.rows * 1));
	return frame;
}

double vanishingPt::get_s_time() {
	return s_time * 1000;
}

double vanishingPt::get_fps_time() {
	return fps_time;
}