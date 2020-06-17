#pragma once
#include <algorithm>
#define PI 3.14159265

using namespace std;

//double calculateRadian(double xold, double yold, double xnew, double ynew) {
//	double Orientation = 0;
double calculateRadian(double x1, double y1, double x2, double y2) {
	double Orientation = 0;
	double xold = round(x1 * 100) / 100;
	double yold = round(y1 * 100) / 100;
	double xnew = round(x2 * 100) / 100;
	double ynew = round(y2 * 100) / 100;
	double xdiff = xnew - xold;
	double ydiff = ynew - yold;

	// calculate arc tan
	double theta = atan(xdiff / ydiff) * 180 / PI;

	// check quadrant
	if (xdiff < 0 && ydiff > 0) {
		Orientation = theta;
	}
	else if (xdiff < 0 && ydiff < 0) {
		Orientation = -180 + theta;
	}
	else if (xdiff > 0 && ydiff < 0) {
		Orientation = 180 + theta;
	}
	else if (xdiff > 0 && ydiff > 0) {
		Orientation = theta;
	}
	// if the distances is zero
	else if (xdiff > 0 && ydiff == 0) {
		Orientation = 90;
	}
	else if (xdiff < 0 && ydiff == 0) {
		Orientation = -90;
	}
	else if (xdiff == 0 && ydiff < 0) {
		Orientation = 180;
	}
	else if (xdiff == 0 && ydiff > 0) {
		Orientation = 0;
	}
	else {
		Orientation = 0;
	}

	// print out
	/*if (isnan(OBO)) {
		printf("bukan\n");
	}
	else {
		printf("%f\n", OBO);
	}*/
	return Orientation;
}
