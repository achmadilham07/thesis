#include <iostream>
#include <stdio.h>
#include "haar.h"

int main() {
	Mat frame;
	VideoCapture cap;
	String VIDEO_FILE = "0";
	String trained_haar = "haarcascade_upperbody.xml";
	String window_name = "Capture - Face detection";

	haar detectHaar(trained_haar);

	namedWindow(window_name, WINDOW_FREERATIO);

	if (VIDEO_FILE != "")
	{
		if (VIDEO_FILE.size() == 1 && isdigit(VIDEO_FILE[0]))
			cap.open(VIDEO_FILE[0] - '0');
		else
			cap.open(VIDEO_FILE);
	}

	while (cap.read(frame)) {
		if (frame.empty())
		{
			printf(" --(!) No captured frame -- Break!");
			break;
		}

		detectHaar.set_img(frame);
		detectHaar.computeHAAR();
		detectHaar.draw_locations(frame, Scalar(0, 255, 0));

		imshow(window_name, frame);

		char c = (char)waitKey(10);
		if (c == 27) { break; } // escape

	}

	return 0;
}