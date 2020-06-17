//
//	I prefer install armadillo library
//	convert image of people to position (x,y)
//	check the argument[1] -->> (fileimg) for file image
//	also teh file trained -->> TRAINED_SVM
//

#include <iostream>
#include <string>
#include "vanishingPt.h"
#include "../socket-client-save/hog.h"
#include "../socket-client-save/projection.h"
#include "windows.h"

#define TRAINED_SVM "D:/source/repos/Thesis/x64/Release/2.72x96/filehog_flip0_twice1.xml"

int main(int argv, char** argc)
{
	if (argv < 2) {
		return -1;
	}
	String fileimg = argc[1];
	cv::Mat draw, imgfile = imread(fileimg);

	draw = imgfile.clone();

	hog hogdescriptor(TRAINED_SVM);
	hogdescriptor.set_threshold(0);
	hogdescriptor.set_resized(1);
	hogdescriptor.set_img(draw);
	hogdescriptor.computeHOG();

	// make object
	vanishingPt obj;
	obj.vanishingPt_(imgfile, false);

	vector< Rect > loc = hogdescriptor.get_locations();
	printf("ini loc : %d\n", loc.size());

	cv::Point vPoint = Point(obj.get_solnX(), obj.get_solnY());
	cout << vPoint.y << endl;
	projection convertImg;
	convertImg.set_fc(27.73955);		// focal length in milimeter
	convertImg.set_yc(1400);			// height of camera in milimeter
	convertImg.set_uc(1224);			//
	convertImg.set_v0(vPoint.y);		//
	convertImg.compute_projection(loc[0], true);

	namedWindow("all detect", WINDOW_FREERATIO);
	obj.drawVP(draw);
	convertImg.drawVP(draw, 1074.57, 1209.3);
	hogdescriptor.draw_locations(draw, loc, Scalar(0, 255, 0));
	imshow("all detect", draw);

	waitKey(0);

	cv::destroyAllWindows();
	return 0;
}