//
//			--> MODE CPU <--
//	Uji hasil training dengan file Video. 
//	Periksa kembali file video (VIDEO_FILE), file training (TRAINED_SVM), 
//	file template (TEMP_CROP), dan check deteksi warna (COLOR_DET).
//

#include <iostream>
#include <string>
#include "../socket-client-save/hog.h"
#include "../socket-client-save/color_det.h"

#define COLOR_DET true
#define TEMP_CROP "D:/source/repos/Thesis2/hog_cpu/file_temp.jpg"
#define TRAINED_SVM "D:/source/repos/Thesis/x64/Release/2.72x96/filehog_flip0_twice1.xml"

String VIDEO_FILE = "D:/source/repos/PlayVideo/x64/Release/experiment/2_person.avi";
//String VIDEO_FILE = "0";

void rekam_temp(char key, Mat& img, vector< Rect >& loc);

int main(int argv, char** argc)
{
	VideoCapture video;
	Mat img, draw;

	char key = 27;
	int i = 0;
	char fileasli[30], filedet[30];
	hog hogdescriptor(TRAINED_SVM);
	hogdescriptor.set_threshold(0.5);			// hit Threshold
	hogdescriptor.set_resized(1);			// change size 0.5 times from the original

	color_det detColor;
	detColor.set_img(TEMP_CROP);

	namedWindow("all detect", WINDOW_FREERATIO);

	if (VIDEO_FILE != "")
	{
		if (VIDEO_FILE.size() == 1 && isdigit(VIDEO_FILE[0]))
			video.open(VIDEO_FILE[0] - '0');
		else
			video.open(VIDEO_FILE);
	}

	//processing time check
	unsigned long AAtime = 0, BBtime = 0;
	int iteration = 0;
	double time_tot = 0, fps_tot = 0;

	bool end_of_process = false;
	while (!end_of_process)
	{
		vector< Mat > bbox;

		video >> img;
		if (img.empty())
			break;

		AAtime = getTickCount();

		draw = img.clone();

		hogdescriptor.set_img(img);
		hogdescriptor.computeHOG();

		vector< Rect > loc = hogdescriptor.get_locations();
		if (loc.size() != 0) {
			//*** Save file img without detection ***
			sprintf_s(fileasli, "asli_%03d.jpg", iteration);
			//imwrite(fileasli, img);

			if (COLOR_DET) {
				detColor.colordet(draw, loc, 32, false);
				detColor.draw_img(draw, loc);
			}
			else {
				hogdescriptor.draw_locations(draw, loc, Scalar(0, 255, 0));
			}
		}

		BBtime = getTickCount();
		double s_time = (BBtime - AAtime) / getTickFrequency();
		double fps_time = 1 / s_time;
		printf("%04d. %5.2lf msec -- %5.2lf msec \n", iteration, hogdescriptor.get_s_time(), detColor.get_s_time());

		time_tot = s_time + time_tot;
		fps_tot = fps_time + fps_tot;


		//*** Show Detection ***
		imshow("all detect", draw);				// based HOG Descriptor

		//*** Save file img with detection ***
		sprintf_s(filedet, "det_%03d.jpg", iteration);
		//imwrite(filedet, img);

		iteration++;
		key = (char)waitKey(1);
		if (27 == key)
			end_of_process = true;
		else if (32 == key) {
			rekam_temp(key, img, loc);
		}
	}

	time_tot = time_tot / iteration;
	fps_tot = fps_tot / iteration;
	printf("\n\nJumlah Frame %d\n", iteration);
	printf("rata-rata adalah %.2lf sec / %.2lf fps \n", time_tot, fps_tot);
	waitKey(0);

	cv::destroyAllWindows();
	return 0;
}

void rekam_temp(char key, Mat& img, vector< Rect >& loc) {
	key = (char)waitKey(10000);
	cout << key << endl;
	if (key == 48 || key == 49 || key == 50 || key == 51) {
		int k = 0;
		switch (key)
		{
		case 48:
			k = 0;
			break;
		case 49:
			k = 1;
			break;
		case 50:
			k = 2;
			break;
		case 51:
			k = 3;
			break;
		default:
			break;
		}
		imwrite("file_temp.jpg", img(loc[k]));
	}
}