#include <iostream>
#include <string>
#include "../socket-client-save/hog.h"
#include "../socket-client-save/color_det.h"

//add OpenCV namespaces
using namespace cv;
using namespace std;
using namespace cv::ml;

//#define TRAINED_SVM "filesvm_flip1_twice1.xml"
#define COLOR_DET false
#define TEMP_CROP "E:/dataset/PascalDataSetCropped/pascal_voc_2.jpg"

//String VIDEO_FILE = "E:/dataset/768x576_2_xvid.avi";
//String VIDEO_FILE = "E:/dataset/TownCentreXVID_xvid.avi";
//String VIDEO_FILE = "E:/dataset/Pedestrian overpass.mp4";
//String VIDEO_FILE = "0";

int main(int argc, char** argv)
{
	const char* keys =
	{
		"{help h|     | show help message}"
		"{i     |     | path of directory of Video or Camera}"
		"{th    |     | threshold on HOG Descriptor}"
		"{rsz   |     | resize on HOG Descriptor}"
		"{si    |false| saving frame every detection on frame}"
		"{fsi   |     | folder where save every frame}"
		"{fn    |file | file name of trained SVM}"
	};

	CommandLineParser parser(argc, argv, keys);

	if (parser.has("help"))
	{
		parser.printMessage();
		exit(0);
	}

	String VIDEO_FILE = parser.get< String >("i");
	String TRAINED_SVM = parser.get< String >("fn");
	String FOLDER_SAVE = parser.get< String >("fsi");
	float thres = parser.get< float >("th");
	float resize = parser.get< float >("rsz");
	bool save_image = parser.get< bool >("si");

	if (VIDEO_FILE.empty() || TRAINED_SVM.empty()) {
		parser.printMessage();
		cout << "Wrong number of parameters.\n\n"
			<< "Example command line:\n" << argv[0] << " -i=0 -fn=filesvm_flip0_twice1.xml -th=0.5 -rsz=1\n";
		exit(1);
	}

	if (save_image) {
		if (FOLDER_SAVE.empty()) {
			parser.printMessage();
			cout << "Wrong number of parameters.\n\n"
				<< "Example command line:\n" << argv[0] << " -i=0 -fn=filesvm_flip0_twice1.xml -th=0.5 -rsz=1 -si -fsi=saveimg\n";
			exit(1);
		}

	}

	if (resize == 0) {
		resize = 1;
	}

	VideoCapture video;
	Mat img, draw;
	ofstream outStream("save.txt");

	char key = 27;
	int i = 0;
	double time_all = 0;
	char fileasli[30], filedet[30], txt[50];
	hog hogdescriptor(TRAINED_SVM);
	hogdescriptor.set_threshold(thres);			// hit Threshold
	hogdescriptor.set_resized(resize);				// change size 0.5 times from the original

	color_det detColor;
	detColor.set_img(TEMP_CROP);

	String obj_det_filename = "testing " + string(TRAINED_SVM);
	namedWindow(obj_det_filename, WINDOW_FREERATIO);
	namedWindow("all detect", WINDOW_FREERATIO);

	if (VIDEO_FILE != "")
	{
		if (VIDEO_FILE.size() == 1 && isdigit(VIDEO_FILE[0]))
			video.open(VIDEO_FILE[0] - '0');
		else
			video.open(VIDEO_FILE);
	}

	cout << "width: " << video.get(CAP_PROP_FRAME_WIDTH) * resize << endl;
	cout << "height: " << video.get(CAP_PROP_FRAME_HEIGHT) * resize << endl;

	unsigned long AAtime = 0, BBtime = 0;

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
			sprintf_s(fileasli, "asli_%03d.jpg", i);
			//imwrite(fileasli, img);

			hogdescriptor.draw_locations(draw, loc, Scalar(0, 255, 0));

			if (COLOR_DET) {
				detColor.colordet(img, loc, 4, false);
				detColor.draw_img(img, loc);
			}
			else {
				hogdescriptor.draw_locations(img, loc, Scalar(0, 255, 0));
			}
		}

		BBtime = getTickCount();
		double s_time = (BBtime - AAtime) / getTickFrequency();
		double fps_time = 1 / s_time;
		std::printf("%04d. %.2lf msec / %5.2lf fps -- n=%02d\n", i, s_time * 1000, fps_time, loc.size());

		//*** Show Detection ***
		cv::imshow(obj_det_filename, img);			// based HOG Descriptor and Color Detection
		cv::imshow("all detect", draw);				// based HOG Descriptor

		//*** Save Images all detect ***
		if (save_image) {
			sprintf_s(filedet, "/alldet_%04d.jpg", i);
			string y = FOLDER_SAVE + filedet;
			imwrite(y, draw);

			sprintf_s(txt, "%04d. %.2lf msec / %5.2lf fps -- n %02d", i, hogdescriptor.get_s_time(), fps_time, loc.size());
			outStream << txt << endl;
		}

		//*** Save file img with Color Detection ***
		sprintf_s(filedet, "det_%03d.jpg", i);
		//imwrite(filedet, img);

		//***
		time_all = time_all + hogdescriptor.get_s_time();

		i++;
		key = (char)waitKey(1);
		if (27 == key)
			end_of_process = true;
	}
	sprintf_s(txt, "\nrata-rata %.2lf msec / %5.2lf fps ", (time_all / i), (1 / (time_all / i / 1000)));
	outStream << txt << endl;

	outStream.close();
	cv::destroyAllWindows();
	return 0;
}