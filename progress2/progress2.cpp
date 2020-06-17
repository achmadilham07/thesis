#include "../socket-client-save/hog.h"
#include "../socket-client-save/color_det.h"
//#include "../../Thesis/hog_online/color_det_weighted.h"
#include "../socket-client-save/projection.h"
#include "../haar_online/haar.h"
//#include "../../hshist/plot_graph/graph.h"
//#include <QtWidgets/QApplication>

//#define TEMP_CROP "D:/source/repos/Thesis2/hog_cpu/file_temp.jpg"
#define TEMP_CROP "file_temp.jpg"

//String VIDEO_FILE = "C:/Users/hp/Videos/Any Video Converter Ultimate/AVI/output_01_Segment_0_xvid.avi";

//String VIDEO_FILE = "D:/source/repos/PlayVideo/x64/Release/trial01/output_09.avi";
//String VIDEO_FILE = "D:/source/repos/PlayVideo/x64/Release/experiment/1_person.avi";
//String VIDEO_FILE = "0";
String VIDEO_FILE = "D:/source/repos/PlayVideo/x64/Release/output_04.avi";
String TRAINED_SVM = "D:/source/repos/Thesis/x64/Release/2.72x96/filehog_flip0_twice1.xml";
String TRAINED_HAAR = "D:/source/repos/Thesis/haar_online/haarcascade_upperbody.xml";
bool COLOR_DET = false;
bool SAVE_IMG = false;
bool REPORT = false;
bool SAVE_VID = false;

void rekam_temp(Mat& img, vector< Rect >& loc, color_det detColor_hog);
bool file_exists(const string& file);

int main(int argc, char** argv)
{
	const char* keys =
	{
		"{help h|     | show help message}"
		"{iv    |     | Directory video(current folder or spesific dir)}"
		"{t     |     | Directory trained SVM (current folder or spesific dir)}"
		"{th    |     | threshold on HOG Descriptor}"
		"{rsz   |     | resize on HOG Descriptor}"
		"{si    |     | save images(real image from video and output detection)}"
		"{re    |     | report all data(coordinates and computation time_hog)}"
		"{cd    |     | file_temp.jpg become template}"
		"{sv    |     | save video}"
	};

	CommandLineParser parser(argc, argv, keys);

	if (parser.has("help"))
	{
		parser.printMessage();
		exit(0);
	}

	VIDEO_FILE = parser.get< String >("iv");
	TRAINED_SVM = parser.get< String >("t");
	float thres = parser.get< float >("th");
	float resize = parser.get< float >("rsz");
	SAVE_IMG = parser.get< bool >("si");
	REPORT = parser.get< bool >("re");
	COLOR_DET = parser.get< bool >("cd");
	SAVE_VID = parser.get< bool >("sv");
	/*
		VIDEO_FILE = "output_07.avi";
		TRAINED_SVM = "filehog_flip0_twice1_new.xml";
		float thres = 0.1;
		float resize = 1.0;
		SAVE_IMG = false;
		REPORT = false;
		COLOR_DET = false;
	*/
	VideoCapture video;
	VideoWriter videowrite;
	Mat img, draw, drawhaar;
	ofstream coord_hog, coord_haar, time_hog, time_haar;

	char key = 27;
	int i = 0;
	char fileasli[30], filedet[30], namefile[30];

	if (REPORT) {
		cout << "REPORT" << endl;
		sprintf_s(namefile, "%s_coord_hog.txt", VIDEO_FILE.c_str());
		coord_hog.open(namefile);
		/*sprintf_s(namefile, "%s_coord_haar.txt", VIDEO_FILE.c_str());
		coord_haar.open(namefile);*/
		sprintf_s(namefile, "%s_time_hog.txt", VIDEO_FILE.c_str());
		/*time_hog.open(namefile);
		sprintf_s(namefile, "%s_time_haar.txt", VIDEO_FILE.c_str());
		time_haar.open(namefile);*/
	}

	double vp_x = 339.921;			// vanishing point X (pixel)
	double vp_y = 197.72;			// vanishing point Y (pixel)
	double c_x = 320;				// horizontal center point (pixel)
	double c_y = 240;				// vertical center point (pixel)
	double fc = 728.3926;			// focal length (pixel)
	double yc = 1500;				// height of camera (milimeter)
	double uc = c_x;				//

	//double vp_x = 312.027;			// vanishing point X (pixel)
	//double vp_y = 97.1087;			// vanishing point Y (pixel)
	//double c_x = 320;				// horizontal center point (pixel)
	//double c_y = 240;				// vertical center point (pixel)
	//double fc = 728.3926;			// focal length (pixel)
	//double yc = 2020;				// height of camera (milimeter)
	//double uc = c_x;				//

	hog hogdescriptor(TRAINED_SVM);
	hogdescriptor.set_threshold(thres);		// hit Threshold
	hogdescriptor.set_resized(resize);			// change size 0.5 times from the original

	haar haarcascade(TRAINED_HAAR);

	color_det detColor_hog, detColor_haar;
	detColor_hog.set_img(TEMP_CROP);
	detColor_haar.set_img(TEMP_CROP);

	projection cam_model_hog, cam_model_haar;
	cam_model_hog.set_h(1.9);
	cam_model_hog.set_fc(fc);
	cam_model_hog.set_yc(yc);
	cam_model_hog.set_uc(uc);
	cam_model_hog.set_v0(vp_y);

	cam_model_haar.set_h(2.7);
	cam_model_haar.set_fc(fc);
	cam_model_haar.set_yc(yc);
	cam_model_haar.set_uc(uc);
	cam_model_haar.set_v0(vp_y);
	/*
		QApplication a(argc, argv);
		graph g;
	*/
	namedWindow("all detect", WINDOW_FREERATIO);
	//	namedWindow("haar", CV_WINDOW_FREERATIO);

	if (VIDEO_FILE != "")
	{
		if (VIDEO_FILE.size() == 1 && isdigit(VIDEO_FILE[0]))
			video.open(VIDEO_FILE[0] - '0');
		else
			video.open(VIDEO_FILE);
	}

	if (SAVE_VID && !isdigit(VIDEO_FILE[0])) {
		std::string delimiter = ".";
		std::string token = VIDEO_FILE.substr(0, VIDEO_FILE.find(delimiter)); // token is "scott"
		sprintf_s(namefile, "%s_hog.avi", token);
		int frame_width = video.get(CAP_PROP_FRAME_WIDTH);
		int frame_height = video.get(CAP_PROP_FRAME_HEIGHT);
		videowrite.open(namefile, VideoWriter::fourcc('M', 'J', 'P', 'G'), 30, Size(frame_width, frame_height));
	}

	//processing time_hog check
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
		drawhaar = img.clone();

		hogdescriptor.set_img(img);
		hogdescriptor.computeHOG();

		//		haarcascade.set_img(img);
		//		haarcascade.computeHAAR();

		vector< Rect > loc = hogdescriptor.get_locations();
		if (loc.size() != 0) {
			//*** Save file img without detection ***
			sprintf_s(fileasli, "asli_%03d", iteration);
			//imwrite(fileasli, img);

			if (COLOR_DET) {
				detColor_hog.colordet(draw, loc, 4, false);
				cam_model_hog.compute_projection(detColor_hog.get_loc_color(), false);

				if (REPORT) {
					//cout << iteration << endl;
					time_hog << iteration << "\t" << hogdescriptor.get_s_time() << "\t" << detColor_hog.get_s_time() << "\t" << cam_model_hog.get_s_time() << endl;
					coord_hog << iteration << "\t" << cam_model_hog.get_coord_x() << "\t" << cam_model_hog.get_coord_y() << "\t" << cam_model_hog.get_coord_z() << endl;
				}
				//g.draw_lines(fileasli, cam_model_hog.get_coord_x(), cam_model_hog.get_coord_y());
			}
			else {
				hogdescriptor.draw_locations(draw, loc, Scalar(0, 255, 0));
			}
		}

		//vector< Rect > loc_haar = haarcascade.get_locations();
		//if (loc_haar.size() != 0) {
		//	//*** Save file img without detection ***
		//	sprintf_s(fileasli, "asli_%03d", iteration);
		//	//imwrite(fileasli, img);

		//	if (COLOR_DET) {
		//		detColor_haar.colordet(drawhaar, loc_haar);
		//		cam_model_haar.compute_projection(detColor_haar.get_loc_color(), false);
		//		if (REPORT) {
		//			//cout << iteration << endl;
		//			time_haar << iteration << "\t" << haarcascade.get_s_time() << "\t" << detColor_haar.get_s_time() << "\t" << cam_model_haar.get_s_time() << endl;
		//			coord_haar << iteration << "\t" << cam_model_haar.get_coord_x() << "\t" << cam_model_haar.get_coord_y() << "\t" << cam_model_haar.get_coord_z() << endl;
		//		}
		//		//g.draw_lines(fileasli, cam_model_hog.get_coord_x(), cam_model_hog.get_coord_y());
		//	}
		//	else {
		//		haarcascade.draw_locations(drawhaar, Scalar(0, 255, 0));
		//	}
		//}

		//*** Save file img with detection ***
		if (SAVE_IMG && COLOR_DET) {
			/*sprintf_s(filedet, "save/img_%d.jpg", iteration);
			imwrite(filedet, img);*/
			sprintf_s(filedet, "hog/color/draw_%d.jpg", iteration);
			imwrite(filedet, draw);
			/*sprintf_s(filedet, "haar/color/drawhaar_%d.jpg", iteration);
			imwrite(filedet, drawhaar);*/
		}
		else if (SAVE_IMG) {
			/*sprintf_s(filedet, "save/img_%04d.jpg", iteration);
			imwrite(filedet, img);*/
			sprintf_s(filedet, "hog/draw_%04d.jpg", iteration);
			imwrite(filedet, draw);
			/*sprintf_s(filedet, "haar/drawhaar_%d.jpg", iteration);
			imwrite(filedet, drawhaar);*/
		}

		if (SAVE_VID) {
			videowrite.write(draw);
		}

		BBtime = getTickCount();
		double s_time = (BBtime - AAtime) / getTickFrequency();
		double fps_time = 1 / s_time;
		//printf("%04d. %5.2lf msec -- %5.2lf msec \n", iteration, hogdescriptor.get_s_time(), detColor_hog.get_s_time());

		time_tot = s_time + time_tot;
		fps_tot = fps_time + fps_tot;

		//*** Show Detection ***
		imshow("all detect", draw);				// based HOG Descriptor
//		imshow("haar", drawhaar);				// based HAAR Cascade

		iteration++;
		key = (char)waitKey(1);
		if (27 == key)
			end_of_process = true;
		else if (32 == key) {
			rekam_temp(img, loc, detColor_hog);
		}
	}

	time_tot = time_tot / iteration;
	fps_tot = fps_tot / iteration;
	printf("\n\nJumlah Frame %d\n", iteration);
	printf("rata-rata adalah %.2lf sec / %.2lf fps \n", time_tot, fps_tot);
	//waitKey(0);

	if (REPORT) {
		coord_hog.close();
		coord_haar.close();
		time_hog.close();
		time_haar.close();
	}
	if (SAVE_VID) {
		videowrite.release();
	}
	cv::destroyAllWindows();
	return 0;
}

void rekam_temp(Mat& img, vector< Rect >& loc, color_det detColor_hog) {
	char namefile[50];
	struct tm newtime;
	time_t now = time(0);
	localtime_s(&newtime, &now);
	int Year = 1900 + newtime.tm_year;
	int Month = 1 + newtime.tm_mon;
	int Day = newtime.tm_mday;
	int Hour = newtime.tm_hour;
	int Min = newtime.tm_min;
	int Sec = newtime.tm_sec;

	char key = (char)waitKey(15000);

	if (key >= 48 && key <= 51) {
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
		sprintf_s(namefile, "temp_%04d_%02d_%02d_%02d_%02d_%02d.jpg", Year, Month, Day, Hour, Min, Sec);
		imwrite(namefile, img(loc[k]));
		imwrite("file_temp.jpg", img(loc[k]));
		COLOR_DET = true;
		detColor_hog.set_img(namefile);
	}
}

bool file_exists(const string& file)
{
	return access(file.c_str(), 0) == 0;
}