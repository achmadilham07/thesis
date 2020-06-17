//
//			--> MODE GPU <-- 
//	install dulu opencv[cuda].
//	saya sarankan gunakan 64bit / x64 / x64-windows
//
//	Uji hasil training dengan file Video. 
//	Periksa kembali file video (filename), file training (TRAINED_SVM), 
//	file template (TEMP_CROP), dan check deteksi warna (COLOR_DET).
//

#include <iostream>  
#include "opencv2\objdetect\objdetect.hpp"
#include "opencv2\highgui\highgui.hpp"
#include "opencv2\imgproc\imgproc.hpp"
#include "opencv2\cudaobjdetect.hpp"
#include "opencv2\cudaimgproc.hpp"
#include "opencv2\cudawarping.hpp"
#include "opencv2\opencv.hpp"
#include <windows.h>

#ifdef _DEBUG               
#pragma comment(lib, "opencv_core343d.lib")       
#pragma comment(lib, "opencv_highgui343d.lib")    
#pragma comment(lib, "opencv_imgcodecs343d.lib")  
#pragma comment(lib, "opencv_objdetect343d.lib")  
#pragma comment(lib, "opencv_imgproc343d.lib")  
#pragma comment(lib, "opencv_videoio343d.lib")    
#pragma comment(lib, "opencv_cudaobjdetect343d.lib")  
#pragma comment(lib, "opencv_cudawarping343d.lib")
#pragma comment(lib, "opencv_cudaimgproc343d.lib")
#endif        

using namespace std;
using namespace cv;
using namespace cv::ml;

void get_svm_detector(const Ptr<SVM>& svm, vector< float >& hog_detector);

void main()
{
	String TRAINED_SVM = "D:/source/repos/Thesis/x64/Release/72x96/filesvm_flip1_twice1.xml";
	Ptr<SVM> svm;
	svm = StatModel::load<SVM>(TRAINED_SVM);
	//svm = StatModel::load<SVM>("C:/Users/hp/source/repos/Thesis/train_hog/grey_64/filesvm_flip1_twice1.xml");

	Ptr<cuda::HOG> d_hog = cuda::HOG::create(Size(72, 96)); //(Size(48, 96)); //(Size(64, 128));// 

	vector< float > hog_detector;
	get_svm_detector(svm, hog_detector);
	d_hog->setSVMDetector(hog_detector);

	//d_hog->setSVMDetector(d_hog->getDefaultPeopleDetector());

	d_hog->setHitThreshold(0);

	//video loading
	Mat img;
	String filename = "D:/Downloads/SamsungA6/20190518_110134.mp4"; //Pedestrian overpass.mp4
	VideoCapture  cap(filename);

	//loading test
	cap >> img;
	if (img.empty())
		return;

	//window
	namedWindow("pedestrian", 0);

	//processing time check
	unsigned long AAtime = 0, BBtime = 0;

	//resize
	double scale = 1;// float(800) / img.cols;
	cuda::GpuMat GpuImg, rGpuImg;
	GpuImg.upload(img);
	cuda::resize(GpuImg, rGpuImg, Size(GpuImg.cols * scale, GpuImg.rows * scale));
	Mat rInimg;
	rGpuImg.download(rInimg);

	int iteration = 0;
	double time_tot = 0, fps_tot = 0;


	while (1)
	{
		//time check
		DWORD start = GetTickCount(); // program starts
		AAtime = getTickCount();

		//loading
		cap >> img;
		if (img.empty())
			break;

		//resize
		GpuImg.upload(img);
		cuda::resize(GpuImg, rGpuImg, Size(GpuImg.cols * scale, GpuImg.rows * scale));
		rGpuImg.download(rInimg);
		cuda::cvtColor(rGpuImg, rGpuImg, COLOR_BGR2GRAY);

		vector< Point> found_locations;
		vector<double> confidences = { 1.4 };
		d_hog->detect(rGpuImg, found_locations);

		vector< Rect> found_locations_rect;
		d_hog->detectMultiScale(rGpuImg, found_locations_rect);

		char cropsave[30];

		for (int i = 0; i < found_locations_rect.size(); ++i)
		{
			rectangle(rInimg, found_locations_rect[i], Scalar(0, 255, 0), 2);
		}

		//imshow("pedestrian", rInimg);

		sprintf_s(cropsave, "%03d.png", iteration);
		imshow("pedestrian", rInimg);
		//imwrite(cropsave, rInimg);
		iteration++;

		waitKey(10);

		//time check
		DWORD end = GetTickCount(); // program ends
		BBtime = getTickCount();
		double s_time = (BBtime - AAtime) / getTickFrequency();
		//double s_time = (end - start) / 1000;
		double fps_time = 1 / s_time;
		printf("%d. %.2lf msec / %.2lf fps \n", iteration, s_time * 1000, fps_time);

		time_tot = s_time + time_tot;
		fps_tot = fps_time + fps_tot;
	}
	time_tot = time_tot / iteration;
	fps_tot = fps_tot / iteration;
	printf("\n\nJumlah Frame %d\n", iteration);
	printf("rata-rata adalah %.2lf sec / %.2lf fps \n", time_tot, fps_tot);
	waitKey(0);
}

void get_svm_detector(const Ptr<SVM>& svm, vector< float >& hog_detector)
{
	// get the support vectors
	Mat sv = svm->getSupportVectors();
	const int sv_total = sv.rows;
	// get the decision function
	Mat alpha, svidx;
	double rho = svm->getDecisionFunction(0, alpha, svidx);

	CV_Assert(alpha.total() == 1 && svidx.total() == 1 && sv_total == 1);
	CV_Assert((alpha.type() == CV_64F && alpha.at<double>(0) == 1.) ||
		(alpha.type() == CV_32F && alpha.at<float>(0) == 1.f));
	CV_Assert(sv.type() == CV_32F);
	hog_detector.clear();

	hog_detector.resize(sv.cols + 1);
	memcpy(&hog_detector[0], sv.ptr(), sv.cols * sizeof(hog_detector[0]));
	hog_detector[sv.cols] = (float)-rho;
}