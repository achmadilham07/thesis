#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/ml.hpp"
#include "opencv2/objdetect.hpp"

#include <iostream>
#include <time.h>
#include <direct.h> 
#include <process.h> 
#include <stdio.h> 

using namespace cv;
using namespace cv::ml;
using namespace std;

vector< float > get_svm_detector(const Ptr< SVM >& svm);
void convert_to_ml(const std::vector< Mat >& train_samples, Mat& trainData);
void load_images(const String& dirname, vector< Mat >& img_lst, bool showImages);
void sample_neg(const vector< Mat >& full_neg_lst, vector< Mat >& neg_lst, const Size& size);
void computeHOGs(const Size wsize, const vector< Mat >& img_lst, vector< Mat >& gradient_lst, bool use_flip, bool visualization, const char* NegOrPos);
void test_trained_detector(String obj_det_filename, String test_dir, String videofilename);
Mat get_hogdescriptor_visu(const Mat& color_origImg, vector<float>& descriptorValues, const Size& size);
int dirExists(const char* path);

char namefile[30];

int main(int argc, char** argv)
{
	//String pos_dir = "E:\\dataset\\PascalDataSetCropped\\64\\color";
	//String neg_dir = "E:\\dataset\\PascalDataSetCropped\\neg";
	//String test_dir = "E:\\dataset\\PascalDataSetCropped";
	//String obj_det_filename = "filesvm_flip1_twice1.xml"; // "filesvm_flip0_twice0.xml";
	//String videofilename = "E:\\dataset\\768x576_2.avi"; //Pedestrian overpass.mp4 //Pedestrian Walking.mp4
	//int detector_width = 64;
	//int detector_height = 64;
	//bool test_detector = false;
	//bool train_twice = true;
	//bool visualization = false;
	//bool flip_samples = true;

	const char* keys =
	{
		"{help h|     | show help message}"
		"{pd    |     | path of directory contains positive images}"
		"{nd    |     | path of directory contains negative images}"
		"{td    |     | path of directory contains test images}"
		"{tv    |     | test video file name}"
		"{dw    |     | width of the detector}"
		"{dh    |     | height of the detector}"
		"{f     |false| indicates if the program will generate and use mirrored samples or not}"
		"{d     |false| train twice}"
		"{v     |false| visualize training steps}"
		//"{t     |false| test a trained detector}"
		//"{fn    |my_detector.yml| file name of trained SVM}"
	};

	CommandLineParser parser(argc, argv, keys);

	if (parser.has("help"))
	{
		parser.printMessage();
		exit(0);
	}

	String pos_dir = parser.get< String >("pd");
	String neg_dir = parser.get< String >("nd");
	String test_dir = parser.get< String >("td");
	//String obj_det_filename = parser.get< String >("fn");
	String videofilename = parser.get< String >("tv");
	int detector_width = parser.get< int >("dw");
	int detector_height = parser.get< int >("dh");
	//bool test_detector = parser.get< bool >("t");
	bool train_twice = parser.get< bool >("d");
	bool visualization = parser.get< bool >("v");
	bool flip_samples = parser.get< bool >("f");

	if (pos_dir.empty() || neg_dir.empty() || detector_width == NULL || detector_height == NULL)
	{
		parser.printMessage();
		cout << "Wrong number of parameters.\n\n"
			<< "Example command line:\n" << argv[0] << " -dw=72 -dh=96 -pd=E:/dataset/PascalDataSetCropped/72x96/grey -nd=E:/dataset/PascalDataSetCropped/neg -td=E:/dataset/INRIAPerson/Test/pos -d -f -v\n";
		exit(1);
	}

	vector< Mat > pos_lst, full_neg_lst, neg_lst, gradient_lst;
	vector< int > labels;

	clog << "Positive images are being loaded...";
	load_images(pos_dir, pos_lst, visualization);
	if (pos_lst.size() > 0)
	{
		clog << "...[done]" << endl;
	}
	else
	{
		clog << "no image in " << pos_dir << endl;
		return 1;
	}

	Size pos_image_size = pos_lst[0].size();

	if (detector_width && detector_height)
	{
		pos_image_size = Size(detector_width, detector_height);
	}
	else
	{
		for (size_t i = 0; i < pos_lst.size(); ++i)
		{
			if (pos_lst[i].size() != pos_image_size)
			{
				cout << "All positive images should be same size!" << endl;
				exit(1);
			}
		}
		pos_image_size = pos_image_size / 8 * 8;
	}

	clog << "Negative images are being loaded...";
	load_images(neg_dir, full_neg_lst, false);
	sample_neg(full_neg_lst, neg_lst, pos_image_size);
	clog << "...[done]" << endl;

	clog << "Histogram of Gradients are being calculated for positive images...";
	computeHOGs(pos_image_size, pos_lst, gradient_lst, flip_samples, visualization, "./pos");
	size_t positive_count = gradient_lst.size();
	labels.assign(positive_count, +1);
	clog << "...[done] ( positive count : " << positive_count << " )" << endl;

	clog << "Histogram of Gradients are being calculated for negative images...";
	computeHOGs(pos_image_size, neg_lst, gradient_lst, flip_samples, visualization, "./neg");
	size_t negative_count = gradient_lst.size() - positive_count;
	labels.insert(labels.end(), negative_count, -1);
	CV_Assert(positive_count < labels.size());
	clog << "...[done] ( negative count : " << negative_count << " )" << endl;

	Mat train_data;
	convert_to_ml(gradient_lst, train_data);

	clog << "Training SVM...";
	Ptr< SVM > svm = SVM::create();
	/* Default values to train SVM */
	svm->setCoef0(0.0);
	svm->setDegree(3);
	svm->setTermCriteria(TermCriteria(TermCriteria::MAX_ITER + TermCriteria::EPS, 1000, 1e-3));
	svm->setGamma(0);
	svm->setKernel(SVM::LINEAR);
	svm->setNu(0.5);
	svm->setP(0.1); // for EPSILON_SVR, epsilon in loss function?
	svm->setC(0.01); // From paper, soft classifier
	svm->setType(SVM::EPS_SVR); // C_SVC; // EPSILON_SVR; // may be also NU_SVR; // do regression task
	svm->train(train_data, ROW_SAMPLE, labels);
	clog << "...[done]" << endl;

	sprintf_s(namefile, "%s%s_flip%d_twice%d.xml", "file", "svm", flip_samples, train_twice);
	svm->save(namefile);

	if (train_twice)
	{
		clog << "Testing trained detector on negative images. This may take a few minutes...";
		HOGDescriptor my_hog;
		my_hog.winSize = pos_image_size;

		// Set the trained svm to my_hog
		my_hog.setSVMDetector(get_svm_detector(svm));

		vector< Rect > detections;
		vector< double > foundWeights;

		for (size_t i = 0; i < full_neg_lst.size(); i++)
		{
			if (full_neg_lst[i].cols >= pos_image_size.width && full_neg_lst[i].rows >= pos_image_size.height)
				my_hog.detectMultiScale(full_neg_lst[i], detections, foundWeights);
			else
				detections.clear();

			for (size_t j = 0; j < detections.size(); j++)
			{
				Mat detection = full_neg_lst[i](detections[j]).clone();
				resize(detection, detection, pos_image_size, 0, 0, INTER_LINEAR_EXACT);
				neg_lst.push_back(detection);
			}

			if (visualization)
			{
				for (size_t j = 0; j < detections.size(); j++)
				{
					rectangle(full_neg_lst[i], detections[j], Scalar(0, 255, 0), 2);
				}
				imshow("testing trained detector on negative images", full_neg_lst[i]);
				waitKey(5);
			}
		}
		clog << "...[done]" << endl;

		gradient_lst.clear();
		clog << "Histogram of Gradients are being calculated for positive images...";
		computeHOGs(pos_image_size, pos_lst, gradient_lst, flip_samples, visualization, "./pos");
		positive_count = gradient_lst.size();
		clog << "...[done] ( positive count : " << positive_count << " )" << endl;

		clog << "Histogram of Gradients are being calculated for negative images...";
		computeHOGs(pos_image_size, neg_lst, gradient_lst, flip_samples, visualization, "./neg");
		negative_count = gradient_lst.size() - positive_count;
		clog << "...[done] ( negative count : " << negative_count << " )" << endl;

		labels.clear();
		labels.assign(positive_count, +1);
		labels.insert(labels.end(), negative_count, -1);

		clog << "Training SVM again...";
		convert_to_ml(gradient_lst, train_data);
		svm->train(train_data, ROW_SAMPLE, labels);
		clog << "...[done]" << endl;

		sprintf_s(namefile, "%s%s_flip%d_twice%d.xml", "file", "svm", flip_samples, train_twice);
		svm->save(namefile);
	}

	HOGDescriptor hog;
	hog.winSize = pos_image_size;
	hog.setSVMDetector(get_svm_detector(svm));

	sprintf_s(namefile, "%s%s_flip%d_twice%d.xml", "file", "hog", flip_samples, train_twice);

	hog.save(namefile);

	//test_trained_detector(obj_det_filename, test_dir, videofilename);

	return 0;
}

void test_trained_detector(String obj_det_filename, String test_dir, String videofilename)
{
	//videofilename = "0";
	cout << "Testing trained detector..." << endl;
	HOGDescriptor hog;

	Ptr<SVM> svm;

	int found = obj_det_filename.find("hog");
	if (found == 4) {
		hog.load(obj_det_filename);
	}
	else {
		svm = StatModel::load<SVM>(obj_det_filename);
		hog.setSVMDetector(get_svm_detector(svm));
	}

	vector< String > files;
	glob(test_dir, files);

	int delay = 0;
	VideoCapture cap;

	if (videofilename != "")
	{
		if (videofilename.size() == 1 && isdigit(videofilename[0]))
			cap.open(videofilename[0] - '0');
		else
			cap.open(videofilename);
	}

	obj_det_filename = "testing " + obj_det_filename;
	namedWindow(obj_det_filename, WINDOW_FREERATIO);

	for (size_t i = 0;; i++)
	{
		Mat img;

		if (cap.isOpened())
		{
			cap >> img;
			delay = 1;
		}
		else if (i < files.size())
		{
			img = imread(files[i]);
		}

		if (img.empty())
		{
			return;
		}

		vector< Rect > detections;
		vector< double > foundWeights;
		int colorWeight;
		char cropsave[30];

		hog.detectMultiScale(img, detections, foundWeights, 0.56); // 0 - 1
		for (size_t j = 0; j < detections.size(); j++)
		{
			Mat crop = img(detections[j]);
			sprintf_s(cropsave, "det%3d.png", j);
			//imshow("deteksi ke-" + j, crop);
			//imwrite(cropsave, crop);

			colorWeight = foundWeights[j] * foundWeights[j] * 200;
			Scalar color = Scalar(0, colorWeight, 0);
			rectangle(img, detections[j], color, img.cols / 400 + 1);
		}

		//imshow(obj_det_filename, img);

		if (waitKey(delay) == 27)
		{
			std::cout << "end of video\n";
			waitKey(0);
			return;
		}
	}
}


vector< float > get_svm_detector(const Ptr< SVM >& svm)
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

	vector< float > hog_detector(sv.cols + 1);
	memcpy(&hog_detector[0], sv.ptr(), sv.cols * sizeof(hog_detector[0]));
	hog_detector[sv.cols] = (float)-rho;
	return hog_detector;
}

/*
* Convert training/testing set to be used by OpenCV Machine Learning algorithms.
* TrainData is a matrix of size (#samples x max(#cols,#rows) per samples), in 32FC1.
* Transposition of samples are made if needed.
*/
void convert_to_ml(const vector< Mat >& train_samples, Mat& trainData)
{
	//--Convert data
	const int rows = (int)train_samples.size();
	const int cols = (int)std::max(train_samples[0].cols, train_samples[0].rows);
	Mat tmp(1, cols, CV_32FC1); //< used for transposition if needed
	trainData = Mat(rows, cols, CV_32FC1);

	for (size_t i = 0; i < train_samples.size(); ++i)
	{
		CV_Assert(train_samples[i].cols == 1 || train_samples[i].rows == 1);

		if (train_samples[i].cols == 1)
		{
			transpose(train_samples[i], tmp);
			tmp.copyTo(trainData.row((int)i));
		}
		else if (train_samples[i].rows == 1)
		{
			train_samples[i].copyTo(trainData.row((int)i));
		}
	}
}

void load_images(const String& dirname, vector< Mat >& img_lst, bool showImages = false)
{
	vector< String > files;
	glob(dirname, files);

	for (size_t i = 0; i < files.size(); ++i)
	{
		Mat img = imread(files[i]); // load the image
		if (img.empty())            // invalid image, skip it.
		{
			cout << files[i] << " is invalid!" << endl;
			continue;
		}

		if (showImages)
		{
			imshow("image", img);
			waitKey(1);
		}
		img_lst.push_back(img);
	}
}

void sample_neg(const vector< Mat >& full_neg_lst, vector< Mat >& neg_lst, const Size& size)
{
	Rect box;
	box.width = size.width;
	box.height = size.height;

	const int size_x = box.width;
	const int size_y = box.height;

	srand((unsigned int)time(NULL));

	for (size_t i = 0; i < full_neg_lst.size(); i++)
		if (full_neg_lst[i].cols > box.width && full_neg_lst[i].rows > box.height)
		{
			box.x = rand() % (full_neg_lst[i].cols - size_x);
			box.y = rand() % (full_neg_lst[i].rows - size_y);
			Mat roi = full_neg_lst[i](box);
			neg_lst.push_back(roi.clone());
		}
}

void computeHOGs(const Size wsize, const vector< Mat >& img_lst, vector< Mat >& gradient_lst, bool use_flip, bool visualization, const char* NegOrPos)
{
	HOGDescriptor hog;
	hog.winSize = wsize;
	Mat gray;
	Mat hogdesc_visu;
	vector< float > descriptors;
	char fileimg[20];

	for (size_t i = 0; i < img_lst.size(); i++)
	{
		if (img_lst[i].cols >= wsize.width && img_lst[i].rows >= wsize.height)
		{
			Rect r = Rect((img_lst[i].cols - wsize.width) / 2,
				(img_lst[i].rows - wsize.height) / 2,
				wsize.width,
				wsize.height);
			Mat img = img_lst[i](r).clone();
			cvtColor(img_lst[i](r), gray, COLOR_BGR2GRAY);
			hog.compute(gray, descriptors, Size(8, 8), Size(0, 0));
			gradient_lst.push_back(Mat(descriptors).clone());
			if (use_flip)
			{
				flip(gray, gray, 1);
				hog.compute(gray, descriptors, Size(8, 8), Size(0, 0));
				gradient_lst.push_back(Mat(descriptors).clone());
			}
			hogdesc_visu = get_hogdescriptor_visu(img, descriptors, wsize);
			sprintf_s(fileimg, "%s/file%04d.jpg", NegOrPos, i);

			if (visualization) {
				imshow("gradient", hogdesc_visu);
				waitKey(10);
			}
			if (dirExists(NegOrPos)) {
				imwrite(fileimg, hogdesc_visu);
			}
		}
	}
}

int dirExists(const char* path)
{
	struct stat info;

	if (stat(path, &info) != 0)
		return 0;
	else if (info.st_mode & S_IFDIR)
		return 1;
	else
		return 0;
}

Mat get_hogdescriptor_visu(const Mat& color_origImg, vector<float>& descriptorValues, const Size& size)
{
	const int DIMX = size.width;
	const int DIMY = size.height;
	float zoomFac = 3;
	Mat visu;
	resize(color_origImg, visu, Size((int)(color_origImg.cols * zoomFac), (int)(color_origImg.rows * zoomFac)));

	int cellSize = 8;
	int gradientBinSize = 9;
	float radRangeForOneBin = (float)(CV_PI / (float)gradientBinSize); // dividing 180� into 9 bins, how large (in rad) is one bin?

	// prepare data structure: 9 orientation / gradient strenghts for each cell
	int cells_in_x_dir = DIMX / cellSize;
	int cells_in_y_dir = DIMY / cellSize;
	float*** gradientStrengths = new float** [cells_in_y_dir];
	int** cellUpdateCounter = new int* [cells_in_y_dir];
	for (int y = 0; y < cells_in_y_dir; y++)
	{
		gradientStrengths[y] = new float* [cells_in_x_dir];
		cellUpdateCounter[y] = new int[cells_in_x_dir];
		for (int x = 0; x < cells_in_x_dir; x++)
		{
			gradientStrengths[y][x] = new float[gradientBinSize];
			cellUpdateCounter[y][x] = 0;

			for (int bin = 0; bin < gradientBinSize; bin++)
				gradientStrengths[y][x][bin] = 0.0;
		}
	}

	// nr of blocks = nr of cells - 1
	// since there is a new block on each cell (overlapping blocks!) but the last one
	int blocks_in_x_dir = cells_in_x_dir - 1;
	int blocks_in_y_dir = cells_in_y_dir - 1;

	// compute gradient strengths per cell
	int descriptorDataIdx = 0;
	int cellx = 0;
	int celly = 0;

	for (int blockx = 0; blockx < blocks_in_x_dir; blockx++)
	{
		for (int blocky = 0; blocky < blocks_in_y_dir; blocky++)
		{
			// 4 cells per block ...
			for (int cellNr = 0; cellNr < 4; cellNr++)
			{
				// compute corresponding cell nr
				cellx = blockx;
				celly = blocky;
				if (cellNr == 1) celly++;
				if (cellNr == 2) cellx++;
				if (cellNr == 3)
				{
					cellx++;
					celly++;
				}

				for (int bin = 0; bin < gradientBinSize; bin++)
				{
					float gradientStrength = descriptorValues[descriptorDataIdx];
					descriptorDataIdx++;

					gradientStrengths[celly][cellx][bin] += gradientStrength;

				} // for (all bins)


				// note: overlapping blocks lead to multiple updates of this sum!
				// we therefore keep track how often a cell was updated,
				// to compute average gradient strengths
				cellUpdateCounter[celly][cellx]++;

			} // for (all cells)


		} // for (all block x pos)
	} // for (all block y pos)


	// compute average gradient strengths
	for (celly = 0; celly < cells_in_y_dir; celly++)
	{
		for (cellx = 0; cellx < cells_in_x_dir; cellx++)
		{

			float NrUpdatesForThisCell = (float)cellUpdateCounter[celly][cellx];

			// compute average gradient strenghts for each gradient bin direction
			for (int bin = 0; bin < gradientBinSize; bin++)
			{
				gradientStrengths[celly][cellx][bin] /= NrUpdatesForThisCell;
			}
		}
	}

	// draw cells
	for (celly = 0; celly < cells_in_y_dir; celly++)
	{
		for (cellx = 0; cellx < cells_in_x_dir; cellx++)
		{
			int drawX = cellx * cellSize;
			int drawY = celly * cellSize;

			int mx = drawX + cellSize / 2;
			int my = drawY + cellSize / 2;

			//rectangle(visu, Point((int)(drawX*zoomFac), (int)(drawY*zoomFac)), Point((int)((drawX + cellSize)*zoomFac), (int)((drawY + cellSize)*zoomFac)), Scalar(100, 100, 100), 1);

			// draw in each cell all 9 gradient strengths
			for (int bin = 0; bin < gradientBinSize; bin++)
			{
				float currentGradStrength = gradientStrengths[celly][cellx][bin];

				// no line to draw?
				if (currentGradStrength == 0)
					continue;

				float currRad = bin * radRangeForOneBin + radRangeForOneBin / 2;

				float dirVecX = sin(currRad);
				float dirVecY = cos(currRad);
				float maxVecLen = (float)(cellSize / 2.f);
				float scale = 2.5; // just a visualization scale, to see the lines better

				// compute line coordinates
				float x1 = mx - dirVecX * currentGradStrength * maxVecLen * scale;
				float y1 = my - dirVecY * currentGradStrength * maxVecLen * scale;
				float x2 = mx + dirVecX * currentGradStrength * maxVecLen * scale;
				float y2 = my + dirVecY * currentGradStrength * maxVecLen * scale;

				// draw gradient visualization
				line(visu, Point((int)(x2 * zoomFac), (int)(y2 * zoomFac)), Point((int)(x1 * zoomFac), (int)(y1 * zoomFac)), Scalar(0, 255, 0), 1);

			} // for (all bins)

		} // for (cellx)
	} // for (celly)


	// don't forget to free memory allocated by helper data structures!
	for (int y = 0; y < cells_in_y_dir; y++)
	{
		for (int x = 0; x < cells_in_x_dir; x++)
		{
			delete[] gradientStrengths[y][x];
		}
		delete[] gradientStrengths[y];
		delete[] cellUpdateCounter[y];
	}
	delete[] gradientStrengths;
	delete[] cellUpdateCounter;

	return visu;

} // get_hogdescriptor_visu