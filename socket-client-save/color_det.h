#pragma once
#include <iostream>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>
#include <windows.h>

//#define TEMP_CROP "E:/dataset/PascalDataSetCropped/pascal_voc_2.jpg"
//#define TEMP_CROP "C:/Users/hp/source/repos/Thesis/hog_online/pascal_voc_0.jpg"
//#define TEMP_CROP "E:/dataset/WIN_20190419_21_06_42_Pro.jpg_crop.jpg"

using namespace cv;
using namespace std;
using namespace cv::ml;

class color_det
{
private:
	string nameimg;
	vector< Mat > bbox, bbox_crop, hist_crop;
	Mat img_detcolor, img_temp_primary, img_temp_secondary;
	Rect loc_color;
	int bins, iterator = 0;;
	bool imgsave, _run = false;

	//processing time check
	unsigned long AAtime = 0, BBtime = 0;
	double s_time, fps_time;

	int maxElementIndex = 0;
	int maxElement = 0;
public:
	Mat compute_colordet(Mat &img, vector<Rect> locations);
	int compute_colordet_weight(Mat &img, vector<Rect> locations, Mat img_temp, vector <double> &all_number);
	void colordet(Mat &img, vector<Rect> locations, int bin, bool saveimg);

	vector<Mat> crop_img(vector<Mat> & bbox);

	vector<Mat> crop_hist(vector<Mat> & bbox_crop, bool showImages);

	int comp_hist(vector<Mat> & hist_crop);
	int comp_hist_weight(vector<Mat> & hist_crop, vector <double> &all_number);

	void draw_locations_color(cv::Mat & img, const Rect locations, const Scalar color);
	void draw_locations(cv::Mat & img, const vector<Rect>& locations, const Scalar & color, int index);
	string parse_namefile(String line);
	void clear();

	void set_img(String str_image);

	double get_s_time();
	double get_fps_time();
	Rect get_loc_color();
	int get_maxindex();
	int get_maxvalue();

	void draw_img(cv::Mat &img, vector<Rect> locations);
};

void color_det::colordet(Mat &img, vector<Rect> locations, int bin, bool saveimg)
{
	bins = bin;
	imgsave = saveimg;

	//to calculate fps
	AAtime = getTickCount();

	if (_run == false || iterator == 0) {
		img_temp_secondary = compute_colordet(img, locations);
		_run = true;
	}
	else {
		vector <double> weight_primary, weight_secondary;
		compute_colordet_weight(img, locations, img_temp_primary, weight_primary);
		compute_colordet_weight(img, locations, img_temp_secondary, weight_secondary);

		vector <double> weighted_all;
		double weighted;
		for (size_t i = 0; i < locations.size(); i++) {
			weighted = 0.3*weight_primary[i] + 0.7*weight_secondary[i];
			weighted_all.push_back(weighted);
		}
		maxElementIndex = max_element(weighted_all.begin(), weighted_all.end()) - weighted_all.begin();
		maxElement = *max_element(weighted_all.begin(), weighted_all.end());
		cout << "max: " << maxElement << " ";
	}

	iterator++;

	if (iterator == 20) {
		iterator = 0;
	}

	//to calculate fps
	BBtime = getTickCount();
	s_time = (BBtime - AAtime) / getTickFrequency();
	fps_time = 1 / s_time;
}

Mat color_det::compute_colordet(Mat &img, vector<Rect> locations)
{
	for (size_t j = 0; j < locations.size(); j++)
	{
		bbox.push_back(img(locations[j]));
	}
	bbox.insert(bbox.begin(), img_temp_primary);

	//*** Crop bounding box into 3x3 region ***
	bbox_crop = crop_img(bbox);

	//*** Calculate Color Histogram ***
	hist_crop = crop_hist(bbox_crop, false);

	//*** Find minimum index form Color Histogram***
	int index_crop = comp_hist(hist_crop);

	loc_color = locations[(index_crop - 1)];

	draw_locations_color(img, loc_color, Scalar(76, 0, 153));
	draw_locations(img, locations, Scalar(0, 255, 0), index_crop - 1);

	clear();

	return img(loc_color);
}

int color_det::compute_colordet_weight(Mat &img, vector<Rect> locations, Mat img_temp, vector <double> &all_number)
{
	for (size_t j = 0; j < locations.size(); j++)
	{
		bbox.push_back(img(locations[j]));
	}
	bbox.insert(bbox.begin(), img_temp);

	//*** Crop bounding box into 3x3 region ***
	bbox_crop = crop_img(bbox);

	//*** Calculate Color Histogram ***
	hist_crop = crop_hist(bbox_crop, false);

	//*** Find minimum index form Color Histogram***
	int index_crop = comp_hist_weight(hist_crop, all_number) - 1;

	loc_color = locations[index_crop];

	clear();

	return index_crop;
}

void color_det::clear() {
	bbox.clear();
	bbox_crop.clear();
	hist_crop.clear();
}

void color_det::draw_img(cv::Mat &img, vector<Rect> locations) {
	draw_locations_color(img, locations[maxElementIndex], Scalar(76, 0, 153));
	draw_locations(img, locations, Scalar(0, 255, 0), maxElementIndex);
}

void color_det::draw_locations_color(cv::Mat & img, const Rect locations, const Scalar color)
{
	//char textOnImg[15] = "color_det";
	rectangle(img, locations, color, 2);
	//putText(img,
	//	textOnImg,
	//	Point(locations.x + 2, locations.y + 11), // Coordinates
	//	FONT_HERSHEY_COMPLEX_SMALL, // Font
	//	1, // Scale. 2.0 = 2x bigger
	//	Scalar(255, 255, 255), // BGR Color
	//	2, // Line Thickness (Optional)
	//	CV_AA); // Anti-alias (Optional)

}

void color_det::draw_locations(cv::Mat & img, const vector<Rect>& locations, const Scalar & color, int index)
{
	for (size_t j = 0; j < locations.size(); j++)
	{
		if (index == j)
			continue;
		rectangle(img, locations[j], color, 2);
	}

}

vector<Mat> color_det::crop_img(vector<Mat> & bbox) {
	vector<Mat> bbox_crop;
	for (size_t j = 0; j < bbox.size(); j++) {
		Mat img = bbox[j];
		Rect cropImg(Point(img.size().width / 3, img.size().height * 2 / 3), Size(img.size().width / 3, img.size().height / 3));

		Mat cropImg_ = img(cropImg);
		resize(cropImg_, cropImg_, cv::Size(150, 200));
		bbox_crop.push_back(cropImg_);

		if (imgsave) {
			char namewin[50];
			sprintf_s(namewin, "ori_%d.jpg", j);
			cout << namewin << endl;
			imwrite(namewin, img);

		}
	}
	return bbox_crop;
}

vector<Mat> color_det::crop_hist(vector<Mat> & bbox_crop, bool showImages = false) {
	//Mat hsv, hist;
	int h_bins = bins;
	int s_bins = bins;
	int histSize[] = { h_bins, s_bins }, channels[] = { 0, 1 };
	float h_ranges[] = { 0, 180 }; // hue range is [0,180]
	float s_ranges[] = { 0, 255 };
	const float* ranges[] = { h_ranges, s_ranges };
	int scale = 10;
	vector<Mat> hsv(bbox_crop.size()), hist(bbox_crop.size()), hist_img(bbox_crop.size());

	for (size_t i = 0; i < bbox_crop.size(); ++i) {
		//imshow("gambar", bbox_crop[i]); waitKey(0);
		cvtColor(bbox_crop[i], hsv[i], COLOR_BGR2HSV);
		calcHist(&hsv[i], 1, channels, noArray(), hist[i], 2, histSize, ranges, true);
		normalize(hist[i], hist[i], 0, 255, NORM_MINMAX);
		hist_img[i] = Mat::zeros(histSize[0] * scale, histSize[1] * scale, CV_8UC3);

		// Draw our histogram For the 5 images
			//
		for (int h = 0; h < histSize[0]; h++)
			for (int s = 0; s < histSize[1]; s++) {
				float hval = hist[i].at<float>(h, s);
				rectangle(
					hist_img[i],
					Rect(h*scale, s*scale, scale, scale),
					Scalar::all(hval),
					-1
				);
			}
	}

	if (showImages) {
		char namewin[20];
		for (size_t i = 0; i < bbox_crop.size(); ++i) {
			//sprintf_s(namewin, "Source%d", i);
			//namedWindow(namewin, CV_WINDOW_FREERATIO); imshow(namewin, src[i]);
			sprintf_s(namewin, "HS Histogram%d", i);
			namedWindow(namewin, 1); imshow(namewin, hist_img[i]);
			sprintf_s(namewin, "crop_img%d", i);
			namedWindow(namewin, WINDOW_FREERATIO); imshow(namewin, bbox_crop[i]);
		}
		waitKey(0);
	}

	if (imgsave) {
		char namewin[50];
		for (size_t i = 0; i < bbox_crop.size(); ++i) {
			sprintf_s(namewin, "hist_%d.jpg", i);
			imwrite(namewin, hist_img[i]);
			sprintf_s(namewin, "crop_%d.jpg", i);
			imwrite(namewin, bbox_crop[i]);
		}
	}

	hist.resize(bbox_crop.size());
	return hist;
}

int color_det::comp_hist(vector<Mat> & hist_crop) {
	// Compare the histogram
	double number, maximum = -1;
	vector <double> all_number;
	int maximumIndex = 0;

	for (size_t i = 1; i < hist_crop.size(); ++i) { // For each histogram
		number = compareHist(hist_crop[0], hist_crop[i], HISTCMP_INTERSECT);
		if (number > maximum) {
			//cout << number << endl;
			maximum = number;
			maximumIndex = i;
		}
		all_number.push_back(number);
	}

	if (imgsave) {
		ofstream savetxt;
		char namefile[30];
		sprintf_s(namefile, "%d_.txt", bins);
		savetxt.open(namefile);
		for (size_t i = 0; i < all_number.size(); i++) {
			savetxt << all_number[i] << endl;
		}
		savetxt.close();
	}

	return maximumIndex;
}

int color_det::comp_hist_weight(vector<Mat> & hist_crop, vector <double> &all_number) {
	// Compare the histogram
	double number, maximum = -1;
	int maximumIndex = 0;
	all_number.clear();

	for (size_t i = 1; i < hist_crop.size(); ++i) { // For each histogram
		number = compareHist(hist_crop[0], hist_crop[i], HISTCMP_INTERSECT);
		if (number > maximum) {
			//cout << number << endl;
			maximum = number;
			maximumIndex = i;
		}
		all_number.push_back(number);
	}

	if (imgsave) {
		ofstream savetxt;
		char namefile[30];
		sprintf_s(namefile, "%d_.txt", bins);
		savetxt.open(namefile);
		for (size_t i = 0; i < all_number.size(); i++) {
			savetxt << all_number[i] << endl;
		}
		savetxt.close();
	}

	return maximumIndex;
}

void color_det::set_img(String str_image) {
	nameimg = parse_namefile(str_image);
	img_temp_primary = imread(str_image, 1);
}

double color_det::get_s_time() {
	return s_time * 1000;
}

double color_det::get_fps_time() {
	return fps_time;
}

Rect color_det::get_loc_color() {
	return loc_color;
}

string color_det::parse_namefile(String line) {
	// Vector of string to save tokens 
	vector <string> tokens;

	// stringstream class check1 
	stringstream check1(line);

	string intermediate;

	// Tokenizing w.r.t. space ' ' 
	while (getline(check1, intermediate, '.'))
	{
		tokens.push_back(intermediate);
	}

	return tokens[0];
}

int color_det::get_maxindex() {
	return maxElementIndex;
}

int color_det::get_maxvalue() {
	return maxElement;
}