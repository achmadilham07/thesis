#include <iostream>
#include <string>
#include <WS2tcpip.h>
#include <windows.h>
#include "hog.h"
#include "color_det.h"
#include "projection.h"
#include "kalman.h"
#include "calculateRadian.h"
#pragma comment(lib, "ws2_32.lib")

#define TEMP_CROP "file_temp.jpg"
#define WIDTH 640.0
#define HEIGHT 480.0
#define CX 600.0
#define CY 40.0
#define RED Scalar(0, 0, 255)
#define GREEN Scalar(0, 255, 0)
#define BLUE Scalar(255, 0, 0)
#define PURPLE Scalar(255, 0, 255)
#define WHITE Scalar(255, 255, 255)
#define R 30.0

using namespace std;
bool file_exists(const string & file);
bool splitBool(char* buf, int valread);
void drawOrientation(Mat &draw, double theta, int addY);
int main(int argc, char** argv)
{
	const char* keys =
	{
		"{help h|     | show help message}"
		"{iv    |     | Directory video(current folder or spesific dir)}"
		"{t     |     | Directory trained SVM (current folder or spesific dir)}"
		"{idc   |     | ID Client ID00}"
		"{th    |     | threshold on HOG Descriptor}"
		"{rsz   |     | resize on HOG Descriptor}"
		"{cd    |     | file_temp.jpg become template}"
		"{ip    |     | IP Server}"
		"{sv    |     | save video}"
		"{re    |     | report all data(coordinates and computation time_hog)}"
		"{kf1   |     | activate kalman filter bounding box)}"
		"{kf2   |     | activate kalman filter coordinate after camera modelling)}"
		"{kf12  |     | activate a combination kalman filter 1 and 2)}"
	};

	CommandLineParser parser(argc, argv, keys);

	if (parser.has("help"))
	{
		parser.printMessage();
		exit(0);
	}

	String VIDEO_FILE = parser.get< String >("iv");
	String TRAINED_SVM = parser.get< String >("t");
	string IDClient = parser.get< String >("idc");
	String ipAddress = parser.get< String >("ip");			// IP Address of the server
	double thres = parser.get< double >("th");
	double resize = parser.get< double >("rsz");
	bool COLOR_DET = parser.get< bool >("cd");
	bool SAVE_VID = parser.get< bool >("sv");
	bool REPORT = parser.get< bool >("re");
	bool KF1 = parser.get< bool >("kf1");
	bool KF2 = parser.get< bool >("kf2");
	bool KF12 = parser.get< bool >("kf12");

	VideoCapture video;
	VideoWriter videowriteOri, videowriteDrw;
	Mat img, draw, drawhaar;
	ofstream coord_hog, hog_kf1, hog_kf2, hog_kf12, time_hog, time_cs;
	ofstream time_hog_kf1, time_hog_kf2, time_hog_kf12, time_kf0;

	char key = 27;
	int i = 0;
	char namefile[70] = "";
	char namefldr[20] = "";

	if (REPORT) {
		cout << "REPORT" << endl;
		for (int i = 0; i < 100; i++) {
			sprintf_s(namefldr, "%s_%02d", IDClient.c_str(), i);
			if (file_exists(namefldr))
				continue;
			else {
				CreateDirectory(namefldr, NULL);
				sprintf_s(namefile, "%s/%s_hog.txt", namefldr, IDClient.c_str());
				coord_hog.open(namefile);
				sprintf_s(namefile, "%s/%s_hog_kf1.txt", namefldr, IDClient.c_str());
				hog_kf1.open(namefile);
				sprintf_s(namefile, "%s/%s_hog_kf2.txt", namefldr, IDClient.c_str());
				hog_kf2.open(namefile);
				sprintf_s(namefile, "%s/%s_hog_kf12.txt", namefldr, IDClient.c_str());
				hog_kf12.open(namefile);
				sprintf_s(namefile, "%s/%s_time.txt", namefldr, IDClient.c_str());
				time_hog.open(namefile);
				sprintf_s(namefile, "%s/%s_timecs.txt", namefldr, IDClient.c_str());
				time_cs.open(namefile);

				sprintf_s(namefile, "%s/%s_time_kf0.txt", namefldr, IDClient.c_str());
				time_kf0.open(namefile);
				sprintf_s(namefile, "%s/%s_time_hog_kf1.txt", namefldr, IDClient.c_str());
				time_hog_kf1.open(namefile);
				sprintf_s(namefile, "%s/%s_time_hog_kf2.txt", namefldr, IDClient.c_str());
				time_hog_kf2.open(namefile);
				sprintf_s(namefile, "%s/%s_time_hog_kf12.txt", namefldr, IDClient.c_str());
				time_hog_kf12.open(namefile);
				break;
			}
		}
	}

	double vp_x = 339.921;			// vanishing point X (pixel)
	double vp_y = 197.72;			// vanishing point Y (pixel)
	double c_x = 320;				// horizontal center point (pixel)
	double c_y = 240;				// vertical center point (pixel)
	double fc = 728.3926;			// focal length (pixel)
	double yc = 1500;				// height of camera (milimeter)
	double uc = c_x;				//

	hog hogdescriptor(TRAINED_SVM);
	hogdescriptor.set_threshold(thres);		// hit Threshold
	hogdescriptor.set_resized(resize);			// change size 0.5 times from the original

	color_det detColor_hog, detColor_haar;
	detColor_hog.set_img(TEMP_CROP);

	projection cam_model_hog, cam_model_haar;
	cam_model_hog.set_h(1.9);
	cam_model_hog.set_fc(fc);
	cam_model_hog.set_yc(yc);
	cam_model_hog.set_uc(uc);
	cam_model_hog.set_v0(vp_y);

	kalman kfPos1, kfPos2, kfPos12, kfPos22;
	kfPos1.kalmanPos1_init();
	kfPos12.kalmanPos1_init();
	kfPos2.kalmanPos2_init();
	kfPos22.kalmanPos2_init();

	namedWindow("all detect", WINDOW_FREERATIO);

	if (VIDEO_FILE != "")
	{
		if (VIDEO_FILE.size() == 1 && isdigit(VIDEO_FILE[0]))
			video.open(VIDEO_FILE[0] - '0');
		else
			video.open(VIDEO_FILE);
	}

	if (SAVE_VID) {
		std::string delimiter = ".";
		std::string token = VIDEO_FILE.substr(0, VIDEO_FILE.find(delimiter));
		int frame_width = video.get(CAP_PROP_FRAME_WIDTH);
		int frame_height = video.get(CAP_PROP_FRAME_HEIGHT);
		for (int i = 0; i < 100; i++) {
			sprintf_s(namefile, "%s/%s_video_ori.avi", namefldr, IDClient.c_str());
			if (file_exists(namefile))
				continue;
			else {
				videowriteOri.open(namefile, VideoWriter::fourcc('M', 'J', 'P', 'G'), 10, Size(frame_width, frame_height));
				sprintf_s(namefile, "%s/%s_video_drw.avi", namefldr, IDClient.c_str());
				videowriteDrw.open(namefile, VideoWriter::fourcc('M', 'J', 'P', 'G'), 10, Size(frame_width, frame_height));
				break;
			}
		}

	}
	memset(namefile, 0, sizeof(namefile));

	//processing time_hog check
	unsigned long AAtime = 0, BBtime = 0, time1 = 0, time2 = 0;
	unsigned long CSA = 0, CSB = 0, CSC = 0, CSD = 0;
	double s_time = 0, s_time_hog = 0;
	int iteration = 0;
	double time_tot = 0, fps_tot = 0;
	int baz[5] = { };
	vector<double> posX_hog, posY_hog, posX_kf1, posY_kf1, posX_kf2, posY_kf2, posX_kf12, posY_kf12;
	double posX_last = 0, posY_last = 0;
	double theta_hog = 0, theta_kf1 = 0, theta_kf2 = 0, theta_kf12 = 0, sinT, cosT;
	double dist_kf1 = 0, dist_det = 0;
	bool after_hog = false, after_kf1 = false, after_kf2 = false, after_kf12 = false;
	bool end_of_process = false;
	bool checkClientConnect = true;
	bool checkClientActive = false;
	char textOnImg[15];

	int port = 54000;						// Listening port # on the server

	// Initialize WinSock
	WSAData data;
	WORD ver = MAKEWORD(2, 2);
	int wsResult = WSAStartup(ver, &data);
	if (wsResult != 0)
	{
		cerr << "Can't start Winsock, Err #" << wsResult << endl;
		return 0;
	}

	// Create socket
	SOCKET sock = socket(AF_INET, SOCK_STREAM, 0);
	if (sock == INVALID_SOCKET)
	{
		cerr << "Can't create socket, Err #" << WSAGetLastError() << endl;
		WSACleanup();
		return 0;
	}

	// Fill in a hint structure
	sockaddr_in hint;
	hint.sin_family = AF_INET;
	hint.sin_port = htons(port);
	inet_pton(AF_INET, ipAddress.c_str(), &hint.sin_addr);

	// Do-while loop to send and receive data
	char buf[4096];
	string userInput;

	// Connect to server
	int connResult = connect(sock, (sockaddr*)&hint, sizeof(hint));
	if (connResult == SOCKET_ERROR)
	{
		cerr << "Can't connect to server, Err #" << WSAGetLastError() << endl;
		closesocket(sock);
		WSACleanup();
		return 0;
	}
	else {
		//cout << "masuk sini looo.." << endl;
		bool status = true;
		while (status) {
			int sendResult;
			//cout << "masuk sini guys.. " << IDClient.c_str() << endl;
			if ((sendResult = send(sock, IDClient.c_str(), strlen(IDClient.c_str()) + 1, 0)) != SOCKET_ERROR)
			{
				cout << "masuk sini lo guys.. " << IDClient.c_str() << endl;
				ZeroMemory(buf, 4096);
				int bytesReceived;
				if ((bytesReceived = recv(sock, buf, 4096, 0)) != 0) {
					checkClientActive = splitBool(buf, bytesReceived);
					cout << "oke guys.. " << checkClientActive << endl;
					status = false;
				}
			}
		}
	}

	while (checkClientConnect)
	{
		while (checkClientActive)
		{
			cout << "Client> ";
			video >> img;
			if (img.empty()) {
				checkClientActive = false;
				break;
			}

			AAtime = getTickCount();

			draw = img.clone();
			drawhaar = img.clone();

			hogdescriptor.set_img(img);
			hogdescriptor.computeHOG();

			vector< Rect > loc = hogdescriptor.get_locations();

			if (loc.size() != 0) {
				if (COLOR_DET) {
					detColor_hog.colordet(draw, loc, 32, false);
					detColor_hog.draw_img(draw, loc);

					cam_model_hog.compute_projection(detColor_hog.get_loc_color(), false);
					posX_hog.push_back(cam_model_hog.get_coord_x());
					posY_hog.push_back(cam_model_hog.get_coord_y());

					theta_hog = calculateRadian(posX_hog[0], posY_hog[0], posX_hog[1], posY_hog[1]);

					if (after_hog) {
						posX_hog.erase(posX_hog.begin());
						posY_hog.erase(posY_hog.begin());
					}

					if (REPORT) {
						//cout << iteration << endl;
						time_hog << iteration << "\t" << hogdescriptor.get_s_time() << "\t" << detColor_hog.get_s_time() << "\t" << cam_model_hog.get_s_time() << endl;
						coord_hog << iteration << "\t" << posX_hog[0] << "\t" << posY_hog[0] << "\t" << theta_hog << endl;
					}
				}
				else {
					hogdescriptor.draw_locations(draw, loc, Scalar(0, 255, 0));
				}
				time2 = getTickCount();
				s_time_hog = (time2 - AAtime) / getTickFrequency();
				time_kf0 << iteration << "\t" << s_time_hog << endl;
				after_hog = true;
				drawOrientation(draw, theta_hog, 0);
			}

			if (KF1 && after_hog) {
				time1 = getTickCount();
				if (kfPos1.found1) {
					kfPos1.setA1(s_time * 1000);
					kfPos1.predictPos1();
					kfPos1.drawRect(draw, kfPos1.getRect());
					cam_model_hog.compute_projection(kfPos1.getRect(), false);

					posX_kf1.push_back(cam_model_hog.get_coord_x());
					posY_kf1.push_back(cam_model_hog.get_coord_y());

					theta_kf1 = calculateRadian(posX_kf1[0], posY_kf1[0], posX_kf1[1], posY_kf1[1]);

					if (after_kf1) {
						posX_kf1.erase(posX_kf1.begin());
						posY_kf1.erase(posY_kf1.begin());
					}

					if (REPORT) {
						hog_kf1 << iteration << "\t" << posX_kf1[0] << "\t" << posY_kf1[0] << "\t" << theta_kf1 << endl;
					}
				}
				else {
					posX_kf1.push_back(cam_model_hog.get_coord_x());
					posY_kf1.push_back(cam_model_hog.get_coord_y());
				}

				if ((iteration % 1) == 0) {
					if (loc.size() != 0) {
						kfPos1.updatePos1(detColor_hog.get_loc_color());
					}
					else {
						kfPos1.updatePos1(kfPos1.getRect());
					}

				}
				time2 = getTickCount();
				double time = s_time_hog + ((time2 - time1) / getTickFrequency());
				time_hog_kf1 << iteration << "\t" << time << endl;
				after_kf1 = true;
				drawOrientation(draw, theta_kf1, 100);
			}

			if (KF2 && after_hog) {
				if (kfPos2.found2) {
					kfPos2.setA2(s_time * 1000);
					kfPos2.predictPos2();

					posX_kf2.push_back(kfPos2.getPos2x());
					posY_kf2.push_back(kfPos2.getPos2y());

					theta_kf2 = calculateRadian(posX_kf2[0], posY_kf2[0], posX_kf2[1], posY_kf2[1]);

					if (after_kf2) {
						posX_kf2.erase(posX_kf2.begin());
						posY_kf2.erase(posY_kf2.begin());
					}

					if (REPORT) {
						hog_kf2 << iteration << "\t" << posX_kf2[0] << "\t" << posY_kf2[0] << "\t" << theta_kf2 << endl;
					}
				}
				else {
					posX_kf2.push_back(cam_model_hog.get_coord_x());
					posY_kf2.push_back(cam_model_hog.get_coord_y());
				}

				if ((iteration % 1) == 0) {
					if (loc.size() != 0) {
						kfPos2.updatePos2(posX_hog[0], posY_hog[0]);
					}
					else {
						kfPos2.updatePos2(posX_kf2[0], posY_kf2[0]);
					}
				}
				time2 = getTickCount();
				double time = s_time_hog + ((time2 - time1) / getTickFrequency());
				time_hog_kf2 << iteration << "\t" << time << endl;
				after_kf2 = true;
				drawOrientation(draw, theta_kf2, 200);
			}

			if (KF12 && after_hog) {
				if (kfPos12.found1) {
					kfPos12.setA1(s_time * 1000);
					kfPos12.predictPos1();
					kfPos12.drawRect(draw, kfPos12.getRect());
					cam_model_hog.compute_projection(kfPos12.getRect(), false);

					kfPos22.setA2(s_time * 1000);
					kfPos22.predictPos2();

					posX_kf12.push_back(kfPos22.getPos2x());
					posY_kf12.push_back(kfPos22.getPos2y());

					theta_kf12 = calculateRadian(posX_kf12[0], posY_kf12[0], posX_kf12[1], posY_kf12[1]);
					//printf("\t%5.2f \t%5.2f \t%7.2f ", posX_kf12[0], posY_kf12[0], theta_kf12);

					if (after_kf12) {
						posX_kf12.erase(posX_kf12.begin());
						posY_kf12.erase(posY_kf12.begin());
					}

					if (REPORT) {
						//cout << iteration << endl;
						hog_kf12 << iteration << "\t" << posX_kf12[0] << "\t" << posY_kf12[0] << "\t" << theta_kf12 << endl;
					}
				}
				else {
					posX_kf12.push_back(cam_model_hog.get_coord_x());
					posY_kf12.push_back(cam_model_hog.get_coord_y());
				}

				if ((iteration % 1) == 0) {
					if (loc.size() != 0) {
						kfPos12.updatePos1(detColor_hog.get_loc_color());
						kfPos22.updatePos2(cam_model_hog.get_coord_x(), cam_model_hog.get_coord_y());
					}
					else {
						kfPos12.updatePos1(kfPos12.getRect());
						kfPos22.updatePos2(posX_kf12[0], posY_kf12[0]);
					}

				}
				time2 = getTickCount();
				double time = s_time_hog + ((time2 - time1) / getTickFrequency());
				time_hog_kf12 << iteration << "\t" << time << endl;
				after_kf12 = true;
				drawOrientation(draw, theta_kf12, 300);
			}

			BBtime = getTickCount();
			s_time = (BBtime - AAtime) / getTickFrequency();
			double fps_time = 1 / s_time;
			printf("\t%2.2f\t", fps_time);

			sprintf_s(textOnImg, "#%05d ", iteration);
			putText(draw, textOnImg, Point(CY, HEIGHT - CY), FONT_HERSHEY_DUPLEX, 0.6, WHITE);
			sprintf_s(textOnImg, "#fps: %3.0f", fps_time);
			putText(draw, textOnImg, Point(CY, CY), FONT_HERSHEY_DUPLEX, 0.6, WHITE);

			if (after_hog)
				sprintf_s(namefile, "$%s_%.2f_%.2f#", IDClient.c_str(), posX_hog[0], posY_hog[0]);

			CSA = getTickCount();
			if (strlen(namefile) != 0) {
				printf("%s\t", namefile);
				int sendResult = send(sock, namefile, strlen(namefile) + 1, 0);
				memset(namefile, 0, sizeof(namefile));
				if (sendResult != SOCKET_ERROR)
				{
					CSB = getTickCount();
					// Wait for response
					ZeroMemory(buf, 4096);
					int bytesReceived = recv(sock, buf, 4096, 0);
					if (bytesReceived != 0)
					{
						CSC = getTickCount();
						// Echo response to console
						checkClientActive = splitBool(buf, bytesReceived);
						printf("%d\t", checkClientActive);
					}
				}
			}
			CSD = getTickCount();
			double AB = (CSB - CSA) / getTickFrequency();
			double BC = (CSC - CSB) / getTickFrequency();
			double CD = (CSD - CSC) / getTickFrequency();
			time_cs << iteration << "\t" << AB << "\t" << BC << "\t" << CD << endl;

			imshow("all detect", draw);

			if (SAVE_VID) {
				videowriteOri.write(drawhaar);
				videowriteDrw.write(draw);
			}

			iteration++;
			key = (char)waitKey(1);
			if (27 == key) {
				checkClientActive = false;
				//checkClientConnect = false;
			}
			printf("\n");
		}

		//if (!checkClientConnect) {
		//	sprintf_s(buf, "", "");
		//	send(sock, buf, strlen(buf), 0);
		//	checkClientConnect = true;
		//	cout << "Client> button clicked" << endl;
		//}
		cout << "keluar guys.. " << checkClientActive << endl;

		if (27 == key) {
			break;
		}

		ZeroMemory(buf, 4096);
		int bytesReceived;
		if ((bytesReceived = recv(sock, buf, 4096, 0)) != 0) {
			checkClientActive = splitBool(buf, bytesReceived);
			cout << "oke guys.. " << checkClientActive << endl;
		}
	}

	if (REPORT) {
		coord_hog.close();
		hog_kf1.close();
		hog_kf2.close();
		hog_kf12.close();
		time_hog.close();
		time_cs.close();
	}
	if (SAVE_VID) {
		videowriteOri.release();
		videowriteDrw.release();
	}
	cv::destroyAllWindows();

	// Gracefully close down everything
	closesocket(sock);
	WSACleanup();

	return 0;
}

bool splitBool(char* buf, int valread) {
	buf[valread] = '\0';
	string text = string(buf, 0, valread);
	string delimiter = " $_#";
	vector<string> arrText;
	char *nextText = NULL;

	char* c = const_cast<char*>(text.c_str());

	char *token = strtok_s(c, delimiter.c_str(), &nextText);
	while (token != NULL)
	{
		arrText.push_back(token);
		token = strtok_s(NULL, delimiter.c_str(), &nextText);
	}

	string IDClient = arrText[0];
	return stoi(arrText[1]);
}

bool file_exists(const string & file)
{
	return access(file.c_str(), 0) == 0;
}

void drawOrientation(Mat &draw, double theta, int addY) {
	double sinT = sin(theta / 180 * PI);
	double cosT = cos(theta / 180 * PI);

	double addYdown = CY + addY;
	double px1 = CX;
	double py1 = CY - R;
	double ax1 = px1 + 5.0;
	double ay1 = py1 + 5.0;
	double bx1 = px1 - 5.0;
	double by1 = py1 + 5.0;

	double px2 = (px1 - CX)*cosT + (CY - py1)*sinT + CX;
	double py2 = (px1 - CX)*sinT + (py1 - CY)*cosT + addYdown;
	double ax2 = (ax1 - CX)*cosT + (CY - ay1)*sinT + CX;
	double ay2 = (ax1 - CX)*sinT + (ay1 - CY)*cosT + addYdown;
	double bx2 = (bx1 - CX)*cosT + (CY - by1)*sinT + CX;
	double by2 = (bx1 - CX)*sinT + (by1 - CY)*cosT + addYdown;

	//Visualisasi sudut kanan atas
	line(draw, Point(CX - R, addYdown), Point(CX + R, addYdown), RED, 2.0);
	line(draw, Point(CX, addYdown - R), Point(CX, addYdown + R), RED, 2.0);
	circle(draw, Point(CX, addYdown), R, BLUE, 2.0);
	line(draw, Point(CX, addYdown), Point(px2, py2), GREEN, 2.0);
	line(draw, Point(ax2, ay2), Point(px2, py2), GREEN, 2.0);
	line(draw, Point(bx2, by2), Point(px2, py2), GREEN, 2.0);

	char textOnImg[15];
	sprintf_s(textOnImg, "%6.1f ", theta);
	putText(draw, textOnImg, Point(CX - R, addYdown + R + 20), FONT_HERSHEY_DUPLEX, 0.6, PURPLE);
}

