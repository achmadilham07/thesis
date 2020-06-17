//
//	Project ini hanya untuk simulasi dengan 3 kamera.
//	Posisi kamera adalah Cam1 (0,0) ; Cam2 (6.7,5) ; Cam3 (0,10)
//

#include "../socket-client-save/kalman.h"
#include "../socket-client-save/calculateRadian.h"
#include "../socket-server-save/myFuzzy_01.h"
#include "../socket-server-save/myFuzzy_02.h"
#include "../socket-server-save/AHCS.h"
#include <iostream>
#include <cstdlib>
#include <ctime>
#include <string>
#include <vector>
#include <fstream>
#include <cmath>
#include <opencv2/videoio.hpp>

using namespace std;

vector<int> seqID = { 0,1,2 };
vector<double> posX = { 0.0, 6.7, 0.0 }; //{ 6.7, 0.0, 0.0 };//{ 0.0, 6.7, 0.0 };
vector<double> posY = { 0.0, 5.0, 10.0 }; //{ 5.0, 0.0, 10.0 };//{ 0.0, 5.0, 10.0 };
vector<double> orientationCamera = { -50.0, 0, 50.0 }; //{ 0, -30.0, 30.0 };//{ -30.0, 0, 30.0 };
vector<string> sideField = { "base", "upbase" , "base" }; //{ "upbase" , "base", "base" }; //{ "base", "upbase" , "base" };
vector<string> axisField = { "vertical", "vertical" , "vertical" };
vector<string> axisDefault = { "horizontal", "vertical" };
vector<string> sideDefault = { "base", "upbase" };
int idclient = 0;
int subiterL = 0, subiterR = 0, subiter = 0, id_client_tmp = 0;
int maxiter = 3;

double distance(double xprsn, double yprsn);
double orientation1(double xprsn, double yprsn);
double orientation2(bool &check, double &oldX, double &oldY, double xprsn, double yprsn);
void setNewPos(double oldPosX, double oldPosY, double &newPosX, double &newPosY);
void setNewPos2(double &newPosX, double &newPosY);
int checkFuzzy_(double in);
void checkFuzzy1(int index, int &id_client);
void AHCS_run(int min_ED_to_Cam_index, int &id_client);
int checkID(int id, int mf);
int changeID(int id);
double setRandom() {
	double min = 0.08;
	double max = 0.20;
	/*double min = 2.5;
	double max = 3.0;*/
	double randu = ((double)rand() / (RAND_MAX));

	double randNum = min + ((max - min)*randu);
	return randNum;
}

int main(int argc, char** argv)
{
	const char* keys =
	{
		"{help h|     | show help message}"
		"{madm  |     | Type of MADM Algorithm (0: Cascade Fuzzy; 1: Trad Fuzzy; 2: AHCS) )}"
		"{fl    |     | Name file random}"
		"{fs    |     | Name file save}"
		"{re    |     | Report all data(coordinates and computation MADM Algorithm)}"
	};

	CommandLineParser parser(argc, argv, keys);

	if (parser.has("help"))
	{
		parser.printMessage();
		exit(0);
	}
	srand(time(NULL));

	//vector<string> VIDEO_FILE = parser.get< vector<string> >("iv");
	int MADM = parser.get< int >("madm");
	bool REPORT = parser.get< bool >("re");
	String nextPos = parser.get< String >("fl"); //"E:/Belajarubic - Home/Tugas Kuliah/PENS - S2/Thesis/Particle Filter/Particle Filter/P_Filter_2D/newPos_019.txt";
	String namefile = parser.get< String >("fs");

	int iter = 0, iter2 = 0;
	int id_client = 0;
	double s_time = 0.100;
	double theta_kf2 = 0;
	double newXkf = 0, newYkf = 0;
	double oldPosX = 0, oldPosY = 0;
	double newPosX = 0, newPosY = 0;
	double Distance, Orientation1, Orientation2;
	vector<double> posX_kf2, posY_kf2;
	bool after_kf2 = false, oldPos = false;

	ofstream hog_kf2;
	hog_kf2.open(namefile);

	kalman kfPos2;
	kfPos2.kalmanPos2_init();

	myFuzzy_01 mFuzz1;
	mFuzz1.setInput();

	myFuzzy_02 mFuzz2;
	mFuzz2.setInput();

	AHCS ahcs;
	ahcs.setInput(posX, posY);

	vector<double> coord_x, coord_y, iteration2;
	ifstream fNextPos(nextPos);
	if (fNextPos.is_open()) {
		double a, b, c = 0;
		while (fNextPos >> a >> b)
		{
			//cout << "x > " << a << "\ty > " << b << endl;
			coord_x.push_back(a);
			coord_y.push_back(b);
			iteration2.push_back(c);
			c++;
		}
		fNextPos.close();
	}

	for (int iteration = 0; iteration < iteration2.size(); iteration++) {
		s_time = setRandom();
		{
			iter = iteration;
			printf("iter = %2d %.3f\t", iter, s_time);
			int kurang = abs(iter2 - iter);
			if (kfPos2.found2 && (kurang == 0 || kurang == 1)) {
				kfPos2.setA2(s_time * 1000);
				kfPos2.predictPos2();

				posX_kf2.push_back(kfPos2.getPos2x());
				posY_kf2.push_back(kfPos2.getPos2y());

				theta_kf2 = calculateRadian(posX_kf2[0], posY_kf2[0], posX_kf2[1], posY_kf2[1]);

				if (after_kf2) {
					posX_kf2.erase(posX_kf2.begin());
					posY_kf2.erase(posY_kf2.begin());
				}

				newXkf = posX_kf2[0];
				newYkf = posY_kf2[0];
			}
			else {
				posX_kf2.push_back(coord_x[iter]);
				posY_kf2.push_back(coord_y[iter]);

				if (posX_kf2.size() >= 2 || posY_kf2.size() >= 2) {
					posX_kf2.erase(posX_kf2.begin());
					posY_kf2.erase(posY_kf2.begin());
				}
			}

			if ((iter % 1) == 0) {
				if (iter != iteration2.size()) {
					kfPos2.updatePos2(coord_x[iter], coord_y[iter]);
				}
				else {
					kfPos2.updatePos2(posX_kf2[0], posY_kf2[0]);
				}
			}
			else {
				kfPos2.updatePos2(posX_kf2[0], posY_kf2[0]);
			}
			after_kf2 = true;
			iter2 = iter;
		}
		{
			setNewPos2(posX_kf2[0], posY_kf2[0]);

			// Euclidean Distance
			Distance = distance(posX_kf2[0], posY_kf2[0]);

			// Orientation Object against Camera
			Orientation1 = orientation1(posX_kf2[0], posY_kf2[0]);

			// Orientation Object against Time
			Orientation2 = orientation2(oldPos, oldPosX, oldPosY, posX_kf2[0], posY_kf2[0]);

			// Set a new position against observation field
			setNewPos(oldPosX, oldPosY, newPosX, newPosY);
		}

		{
			double output_01 = 0, output_02 = 0;
			int index = 0;
			switch (MADM)
			{
			case 0:
				// CASCADE FUZZY RUN
				mFuzz1.get_sugeno(Distance, Orientation1, Orientation2, output_01, output_02);
				index = checkFuzzy_(output_02);
				checkFuzzy1(index, id_client);
				if (MADM == 0) {
					printf("Output : %6.2f", output_02);
				}
				break;
			case 1:
				// TRADITIONAL FUZZY RUN
				mFuzz2.get_sugeno(Distance, Orientation1, Orientation2, output_01);
				index = checkFuzzy_(output_01);
				checkFuzzy1(index, id_client);
				if (MADM == 1) {
					printf("Output : %6.2f", output_01);
				}
				break;
			case 2:
				// AHCS RUN
				double min_ED_to_Cam;
				int min_ED_to_Cam_index;
				ahcs.setED(newPosX, newPosY, min_ED_to_Cam, min_ED_to_Cam_index);
				AHCS_run(min_ED_to_Cam_index, id_client);
				break;
			}
		}
		if (REPORT) {
			hog_kf2 << iteration << "\t" << newPosX << "\t" << newPosY << "\t" << id_client << endl;
		}
		printf("|| %6.2f \t%6.2f || %6.2f \t%6.2f\n", oldPosX, oldPosY, newPosX, newPosY);
		printf("\n");


	}
}

double distance(double xprsn, double yprsn) {
	double d1 = (yprsn);
	double d2 = (xprsn);

	// Euclidean Distance
	return pow((pow(d1, 2) + pow(d2, 2)), 0.5);
}

double orientation1(double xprsn, double yprsn) {
	double d1 = (yprsn);
	double d2 = (xprsn);

	// Orientation Object Against Camera
	return atan(d2 / d1) * 180 / PI;
}

double orientation2(bool &check, double &oldX, double &oldY, double xprsn, double yprsn) {
	double Orientation = 0;
	if (!check) {
		oldX = xprsn;
		oldY = yprsn;
		check = true;
	}
	if (check) {
		Orientation = calculateRadian(oldX, oldY, xprsn, yprsn);
		oldX = xprsn;
		oldY = yprsn;
	}

	// Orientation Object Against current time and previous time
	return Orientation;
}

void setNewPos(double oldPosX, double oldPosY, double &newPosX, double &newPosY) {
	double newPosXtemp = oldPosX * cos(orientationCamera[idclient] * PI / 180) + oldPosY * sin(orientationCamera[idclient] * PI / 180);
	double newPosYtemp = -(oldPosX)* sin(orientationCamera[idclient] * PI / 180) + oldPosY * cos(orientationCamera[idclient] * PI / 180);
	if (axisField[idclient] == axisDefault[0] && sideField[idclient] == sideDefault[0]) {
		newPosX = posX[idclient] + newPosXtemp;
		newPosY = posY[idclient] + newPosYtemp;
	}
	else if (axisField[idclient] == axisDefault[0] && sideField[idclient] == sideDefault[1]) {
		newPosX = posX[idclient] - newPosXtemp;
		newPosY = posY[idclient] - newPosYtemp;
	}
	else if (axisField[idclient] == axisDefault[1] && sideField[idclient] == sideDefault[0]) {
		newPosX = posX[idclient] + newPosYtemp;
		newPosY = posY[idclient] - newPosXtemp;
	}
	else if (axisField[idclient] == axisDefault[1] && sideField[idclient] == sideDefault[1]) {
		newPosX = posX[idclient] - newPosYtemp;
		newPosY = posY[idclient] + newPosXtemp;
	}
}

void setNewPos2(double &newPosX, double &newPosY) {
	double newPosXTemp = 0, newPosYTemp = 0;
	if (axisField[idclient] == axisDefault[0] && sideField[idclient] == sideDefault[0]) {
		newPosXTemp = newPosX - posX[idclient];
		newPosYTemp = newPosY - posY[idclient];
	}
	else if (axisField[idclient] == axisDefault[0] && sideField[idclient] == sideDefault[1]) {
		newPosXTemp = posX[idclient] - newPosX;
		newPosYTemp = posY[idclient] - newPosY;
	}
	else if (axisField[idclient] == axisDefault[1] && sideField[idclient] == sideDefault[0]) {
		newPosXTemp = posY[idclient] - newPosY;
		newPosYTemp = newPosX - posX[idclient];
	}
	else if (axisField[idclient] == axisDefault[1] && sideField[idclient] == sideDefault[1]) {
		newPosXTemp = newPosY - posY[idclient];
		newPosYTemp = posX[idclient] - newPosX;
	}
	double degree = -1 * orientationCamera[idclient];
	newPosX = newPosXTemp * cos(degree* PI / 180) + newPosYTemp * sin(degree * PI / 180);
	newPosY = -(newPosXTemp)* sin(degree * PI / 180) + newPosYTemp * cos(degree * PI / 180);
}

int checkFuzzy_(double in) {
	double out = in;
	if (out >= -1 && out <= -0.5) {
		return -1;
	}
	else if (out > -0.5 && out < 0.5) {
		return 0;
	}
	else if (out >= 0.5 && out <= 1) {
		return 1;
	}
	return 0;
}

void checkFuzzy1(int index, int &id_client) {
	int index_temp = index;
	printf("\tindex_temp: %2d ", index_temp);

	if (index_temp != 0) {
		if (index_temp == -1) {
			subiterL++;
		}
		else {
			subiterR++;
		}
		subiter++;
		if ((subiterR == maxiter || subiterL == maxiter) && subiter == maxiter) {
			idclient = checkID(id_client, index_temp);
			printf("\n-----------------------pindah ke ID0%d-----------------------\n", idclient);
			id_client_tmp = changeID(idclient);
			printf("\tid_client_tmp: %d\t", id_client_tmp);
			id_client = id_client_tmp;
		}
	}
	else {
		subiter = 0;
		subiterR = 0;
		subiterL = 0;
	}
	if (subiter >= maxiter) {
		subiter = 0;
		subiterR = 0;
		subiterL = 0;
	}
}

void AHCS_run(int min_ED_to_Cam_index, int &id_client) {
	int index_temp = min_ED_to_Cam_index;
	if (index_temp != id_client) {
		id_client_tmp = changeID(index_temp);
		id_client = id_client_tmp;
		printf("\n-----------------------pindah ke ID0%d-----------------------\n", index_temp);
	}
}

int checkID(int id, int mf) {
	if (id == 0) {
		if (mf == -1 || mf == 1) {
			return 1;
		}
	}
	if (id == 1) {
		if (mf == -1) {
			return 0;
		}
		else if (mf == 1) {
			return 2;
		}
	}
	if (id == 2) {
		if (mf == -1 || mf == 1) {
			return 1;
		}
	}
	return 0;
}

int changeID(int id) {
	for (size_t k = 0; k < seqID.size(); k++) {
		//printf("| %d %d %d|", id, seqID[k], k);
		if (id == seqID[k]) {
			return k;
		}
	}
	return 0;
}