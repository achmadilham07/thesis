//Example code: A simple server side code, which echos back the received message. 
//Handle multiple socket connections with select and fd_set on Linux 

#include "myFuzzy_01.h"
#include "myFuzzy_02.h"
#include "AHCS.h"
#include <stdio.h> 
#include <stdlib.h> 
#include <errno.h> 
#include <sys/types.h> 
#include <WS2tcpip.h>
#include <string>
#include <iostream>
#include <vector>
#include <sstream>
#include <fstream>
#include "../socket-client-save/calculateRadian.h"
#include <io.h>
#include <fcntl.h>
#include <opencv2/videoio.hpp>
using namespace cv;

#define TRUE 1 
#define FALSE 0 
#define PORT 54000 
#pragma comment(lib, "ws2_32.lib")

#ifndef __linux
#include <io.h> 
#define access _access_s
#else
#include <unistd.h>
#include <memory>
#endif

vector<string> IDClient = { "ID00", "ID01", "ID02" };
vector<string> Boolean = { "$SRVR_0#", "$SRVR_1#" };
vector<string> axisDefault = { "horizontal", "vertical" };
vector<string> sideDefault = { "base", "upbase" };
int idclient = 0;
vector<int> seqID;
vector<double> posX = { 6.7, 0.0, 0.0 };//{ 0.0, 6.7, 0.0 };
vector<double> posY = { 5.0, 0.0, 10.0 };//{ 0.0, 5.0, 10.0 };
vector<double> orientationCamera = { 0, -30.0, 30.0 };//{ -30.0, 0, 30.0 };
vector<double> coordField = { 24.0, 10.0 };
vector<string> axisField = { "vertical", "vertical" , "vertical" };
vector<string> sideField = { "upbase" , "base", "base" }; //{ "base", "upbase" , "base" };
const char *mfout2[3] = { "left  <" , "none" , "right\t>" };
int iteration = 0, subiterL = 0, subiterR = 0;

void splitPos(char* buf, int valread, string &IDClientActive, double &x, double &y);
double distance(double xcam, double ycam, double xprsn, double yprsn);
double orientation1(double xcam, double ycam, double xprsn, double yprsn);
double orientation2(bool &check, double &oldX, double &oldY, double xprsn, double yprsn);
char* checkFuzzy2(double in);
void printText(const char * text, ...);
int checkID(int id, const char * mf);
void sendBool(int sd, int boolean);
int changeID(int id);
void setNewPos(double oldPosX, double oldPosY, double &newPosX, double &newPosY);
void setChar(double orientation);
void checkFuzzy(char *mfoutput2, int sd, int id_client, vector<int> &sockID);
void AHCS_run(int min_ED_to_Cam_index, int sd, int id_client, vector<int> &sockID);
bool file_exists(const std::string& name);

int main(int argc, char *argv[])
{

	const char* keys =
	{
		"{help h|     | show help message}"
		"{madm  |     | Type od MADM Algorithm (0: Cascade Fuzzy; 1: Trad Fuzzy; 2: AHCS) )}"
		"{re    |     | Report all data(coordinates and computation MADM Algorithm)}"
	};

	CommandLineParser parser(argc, argv, keys);

	if (parser.has("help"))
	{
		parser.printMessage();
		exit(0);
	}

	//vector<string> VIDEO_FILE = parser.get< vector<string> >("iv");
	int MADM = parser.get< int >("madm");
	bool REPORT = parser.get< bool >("re");

	int opt = TRUE;
	int master_socket, addrlen, new_socket, client_socket[30], max_clients = 30, activity, i, valread, sd;
	int max_sd;
	struct sockaddr_in address;
	double oldPosX = 0, oldPosY = 0;
	double newPosX, newPosY;
	bool oldPos = false;
	int id_client = 0;
	unsigned long AAtime = 0, BBtime = 0;
	double s_time = 0;

	myFuzzy_01 mFuzz1;
	mFuzz1.setInput();

	myFuzzy_02 mFuzz2;
	mFuzz2.setInput();

	AHCS ahcs;
	ahcs.setInput(posX, posY);

	int j = 0;
	vector<int> sockID;

	char namefile[30];
	char namefldr[20] = "";
	ofstream savereport, MADM_CF, MADM_TF, MADM_AHCS;

	if (REPORT) {
		cout << "REPORT" << endl;
		for (int k = 0; k < 100; k++) {

			sprintf_s(namefldr, "server_%02d", k);
			if (file_exists(namefldr))
				continue;
			else {
				CreateDirectory(namefldr, NULL);
				sprintf_s(namefile, "%s/position.txt", namefldr);
				savereport.open(namefile);
				sprintf_s(namefile, "%s/MADM_CF.txt", namefldr);
				MADM_CF.open(namefile);
				sprintf_s(namefile, "%s/MADM_TF.txt", namefldr);
				MADM_TF.open(namefile);
				sprintf_s(namefile, "%s/MADM_AHCS.txt", namefldr);
				MADM_AHCS.open(namefile);
				break;
			}
		}
	}

	char buffer[4096]; //data buffer of 1K 
	char host[NI_MAXHOST];

	//set of socket descriptors 
	fd_set readfds;

	//initialise all client_socket[] to 0 so not checked 
	for (i = 0; i < max_clients; i++)
	{
		client_socket[i] = 0;
		/*sockID[i] = 0;
		seqID[i] = 0;*/
	}

	// Initialze winsock
	WSADATA wsData;
	WORD ver = MAKEWORD(2, 2);

	int wsOk = WSAStartup(ver, &wsData);
	if (wsOk != 0)
	{
		cerr << "Can't Initialize winsock! Quitting" << endl;
		exit(EXIT_FAILURE);
	}

	//create a master socket 
	if ((master_socket = socket(AF_INET, SOCK_STREAM, 0)) == 0)
	{
		perror("socket failed");
		exit(EXIT_FAILURE);
	}

	//set master socket to allow multiple connections , 
	//this is just a good habit, it will work without this 
	int setSockOpt = setsockopt(master_socket, SOL_SOCKET, SO_REUSEADDR, (char *)&opt, sizeof(opt));
	if (setSockOpt < 0) {
		perror("setsockopt");
		exit(EXIT_FAILURE);
	}

	//type of socket created 
	address.sin_family = AF_INET;
	address.sin_addr.s_addr = INADDR_ANY;
	address.sin_port = htons(PORT);

	//bind the socket to localhost port 54000 
	if ((::bind(master_socket, (struct sockaddr *)&address, sizeof(address))) < 0)
	{
		perror("bind failed");
		exit(EXIT_FAILURE);
	}
	printf("Listener on port %d \n", PORT);

	//try to specify maximum of 3 pending connections for the master socket 
	if (listen(master_socket, 3) < 0)
	{
		perror("listen");
		exit(EXIT_FAILURE);
	}

	//accept the incoming connection 
	addrlen = sizeof(address);
	puts("Waiting for connections ...");

	while (TRUE)
	{
		//clear the socket set 
		FD_ZERO(&readfds);

		//add master socket to set 
		FD_SET(master_socket, &readfds);
		max_sd = master_socket;

		//add child sockets to set 
		for (i = 0; i < max_clients; i++)
		{
			//socket descriptor 
			sd = client_socket[i];

			//if valid socket descriptor then add to read list 
			if (sd > 0)
				FD_SET(sd, &readfds);

			//highest file descriptor number, need it for the select function 
			if (sd > max_sd)
				max_sd = sd;
		}

		//wait for an activity on one of the sockets , timeout is NULL , 
		//so wait indefinitely 
		activity = select(max_sd + 1, &readfds, NULL, NULL, NULL);

		if ((activity < 0) && (errno != EINTR))
		{
			printf("select error");
		}

		//If something happened on the master socket , 
		//then its an incoming connection 
		if (FD_ISSET(master_socket, &readfds))
		{
			if ((new_socket = accept(master_socket, (struct sockaddr *)&address, (socklen_t*)&addrlen)) < 0)
			{
				perror("accept");
				exit(EXIT_FAILURE);
			}

			ZeroMemory(host, NI_MAXHOST);
			inet_ntop(AF_INET, &address.sin_addr, host, NI_MAXHOST);

			//inform user of socket number - used in send and receive commands 
			printf("New connection , socket fd is %d , ip is : %s , port : %d \n", new_socket, host, ntohs(address.sin_port));

			puts("Welcome message sent successfully");

			//add new socket to array of sockets 
			for (i = 0; i < max_clients; i++)
			{
				//if position is empty 
				if (client_socket[i] == 0)
				{
					client_socket[i] = new_socket;
					printf("Adding to list of sockets as %d\n", i);
					j = i;

					break;
				}
			}
		}

		//check first client id that is on
		for (i = 0; i < max_clients; i++)
		{
			if (i == j) {
				sd = client_socket[i];
				//cout << host_sock[i] << endl;

				if (FD_ISSET(sd, &readfds))
				{

					valread = recv(sd, buffer, 4096, 0);
					if (valread == 0)
					{
						cout << "Client " << i << " disconnected " << endl;
						// Close the socket
						closesocket(sd);
						client_socket[i] = 0;
					}
					else
					{
						buffer[valread] = '\0';
						string text = string(buffer, 0, valread);
						int id = (int)text[3] - 48;
						sockID.push_back(sd);
						seqID.push_back(id);
						j = -1;
						if (id == 0) {
							cout << text << endl;
							sendBool(sd, 1);
						}
					}
				}
			}

			if (i == id_client) {
				sd = client_socket[i];
				cout << "Server > ";

				if (FD_ISSET(sd, &readfds))
				{
					//Check if it was for closing , and also read the
					//incoming message
					ZeroMemory(buffer, 4096);
					valread = recv(sd, buffer, 4096, 0);

					if (valread == 0)
					{
						cout << "Client " << i << " disconnected " << endl;
						// Close the socket
						closesocket(sd);

						client_socket[i] = 0;

					}

					//Echo back the message that came in
					else
					{
						//set the string terminating NULL byte on the end
						//of the data read
						double x = 0, y = 0;
						string idClientActive;
						splitPos(buffer, valread, idClientActive, x, y);

						// Check ID Client and Camera Position
						double posXActive = 0, posYActive = 0;

						// Euclidean Distance
						double Distance = distance(posXActive, posYActive, x, y);

						// Orientation Object against Camera
						double Orientation1 = orientation1(posXActive, posYActive, x, y);

						// Orientation Object against Time
						double Orientation2 = orientation2(oldPos, oldPosX, oldPosY, x, y);

						// Set a new position against observation field
						setNewPos(oldPosX, oldPosY, newPosX, newPosY);

						//printText("id: %s \tX: %4.2f \tY: %4.2f ", idClientActive.c_str(), x, y);

						printf("PosX = %4.2f \tPosY = %4.2f \t%f \t%f \t%f ", newPosX, newPosY, Distance, Orientation1, Orientation2);
						//printText("dist: %4.2f \tarah1: %4.2f \tarah2: %7.2f ", Distance, Orientation1, Orientation2);
						//setChar(Orientation2);

						switch (MADM)
						{
						case 0:
							// CASCADE FUZZY RUN
							AAtime = getTickCount();
							double output_01, output_02;
							mFuzz1.get_sugeno(Distance, Orientation1, Orientation2, output_01, output_02);
							char *mfoutput1;
							mfoutput1 = checkFuzzy2(output_02);
							checkFuzzy(mfoutput1, sd, id_client, sockID);
							BBtime = getTickCount();
							s_time = (BBtime - AAtime) / getTickFrequency();
							MADM_CF << iteration << "\t" << s_time << endl;
							break;
						case 1:
							// TRADITIONAL FUZZY RUN
							AAtime = getTickCount();
							double output_11;
							mFuzz2.get_sugeno(Distance, Orientation1, Orientation2, output_11);
							char *mfoutput2;
							mfoutput2 = checkFuzzy2(output_11);
							checkFuzzy(mfoutput2, sd, id_client, sockID);
							BBtime = getTickCount();
							s_time = (BBtime - AAtime) / getTickFrequency();
							MADM_TF << iteration << "\t" << s_time << endl;
							break;
						case 2:
							// AHCS RUN
							AAtime = getTickCount();
							double min_ED_to_Cam;
							int min_ED_to_Cam_index;
							ahcs.setED(newPosX, newPosY, min_ED_to_Cam, min_ED_to_Cam_index);
							AHCS_run(min_ED_to_Cam_index, sd, id_client, sockID);
							BBtime = getTickCount();
							s_time = (BBtime - AAtime) / getTickFrequency();
							MADM_AHCS << iteration << "\t" << s_time << endl;
							break;
						default:
							break;
						}

						//printf("PosX = %4.2f \tPosY = %4.2f \t%f \t%f \t%f ", newPosX, newPosY, Distance, Orientation1, Orientation2);

						if (REPORT) {
							savereport << iteration << "\t" << newPosX << "\t" << newPosY << "\t" << idclient << endl;
						}

						iteration++;
						printText("\n");
					}
				}
			}
		}

		////else its some IO operation on some other socket 
		//for (i = 0; i < max_clients; i++)
		//{
		//	sd = client_socket[i];
		//	if (FD_ISSET(sd, &readfds))
		//	{
		//		//Check if it was for closing , and also read the 
		//		//incoming message 
		//		if ((valread = recv(sd, buffer, 4096, 0)) == 0)
		//		{
		//			//Somebody disconnected , get his details and print 
		//			ZeroMemory(host, NI_MAXHOST);
		//			inet_ntop(AF_INET, &address.sin_addr, host, NI_MAXHOST);
		//			getpeername(sd, (struct sockaddr*)&address, (socklen_t*)&addrlen);
		//			printf("Host disconnected , ip %s , port %d \n", host, ntohs(address.sin_port));
		//			//Close the socket and mark as 0 in list for reuse 
		//			closesocket(sd);
		//			client_socket[i] = 0;
		//		}
		//		//Echo back the message that came in 
		//		else
		//		{
		//			//set the string terminating NULL byte on the end 
		//			//of the data read 
		//			buffer[valread] = '\0';
		//			string text = string(buffer, 0, valread);
		//			cout << text << endl;
		//			send(sd, buffer, strlen(buffer), 0);
		//		}
		//	}
		//}

	}
	if (REPORT) {
		savereport.close();
		MADM_CF.close();
		MADM_TF.close();
		MADM_AHCS.close();
	}

	return 0;
}

void AHCS_run(int min_ED_to_Cam_index, int sd, int id_client, vector<int> &sockID) {
	id_client = changeID(min_ED_to_Cam_index);
	sd = sockID[id_client];
	sendBool(sd, 1);
}

void checkFuzzy(char *mfoutput, int sd, int id_client, vector<int> &sockID) {
	if (strcmp(mfoutput, mfout2[0]) == 0) {
		subiterL++;
		//printText("ini masuk sini - kiri");
	}
	else if (strcmp(mfoutput, mfout2[2]) == 0) {
		subiterR++;
		//printText("ini masuk sini - kanan");
	}

	if (subiterL >= 7 && (iteration % 10) == 0) {
		cout << "\n___________________________LEFT________________________________" << endl;
		sendBool(sd, 0);
		subiterL = 0;
		idclient = checkID(idclient, mfout2[0]);
		printText("-----------------------pindah ke ID0%d-----------------------\n", idclient);
		id_client = changeID(idclient);
		sd = sockID[id_client];
		sendBool(sd, 1);
	}
	else if (subiterR >= 7 && (iteration % 10) == 0) {
		cout << "\n____________________________RIGHT_____________________________" << endl;
		sendBool(sd, 0);
		subiterR = 0;
		idclient = checkID(idclient, mfout2[2]);
		printText("-----------------------pindah ke ID0%d-----------------------\n", idclient);
		id_client = changeID(idclient);
		sd = sockID[id_client];
		sendBool(sd, 1);
	}
	else {
		sendBool(sd, 1);
	}

	if ((iteration % 10) == 1) {
		subiterL = 0;
		subiterR = 0;
	}
}


void splitPos(char* buf, int valread, string &IDClientActive, double &x, double &y) {
	buf[valread] = '\0';
	string text;
	stringstream(string(buf, 0, valread)) >> text;
	char seps[] = " $_#";
	vector<string> arrText;
	char *nextText = NULL;
	char *token = NULL;

	char* c = const_cast<char*>(text.c_str());
	token = strtok_s(c, seps, &nextText);

	while (token != NULL)
	{
		arrText.push_back(token);
		token = strtok_s(NULL, seps, &nextText);

	}

	IDClientActive = arrText[0];
	x = stof(arrText[1]);
	y = stof(arrText[2]);
}

double distance(double xcam, double ycam, double xprsn, double yprsn) {
	double d1 = (yprsn - ycam);
	double d2 = (xprsn - xcam);

	// Euclidean Distance
	return pow((pow(d1, 2) + pow(d2, 2)), 0.5);
}

double orientation1(double xcam, double ycam, double xprsn, double yprsn) {
	double d1 = (yprsn - ycam);
	double d2 = (xprsn - xcam);

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

char* checkFuzzy2(double in) {
	char out[10] = "";
	if (in >= -1 && in < -0.5) {
		sprintf_s(out, "%s", mfout2[0]);
	}
	else if (in >= -0.5 && in < 0.5) {
		sprintf_s(out, "%s", mfout2[1]);
	}
	else if (in >= 0.5 && in < 1) {
		sprintf_s(out, "%s", mfout2[2]);
	}
	return out;
}

void printText(const char * text, ...) {
	printf(text);
}

int checkID(int id, const char * mf) {
	if (id == 0 && mf == mfout2[0]) {
		return 0;
	}
	else if (id == 0 && mf == mfout2[2]) {
		return 1;
	}
	if (id == 1 && mf == mfout2[0]) {
		return 0;
	}
	else if (id == 1 && mf == mfout2[2]) {
		return 2;
	}
	if (id == 2 && mf == mfout2[0]) {
		return 1;
	}
	else if (id == 2 && mf == mfout2[2]) {
		return 2;
	}
	return 0;
}

void sendBool(int sd, int boolean) {
	const char * charOK = Boolean[boolean].c_str();
	int lenOK = strlen(Boolean[boolean].c_str()) + 1;
	int sendResult = send(sd, charOK, lenOK, 0);
}

int changeID(int id) {
	for (size_t k = 0; k < seqID.size(); k++) {
		if (id == seqID[k]) {
			return k;
		}
	}
	return 0;
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

void setChar(double orientation) {
	//SetConsoleOutputCP(CP_UTF8);
	_setmode(_fileno(stdout), _O_U16TEXT);
	if (orientation > -180 && orientation <= (-135 - 22.5)) {
		wprintf(L"\x2193\x2193        ");
	}
	else if (orientation > (-135 - 22.5) && orientation <= (-90 - 22.5)) {
		wprintf(L" \x2193\x2190       ");
	}
	else if (orientation > (-90 - 22.5) && orientation <= (-45 - 22.5)) {
		wprintf(L"  \x2190\x2190      ");
	}
	else if (orientation > (-45 - 22.5) && orientation <= (0 - 22.5)) {
		wprintf(L"   \x2190\x2191     ");
	}
	else if (orientation > (0 - 22.5) && orientation <= (0 + 22.5)) {
		wprintf(L"    \x2191\x2191    ");
	}
	else if (orientation > (0 + 22.5) && orientation <= (45 + 22.5)) {
		wprintf(L"     \x2191\x2192   ");
	}
	else if (orientation > (45 + 22.5) && orientation <= (90 + 22.5)) {
		wprintf(L"      \x2192\x2192  ");
	}
	else if (orientation > (90 + 22.5) && orientation <= (135 + 22.5)) {
		wprintf(L"       \x2192\x2193 ");
	}
	else if (orientation > (135 + 22.5) && orientation <= 180) {
		wprintf(L"        \x2193\x2193");
	}
	_setmode(_fileno(stdout), _O_TEXT);
}
//
//bool file_exists(const string & file)
//{
//	return access(file.c_str(), 0) == 0;
//}
bool file_exists(const std::string& name) {
	struct stat buffer;
	return (stat(name.c_str(), &buffer) == 0);
}