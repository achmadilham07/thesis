#pragma once
// https://ieeexplore.ieee.org/document/6387258
// J. Lin, K. Hwang and C. Huang, "Active and Seamless Handover Control of Multi-Camera Systems With 1-DoF Platforms," 
// in IEEE Systems Journal, vol. 8, no. 3, pp. 769-777, Sept. 2014. doi: 10.1109 / JSYST.2012.2224611

#include <vector>
#include <cmath>
#include <algorithm>

using namespace std;

class AHCS
{
private:
	vector<double> posX, posY, EDtoCam;

public:
	void setInput(vector<double> posx, vector<double> posy);
	void setED(double newPosX, double newPosY, double &min_ED_to_Cam, int &min_ED_to_Cam_index);
};

void AHCS::setInput(vector<double> posx, vector<double> posy) {
	posX = posx;
	posY = posy;
}

void AHCS::setED(double newPosX, double newPosY, double &min_ED_to_Cam, int &min_ED_to_Cam_index) {
	EDtoCam.clear();
	for (int i = 0; i < posX.size(); i++) {
		double ED = sqrt(pow((newPosX - posX[i]), 2) + pow((newPosY - posY[i]), 2));
		EDtoCam.push_back(ED);
		//printf(" %.2f", ED);
	}
	/*for (int i = 0; i < EDtoCam.size(); i++)
		printf(" %.2f", EDtoCam[i]);
*/
	min_ED_to_Cam = *min_element(EDtoCam.begin(), EDtoCam.end());
	min_ED_to_Cam_index = min_element(EDtoCam.begin(), EDtoCam.end()) - EDtoCam.begin();
	//printf(" %.2f %d", min_ED_to_Cam, min_ED_to_Cam_index);
}