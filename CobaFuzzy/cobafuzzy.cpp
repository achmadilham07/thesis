#pragma once
#include "fl/Headers.h"
#include "../socket-server-save/myFuzzy_01.h"
#include "../socket-server-save/myFuzzy_02.h"
#include <string.h>
#include <stdio.h> 
//assadsa
using namespace fl;
using namespace std;

int main(int argc, char* argv[]) {

	float mdistance = stof(argv[1]);
	float morientation = stof(argv[2]);
	float mheading = stof(argv[3]);

	myFuzzy_01 mFuzzy;
	double output1_sugeno, output2_sugeno;

	myFuzzy_02 mFuzzy2;
	double output1_sugeno2;

	mFuzzy.setInput();
	mFuzzy.get_sugeno(mdistance, morientation, mheading, output1_sugeno, output2_sugeno);

	mFuzzy2.setInput();
	mFuzzy2.get_sugeno(mdistance, morientation, mheading, output1_sugeno2);

	cout << "distance.input = " << mdistance << endl;
	cout << "orientation.input = " << morientation << endl;
	cout << "orientation.input = " << mheading << endl;
	cout << "output sugeno cascade 1 = " << output1_sugeno << endl;
	cout << "output sugeno cascade 2 = " << output2_sugeno << endl;
	cout << "output sugeno trad 1 = " << output1_sugeno2 << endl;
}