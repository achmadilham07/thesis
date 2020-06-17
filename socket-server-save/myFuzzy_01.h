#pragma once
#include "fl/Headers.h"

using namespace fl;
using namespace std;

class myFuzzy_01
{
private:
	Engine *engine_sugeno1, *engine_sugeno2;
	InputVariable* mDistance, *mOrientation, *moutput1, *mHeading;
	OutputVariable *output1_sugeno, *output2_sugeno;

	void set_input1();
	void set_input2();
	void set_sugeno1();
	void set_sugeno2();
public:
	void setInput();
	void get_sugeno(double distance, double orientation, double heading, double &output1, double &output2);
};

void myFuzzy_01::setInput() {
	set_input1();
	set_sugeno1();

	set_input2();
	set_sugeno2();
}

void myFuzzy_01::set_input1() {
	mDistance = new InputVariable;
	mDistance->setName("Distance");
	mDistance->setDescription("");
	mDistance->setEnabled(true);
	mDistance->setRange(0.000, 5.000);
	mDistance->setLockValueInRange(false);
	mDistance->addTerm(new Ramp("very_small", 1.25, 0));
	mDistance->addTerm(new Triangle("small", 0, 1.25, 2.5));
	mDistance->addTerm(new Triangle("medium", 1.25, 2.5, 3.75));
	mDistance->addTerm(new Triangle("big", 2.5, 3.75, 5));
	mDistance->addTerm(new Ramp("very_big", 3.75, 5));

	mOrientation = new InputVariable;
	mOrientation->setName("Orientation");
	mOrientation->setDescription("");
	mOrientation->setEnabled(true);
	mOrientation->setRange(-17.000, 17.000);
	mOrientation->setLockValueInRange(false);
	mOrientation->addTerm(new Ramp("negative_big", -11.33, -17));
	mOrientation->addTerm(new Triangle("negative_medium", -17, -11.33, -5.667));
	mOrientation->addTerm(new Triangle("negative_small", -11.33, -5.667, 0));
	mOrientation->addTerm(new Triangle("zero", -5.667, 0, 5.667));
	mOrientation->addTerm(new Triangle("positive_small", 0, 5.667, 11.33));
	mOrientation->addTerm(new Triangle("positive_medium", 5.667, 11.33, 17));
	mOrientation->addTerm(new Ramp("positive_big", 11.33, 17));

	output1_sugeno = new OutputVariable;
	output1_sugeno->setName("output1");
	output1_sugeno->setDescription("");
	output1_sugeno->setEnabled(true);
	output1_sugeno->setRange(-1.000, 1.000);
	output1_sugeno->setLockValueInRange(false);
	output1_sugeno->setAggregation(new AlgebraicSum);
	output1_sugeno->setDefuzzifier(new WeightedAverage("TakagiSugeno"));
	output1_sugeno->setDefaultValue(0);
	output1_sugeno->setLockPreviousValue(false);
	output1_sugeno->addTerm(new Constant("left_side", -1));
	output1_sugeno->addTerm(new Constant("none", 0));
	output1_sugeno->addTerm(new Constant("right_side", 1));
};

void myFuzzy_01::set_input2() {
	moutput1 = new InputVariable;
	moutput1->setName("Output1");
	moutput1->setDescription("");
	moutput1->setEnabled(true);
	moutput1->setRange(-1.000, 1.000);
	moutput1->setLockValueInRange(false);
	moutput1->addTerm(new Ramp("left_side", 0, -1));
	moutput1->addTerm(new Triangle("none", -1, 0, 1));
	moutput1->addTerm(new Ramp("right_side", 0, 1));

	mHeading = new InputVariable;
	mHeading->setName("Heading");
	mHeading->setDescription("");
	mHeading->setEnabled(true);
	mHeading->setRange(-180.000, 180.000);
	mHeading->setLockValueInRange(false);
	mHeading->addTerm(new Ramp("negative_front", -135, -180));
	mHeading->addTerm(new Triangle("front_left", -180, -135, -90));
	mHeading->addTerm(new Triangle("left", -135, -90, -45));
	mHeading->addTerm(new Triangle("back_left", -90, -45, 0));
	mHeading->addTerm(new Triangle("back", -45, 0, 45));
	mHeading->addTerm(new Triangle("back_right", 0, 45, 90));
	mHeading->addTerm(new Triangle("right", 45, 90, 135));
	mHeading->addTerm(new Triangle("front_right", 90, 135, 180));
	mHeading->addTerm(new Ramp("positive_front", 135, 180));

	output2_sugeno = new OutputVariable;
	output2_sugeno->setName("output2");
	output2_sugeno->setDescription("");
	output2_sugeno->setEnabled(true);
	output2_sugeno->setRange(-1.000, 1.000);
	output2_sugeno->setLockValueInRange(false);
	output2_sugeno->setAggregation(new AlgebraicSum);
	output2_sugeno->setDefuzzifier(new WeightedAverage("TakagiSugeno"));
	output2_sugeno->setDefaultValue(0);
	output2_sugeno->setLockPreviousValue(false);
	output2_sugeno->addTerm(new Constant("left_handoff", -1));
	output2_sugeno->addTerm(new Constant("none", 0));
	output2_sugeno->addTerm(new Constant("right_handoff", 1));
};

void myFuzzy_01::get_sugeno(double distance, double orientation, double heading, double &output1, double &output2) {

	mDistance->setValue(distance);
	mOrientation->setValue(orientation);
	engine_sugeno1->process();

	output1 = output1_sugeno->getValue();
	moutput1->setValue(output1);
	mHeading->setValue(heading);
	engine_sugeno2->process();

	output2 = output2_sugeno->getValue();
};

void myFuzzy_01::set_sugeno1() {
	engine_sugeno1 = new Engine;
	engine_sugeno1->setName("Fuzzy1_sugeno");
	engine_sugeno1->setDescription("");

	engine_sugeno1->addInputVariable(mDistance);
	engine_sugeno1->addInputVariable(mOrientation);
	engine_sugeno1->addOutputVariable(output1_sugeno);

	RuleBlock *ruleBlock = new RuleBlock;
	ruleBlock->setName("");
	ruleBlock->setDescription("");
	ruleBlock->setEnabled(true);
	ruleBlock->setConjunction(new Minimum);
	ruleBlock->setDisjunction(new Maximum);
	ruleBlock->setImplication(new AlgebraicProduct);
	ruleBlock->setActivation(new General);
	ruleBlock->addRule(Rule::parse("if (Distance is very_small) and (Orientation is negative_big) then output1 is none", engine_sugeno1));
	ruleBlock->addRule(Rule::parse("if (Distance is very_small) and (Orientation is negative_medium) then output1 is none", engine_sugeno1));
	ruleBlock->addRule(Rule::parse("if (Distance is very_small) and (Orientation is negative_small) then output1 is none", engine_sugeno1));
	ruleBlock->addRule(Rule::parse("if (Distance is very_small) and (Orientation is zero) then output1 is none", engine_sugeno1));
	ruleBlock->addRule(Rule::parse("if (Distance is very_small) and (Orientation is positive_small) then output1 is none", engine_sugeno1));
	ruleBlock->addRule(Rule::parse("if (Distance is very_small) and (Orientation is positive_medium) then output1 is none", engine_sugeno1));
	ruleBlock->addRule(Rule::parse("if (Distance is very_small) and (Orientation is positive_big) then output1 is none", engine_sugeno1));

	ruleBlock->addRule(Rule::parse("if (Distance is small) and (Orientation is negative_big) then output1 is none", engine_sugeno1));
	ruleBlock->addRule(Rule::parse("if (Distance is small) and (Orientation is negative_medium) then output1 is none", engine_sugeno1));
	ruleBlock->addRule(Rule::parse("if (Distance is small) and (Orientation is negative_small) then output1 is none", engine_sugeno1));
	ruleBlock->addRule(Rule::parse("if (Distance is small) and (Orientation is zero) then output1 is none", engine_sugeno1));
	ruleBlock->addRule(Rule::parse("if (Distance is small) and (Orientation is positive_small) then output1 is none", engine_sugeno1));
	ruleBlock->addRule(Rule::parse("if (Distance is small) and (Orientation is positive_medium) then output1 is none", engine_sugeno1));
	ruleBlock->addRule(Rule::parse("if (Distance is small) and (Orientation is positive_big) then output1 is none", engine_sugeno1));

	ruleBlock->addRule(Rule::parse("if (Distance is medium) and (Orientation is negative_big) then output1 is left_side", engine_sugeno1));
	ruleBlock->addRule(Rule::parse("if (Distance is medium) and (Orientation is negative_medium) then output1 is none", engine_sugeno1));
	ruleBlock->addRule(Rule::parse("if (Distance is medium) and (Orientation is negative_small) then output1 is none", engine_sugeno1));
	ruleBlock->addRule(Rule::parse("if (Distance is medium) and (Orientation is zero) then output1 is none", engine_sugeno1));
	ruleBlock->addRule(Rule::parse("if (Distance is medium) and (Orientation is positive_small) then output1 is none", engine_sugeno1));
	ruleBlock->addRule(Rule::parse("if (Distance is medium) and (Orientation is positive_medium) then output1 is none", engine_sugeno1));
	ruleBlock->addRule(Rule::parse("if (Distance is medium) and (Orientation is positive_big) then output1 is right_side", engine_sugeno1));

	ruleBlock->addRule(Rule::parse("if (Distance is big) and (Orientation is negative_big) then output1 is left_side", engine_sugeno1));
	ruleBlock->addRule(Rule::parse("if (Distance is big) and (Orientation is negative_medium) then output1 is left_side", engine_sugeno1));
	ruleBlock->addRule(Rule::parse("if (Distance is big) and (Orientation is negative_small) then output1 is none", engine_sugeno1));
	ruleBlock->addRule(Rule::parse("if (Distance is big) and (Orientation is zero) then output1 is none", engine_sugeno1));
	ruleBlock->addRule(Rule::parse("if (Distance is big) and (Orientation is positive_small) then output1 is none", engine_sugeno1));
	ruleBlock->addRule(Rule::parse("if (Distance is big) and (Orientation is positive_medium) then output1 is right_side", engine_sugeno1));
	ruleBlock->addRule(Rule::parse("if (Distance is big) and (Orientation is positive_big) then output1 is right_side", engine_sugeno1));

	ruleBlock->addRule(Rule::parse("if (Distance is very_big) and (Orientation is negative_big) then output1 is left_side", engine_sugeno1));
	ruleBlock->addRule(Rule::parse("if (Distance is very_big) and (Orientation is negative_medium) then output1 is left_side", engine_sugeno1));
	ruleBlock->addRule(Rule::parse("if (Distance is very_big) and (Orientation is negative_small) then output1 is left_side", engine_sugeno1));
	ruleBlock->addRule(Rule::parse("if (Distance is very_big) and (Orientation is zero) then output1 is right_side", engine_sugeno1));
	ruleBlock->addRule(Rule::parse("if (Distance is very_big) and (Orientation is positive_small) then output1 is right_side", engine_sugeno1));
	ruleBlock->addRule(Rule::parse("if (Distance is very_big) and (Orientation is positive_medium) then output1 is right_side", engine_sugeno1));
	ruleBlock->addRule(Rule::parse("if (Distance is very_big) and (Orientation is positive_big) then output1 is right_side", engine_sugeno1));
	engine_sugeno1->addRuleBlock(ruleBlock);

	std::string status;
	if (not engine_sugeno1->isReady(&status))
		cout << "[engine_mamdani error] engine_mamdani is not ready:" << status << endl;
};

void myFuzzy_01::set_sugeno2() {
	engine_sugeno2 = new Engine;
	engine_sugeno2->setName("Fuzzy1_sugeno");
	engine_sugeno2->setDescription("");

	engine_sugeno2->addInputVariable(moutput1);
	engine_sugeno2->addInputVariable(mHeading);
	engine_sugeno2->addOutputVariable(output2_sugeno);

	RuleBlock *ruleBlock = new RuleBlock;
	ruleBlock->setName("");
	ruleBlock->setDescription("");
	ruleBlock->setEnabled(true);
	ruleBlock->setConjunction(new Minimum);
	ruleBlock->setDisjunction(new Maximum);
	ruleBlock->setImplication(new AlgebraicProduct);
	ruleBlock->setActivation(new General);
	ruleBlock->addRule(Rule::parse("if (Output1 is left_side) and (Heading is negative_front) then output2 is none", engine_sugeno2));
	ruleBlock->addRule(Rule::parse("if (Output1 is left_side) and (Heading is front_left) then output2 is none", engine_sugeno2));
	ruleBlock->addRule(Rule::parse("if (Output1 is left_side) and (Heading is left) then output2 is left_handoff", engine_sugeno2));
	ruleBlock->addRule(Rule::parse("if (Output1 is left_side) and (Heading is back_left) then output2 is left_handoff", engine_sugeno2));
	ruleBlock->addRule(Rule::parse("if (Output1 is left_side) and (Heading is back) then output2 is left_handoff", engine_sugeno2));
	ruleBlock->addRule(Rule::parse("if (Output1 is left_side) and (Heading is back_right) then output2 is right_handoff", engine_sugeno2));
	ruleBlock->addRule(Rule::parse("if (Output1 is left_side) and (Heading is right) then output2 is right_handoff", engine_sugeno2));
	ruleBlock->addRule(Rule::parse("if (Output1 is left_side) and (Heading is front_right) then output2 is none", engine_sugeno2));
	ruleBlock->addRule(Rule::parse("if (Output1 is left_side) and (Heading is positive_front) then output2 is none", engine_sugeno2));

	ruleBlock->addRule(Rule::parse("if (Output1 is none) and (Heading is negative_front) then output2 is none", engine_sugeno2));
	ruleBlock->addRule(Rule::parse("if (Output1 is none) and (Heading is front_left) then output2 is none", engine_sugeno2));
	ruleBlock->addRule(Rule::parse("if (Output1 is none) and (Heading is left) then output2 is none", engine_sugeno2));
	ruleBlock->addRule(Rule::parse("if (Output1 is none) and (Heading is back_left) then output2 is none", engine_sugeno2));
	ruleBlock->addRule(Rule::parse("if (Output1 is none) and (Heading is back) then output2 is none", engine_sugeno2));
	ruleBlock->addRule(Rule::parse("if (Output1 is none) and (Heading is back_right) then output2 is none", engine_sugeno2));
	ruleBlock->addRule(Rule::parse("if (Output1 is none) and (Heading is right) then output2 is none", engine_sugeno2));
	ruleBlock->addRule(Rule::parse("if (Output1 is none) and (Heading is front_right) then output2 is none", engine_sugeno2));
	ruleBlock->addRule(Rule::parse("if (Output1 is none) and (Heading is positive_front) then output2 is none", engine_sugeno2));

	ruleBlock->addRule(Rule::parse("if (Output1 is right_side) and (Heading is negative_front) then output2 is none", engine_sugeno2));
	ruleBlock->addRule(Rule::parse("if (Output1 is right_side) and (Heading is front_left) then output2 is none", engine_sugeno2));
	ruleBlock->addRule(Rule::parse("if (Output1 is right_side) and (Heading is left) then output2 is left_handoff", engine_sugeno2));
	ruleBlock->addRule(Rule::parse("if (Output1 is right_side) and (Heading is back_left) then output2 is left_handoff", engine_sugeno2));
	ruleBlock->addRule(Rule::parse("if (Output1 is right_side) and (Heading is back) then output2 is right_handoff", engine_sugeno2));
	ruleBlock->addRule(Rule::parse("if (Output1 is right_side) and (Heading is back_right) then output2 is right_handoff", engine_sugeno2));
	ruleBlock->addRule(Rule::parse("if (Output1 is right_side) and (Heading is right) then output2 is right_handoff", engine_sugeno2));
	ruleBlock->addRule(Rule::parse("if (Output1 is right_side) and (Heading is front_right) then output2 is none", engine_sugeno2));
	ruleBlock->addRule(Rule::parse("if (Output1 is right_side) and (Heading is positive_front) then output2 is none", engine_sugeno2));
	engine_sugeno2->addRuleBlock(ruleBlock);

	std::string status;
	if (not engine_sugeno2->isReady(&status))
		cout << "[engine_mamdani error] engine_mamdani is not ready:" << status << endl;
};

