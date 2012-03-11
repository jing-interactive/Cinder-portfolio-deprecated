#pragma once

#include "../../../_common/State.h"

class LedMatrixApp;

enum StateType
{
	T_INVALID,
	//idle
	T_BREATHE,
	T_LOTS,
	T_SPARK,
	//interactive
	T_FOLLOWING,
	T_ANIMAL,
	T_SPARK_INT,
};

struct LedState : public State<LedMatrixApp>
{
	static LedState* create(LedMatrixApp& app, int dev_id, StateType typ);
	static bool isIdleState(StateType typ);
	LedState(LedMatrixApp& app, int dev_id, StateType type)
		:State<LedMatrixApp>(app),_dev_id(dev_id), _type(type){}
	int _dev_id;
	StateType _type;
protected:
	double _time;
	double getElapsedSeconds();
	void resetTimer();
};
