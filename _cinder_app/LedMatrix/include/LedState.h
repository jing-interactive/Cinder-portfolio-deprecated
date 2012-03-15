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
	T_RIPPLE,
	T_CUBE,
	//interactive
	T_FOLLOWING,
	T_ANIMAL,
	T_SPARK_INT,
};

enum{
	T_RUNNING,
	T_DYING = -1,
};

struct LedState : public State<LedMatrixApp>
{
	static LedState* create(LedMatrixApp& app, int dev_id, StateType typ);
	static bool isIdleState(StateType typ);
	int _dev_id;
	StateType _type;
	virtual void update();
protected:
	LedState(LedMatrixApp& app, int dev_id, StateType type);
	int n_countdown;
	double _time;
	double getElapsedSeconds();
	void resetTimer();
	int inner_state;
};
