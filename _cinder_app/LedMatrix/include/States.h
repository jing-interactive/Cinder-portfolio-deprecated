#pragma once

#include "LedState.h"

struct StateBreathe: public LedState
{
	StateBreathe(LedMatrixApp& app, int dev_id)
		:LedState(app, dev_id, T_BREATHE){}
	void enter();
	void update();
	void draw();
	void exit();
private:
	struct BreatheLine* items;
	float breathe_decay; 
	float sin_value;//
	float sin_counter;
	float speed;// 
	float min_br;
};

struct StateLotsOfLines: public LedState
{
	StateLotsOfLines(LedMatrixApp& app, int dev_id)
		:LedState(app, dev_id, T_LOTS){}
	void enter();
	void update();
	void draw();
	void exit();
private:
	struct Line* items;
};

struct StateRipple: public LedState
{
	StateRipple(LedMatrixApp& app, int dev_id)
		:LedState(app, dev_id, T_RIPPLE){}
	void enter();
	void update();
	void draw();
	void exit();
};

struct StateCube: public LedState
{
	StateCube(LedMatrixApp& app, int dev_id)
		:LedState(app, dev_id, T_CUBE){}
	void enter();
	void update();
	void draw();
	void exit();
};

struct StateSpark: public LedState
{
	StateSpark(LedMatrixApp& app, int dev_id)
		:LedState(app, dev_id, T_SPARK){}
	void enter();
	void update();
	void draw();
	void exit();
private:
	struct Spark* items;
};

struct StateFollowingLines: public LedState
{
	StateFollowingLines(LedMatrixApp& app, int dev_id)
		:LedState(app, dev_id, T_FOLLOWING){}
	void enter();
	void update();
	void draw();
	void exit();
private:
	struct Line* items;
};

struct StateAnimal: public LedState
{
	StateAnimal(LedMatrixApp& app, int dev_id)
		:LedState(app, dev_id, T_ANIMAL){}
	void enter();
	void update();
	void draw();
	void exit();
private:
	struct Animal* items;
};

struct StateSparkInteractive: public LedState
{
	StateSparkInteractive(LedMatrixApp& app, int dev_id)
		:LedState(app, dev_id, T_SPARK_INT){}
	void enter();
	void update();
	void draw();
	void exit();
private:
	struct Spark* items;
};

struct StateSuper: public LedState
{
	StateSuper(LedMatrixApp& app, int dev_id)
		:LedState(app, dev_id, T_SPARK_INT){}
	void enter();
	void update();
	void draw();
	void exit();
	void setIndex(int idx);
private:
	int idx;
};
