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
};

struct StateLotsOfLines: public LedState
{
	StateLotsOfLines(LedMatrixApp& app, int dev_id)
		:LedState(app, dev_id, T_LOTS){}
	void enter();
	void update();
	void draw();
	void exit();
	struct Line* lines;
};

struct StateSpark: public LedState
{
	StateSpark(LedMatrixApp& app, int dev_id)
		:LedState(app, dev_id, T_SPARK){}
	void enter();
	void update();
	void draw();
	void exit();
	struct Spark* sparks;
};

struct StateFollowingLines: public LedState
{
	StateFollowingLines(LedMatrixApp& app, int dev_id)
		:LedState(app, dev_id, T_FOLLOWING){}
	void enter();
	void update();
	void draw();
	void exit();
	struct Line* lines;
};

struct StateAnimal: public LedState
{
	StateAnimal(LedMatrixApp& app, int dev_id)
		:LedState(app, dev_id, T_ANIMAL){}
	void enter();
	void update();
	void draw();
	void exit();
};

struct StateSparkInteractive: public LedState
{
	StateSparkInteractive(LedMatrixApp& app, int dev_id)
		:LedState(app, dev_id, T_SPARK_INT){}
	void enter();
	void update();
	void draw();
	void exit();
	struct Spark* sparks;
};
