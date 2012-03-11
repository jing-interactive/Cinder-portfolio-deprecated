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

struct StateLotsOfStars: public LedState
{
	StateLotsOfStars(LedMatrixApp& app, int dev_id)
		:LedState(app, dev_id, T_LOTS){}
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
	struct Spark* sparks;
};

struct StateFollowingStars: public LedState
{
	StateFollowingStars(LedMatrixApp& app, int dev_id)
		:LedState(app, dev_id, T_FOLLOWING){}
	void enter();
	void update();
	void draw();
	void exit();
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
