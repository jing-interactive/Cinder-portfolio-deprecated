#include "LedState.h"
#include "States.h"
#include "LedMatrixApp.h"

double LedState::getElapsedSeconds()
{
	return _app.getElapsedSeconds() - _time;
}

void LedState::resetTimer()
{
	_time = _app.getElapsedSeconds();
}

LedState* LedState::create(LedMatrixApp& app, int dev_id, StateType typ)
{
	LedState* st = NULL;
	switch (typ)
	{
	case T_ANIMAL:
		st = new StateAnimal(app, dev_id);
		break;
	case T_BREATHE:
		st = new StateBreathe(app, dev_id);
		break;
	case T_LOTS:
		st = new StateLotsOfStars(app, dev_id);
		break;
	case T_FOLLOWING:
		st = new StateFollowingStars(app, dev_id);
		break;
	case T_SPARK_INT:
		st = new StateSpark(app, dev_id);
		break;
	case T_SPARK:
		st = new StateSpark(app, dev_id);
		break;
	default:
		break;
	}
	return st;
}

bool LedState::isIdleState( StateType typ )
{
	return typ == T_ANIMAL || typ == T_LOTS || typ == T_SPARK;
}
