#include "LedState.h"
#include "States.h"
#include "LedMatrixApp.h"
#include "LedManager.h"
#include "Config.h"

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
		st = new StateLotsOfLines(app, dev_id);
		break;
	case T_RIPPLE:
		st = new StateRipple(app, dev_id);
		break;
	case T_FOLLOWING:
		st = new StateFollowingLines(app, dev_id);
		break;
	case T_SPARK_INT:
		st = new StateSparkInteractive(app, dev_id);
		break;
	case T_SPARK:
		st = new StateSpark(app, dev_id);
		break;
	case T_0:
	case T_1:
		{
			int idx = typ - T_0;
			st = new StateSuper(app, dev_id);
			st->_type = typ;
			((StateSuper*)st)->setIndex(idx);
		}break;
	default:
		break;
	}
	return st;
}

bool LedState::isIdleState( StateType typ )
{
	return typ == T_BREATHE || typ == T_LOTS || typ == T_SPARK || typ == T_RIPPLE || typ == T_0 ||typ == T_1;
}

void LedState::update()
{
	if (getElapsedSeconds() > n_countdown && inner_state != T_DYING)//using ! and T_DYING = -1
	{
//		ci::Tween<float>::Options option = LedManager::get(_dev_id).fadeOut(SEC_FADE_OUT);
		inner_state = T_DYING;

		if (LedState::isIdleState(_type))
		{
			_app.changeToRandomIdleState(_dev_id);
//			option = option.finishFn(std::bind(&LedMatrixApp::changeToRandomIdleState, &_app, _dev_id));
		}
		else
		{
		//	option = option.finishFn(std::bind(&LedMatrixApp::changeToRandomInteractiveState, &_app, _dev_id));
		}
	}
}

LedState::LedState( LedMatrixApp& app, int dev_id, StateType type ) 
:State<LedMatrixApp>(app),
_dev_id(dev_id), 
_type(type), 
inner_state(T_RUNNING),
n_countdown(DEFAULT_COUNTDOWN)
{

}
