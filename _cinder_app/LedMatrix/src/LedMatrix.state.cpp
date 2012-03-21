#include <cinder/Rand.h>
#include "LedMatrixApp.h"
#include "LedState.h"
#include "LedManager.h"
#include "Config.h"

void LedMatrixApp::changeToState(LedState* new_state)
{
	assert(new_state != NULL);
	int dev = new_state->_dev_id;
	if (current_states[dev])
	{
		current_states[dev]->exit();
	}
	current_states[dev] = shared_ptr<LedState>(new_state);
	LedManager::get(dev).resetAlpha();
	current_states[dev]->enter();
}

void LedMatrixApp::setupStates()
{
	for (int dev=0;dev<2;dev++)
	{
		changeToRandomIdleState(dev);
//		LedManager::get(dev).fadeIn(SEC_RIPPLE_FADEIN);
//		LedManager::get(dev).k_alpha = 0.0f;
	}
}

void LedMatrixApp::changeToRandomIdleState( int dev )
{
#if 1
	static StateType idle_states[] = {T_0, T_1, T_SPARK/*,T_LOTS, T_RIPPLE*/};
	changeToStateAmong(dev, idle_states, _countof(idle_states));
#else
	changeToState(LedState::create(*this, dev, T_LOTS));
#endif
	assert(LedState::isIdleState(current_states[dev]->_type));
}

void LedMatrixApp::changeToRandomInteractiveState( int dev )
{
#if 0
	static StateType interative_states[] = {T_FOLLOWING,T_SPARK_INT,T_ANIMAL};
	changeToStateAmong(dev, interative_states, _countof(interative_states));
#else
	changeToState(LedState::create(*this, dev, T_ANIMAL));
#endif
	assert(!LedState::isIdleState(current_states[dev]->_type));
}

void LedMatrixApp::changeToStateAmong( int dev, StateType* state_types, int n_types)
{ 
	StateType current = T_INVALID;
	if (current_states[dev])
		current = current_states[dev]->_type;
	int next = randInt(n_types);
	while (state_types[next] == current)
	{
		next = randInt(n_types);
	}
	LedState* new_st = LedState::create(*this, dev, state_types[next]);
	if (new_st)
		changeToState(new_st);
} 
