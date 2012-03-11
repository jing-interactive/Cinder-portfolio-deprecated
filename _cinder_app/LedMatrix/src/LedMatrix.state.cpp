#include "LedMatrixApp.h"
#include "LedState.h"
#include <cinder/Rand.h>

void LedMatrixApp::changeToState(LedState* new_state)
{
	assert(new_state != NULL);
	int id = new_state->_dev_id;
	if (current_states[id])
	{
		current_states[id]->exit();
	}
	current_states[id] = shared_ptr<LedState>(new_state);
	current_states[id]->enter();
}

void LedMatrixApp::setupStates()
{
	for (int i=0;i<2;i++)
	{
		changeToRandomIdleState(i);
	}
}

void LedMatrixApp::changeToRandomIdleState( int dev_id )
{
	static StateType idle_states[3] = {T_SPARK,T_BREATHE,T_LOTS};
	changeToStateAmong(dev_id, idle_states);
}

void LedMatrixApp::changeToRandomInteractiveState( int dev_id )
{
	static StateType interative_states[3] = {T_FOLLOWING,T_SPARK_INT,T_ANIMAL};
	changeToStateAmong(dev_id, interative_states);
}

void LedMatrixApp::changeToStateAmong( int dev_id, StateType state_types[3])
{ 
	StateType current = T_INVALID;
	if (current_states[dev_id])
		current = current_states[dev_id]->_type;
	int next = randInt(3);
	while (state_types[next] == current)
	{
		next = randInt(3);
	}
	LedState* new_st = LedState::create(*this, dev_id, state_types[next]);
	if (new_st)
		changeToState(new_st);
} 
