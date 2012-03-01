#include "BookAR.h"
#include "State/States.h"

void BookAR::setupStates()
{
	_state_tracking	= shared_ptr<State<BookAR>>(new StateTracking(*this));
	_state_creating	= shared_ptr<State<BookAR>>(new StateCreating(*this));
	_state_sharing	= shared_ptr<State<BookAR>>(new StateSharing(*this));

	_global_state = shared_ptr<State<BookAR>>(new StateGlobal(*this));
}
