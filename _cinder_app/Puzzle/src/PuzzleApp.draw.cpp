#include "PuzzleApp.h"
#include "State.h"
#include "Hand.h"

void PuzzleApp::draw()
{
	gl::enableAlphaBlending();
	gl::clear( Color( 0.1f, 0.1f, 0.1f ) );
	gl::setMatricesWindow( getWindowSize());

	StateMachine::draw();

	for (int i=0;i<N_HANDS;i++)
	{
		_hands[i]->draw();
	}
}