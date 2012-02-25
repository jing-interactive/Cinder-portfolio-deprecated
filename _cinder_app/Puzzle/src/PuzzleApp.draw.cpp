#include "PuzzleApp.h"
#include "State.h"

void PuzzleApp::draw()
{
	gl::enableAlphaBlending();
	gl::clear( Color( 0.1f, 0.1f, 0.1f ) );
	gl::setMatricesWindow( getWindowSize());

	gl::color(Color8u(255,0,0));

	for (int i=0;i<N_HANDS;i++)
	{
		gl::drawStrokedCircle(_hands[i].pos, 10);
		if (_hands[i].push)
			gl::drawStrokedCircle(_hands[i].pos, 20);
	}

	_current_state->draw();
}