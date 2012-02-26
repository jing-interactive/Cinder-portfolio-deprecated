#include "PuzzleApp.h"
#include "State.h"

void Hand::draw()
{
	if (push)
	{
		gl::color(Color8u(100,255,0));
		gl::drawSolidCircle(pos, 20);
	}
	gl::color(Color8u(255,0,0));
	gl::drawSolidCircle(pos, 15);
}

void PuzzleApp::draw()
{
	gl::enableAlphaBlending();
	gl::clear( Color( 0.1f, 0.1f, 0.1f ) );
	gl::setMatricesWindow( getWindowSize());

	_current_state->draw();

	for (int i=0;i<N_HANDS;i++)
	{
		_hands[i].draw();
	}
}