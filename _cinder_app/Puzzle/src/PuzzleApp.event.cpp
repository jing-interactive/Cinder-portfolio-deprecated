#include "PuzzleApp.h"
#include <boost/foreach.hpp>
#include "Sprite.h"
#include "Hand.h"

namespace
{
	static bool verbose = false;
}

void PuzzleApp::mouseMove( MouseEvent event )
{
	if (verbose) 
		console() << "move" <<endl;
	_hands[RIGHT]->pos.set(event.getPos());
	_hands[RIGHT]->state = Hand::NORMAL;
}

void PuzzleApp::mouseDrag( MouseEvent event )
{
	if (verbose) 
		console() << "drag" <<endl;
	_hands[RIGHT]->state = Hand::DRAG;
	_hands[RIGHT]->pos.set(event.getPos());
}

void PuzzleApp::mouseDown( MouseEvent event )
{
	if (verbose) 
		console() << "down" <<endl;
	_hands[RIGHT]->pos = event.getPos();
	_hands[RIGHT]->state = Hand::CLICK;
}

void PuzzleApp::mouseUp( MouseEvent event )
{
	if (verbose)
		_hands[RIGHT]->state = Hand::NORMAL;
}

void PuzzleApp::mouseWheel( MouseEvent event )
{
	if (verbose) 
		console() << event.getWheelIncrement() <<endl;
	_rotate = event.getWheelIncrement();
}

void PuzzleApp::keyDown( KeyEvent event )
{
	if (event.getChar() == KeyEvent::KEY_ESCAPE)
		quit();
	if (event.getChar() == KeyEvent::KEY_BACKSPACE)
		changeToState(_current_state);
	if (event.getChar() == KeyEvent::KEY_f)
		setFullScreen(!isFullScreen());
	if (event.getChar() == KeyEvent::KEY_1)
		_hands[LEFT]->state = Hand::CLICK;
	if (event.getChar() == KeyEvent::KEY_2)
		_hands[LEFT]->state = Hand::NORMAL;
}
