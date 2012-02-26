#include "PuzzleApp.h"
#include <boost/foreach.hpp>
#include "Sprite.h"

void PuzzleApp::mouseMove( MouseEvent event )
{
	console() << "move" <<endl;
	_hands[RIGHT].pos.set(event.getPos());
	_hands[RIGHT].push = false;
}

void PuzzleApp::mouseDrag( MouseEvent event )
{
	_hands[RIGHT].pos.set(event.getPos());
	console() << "drag" <<endl;
}

void PuzzleApp::mouseDown( MouseEvent event )
{
	console() << "down" <<endl;
	_hands[RIGHT].pos = event.getPos();
	_hands[RIGHT].push = true;
}

void PuzzleApp::mouseUp( MouseEvent event )
{
	_hands[RIGHT].push = false;
	if (_sprite_selected)
		_sprite_selected.reset();
}

void PuzzleApp::mouseWheel( MouseEvent event )
{
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
}
