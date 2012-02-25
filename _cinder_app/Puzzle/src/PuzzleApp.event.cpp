#include "PuzzleApp.h"

using namespace ci;
using namespace ci::app;
using namespace std;

void PuzzleApp::mouseMove( MouseEvent event )
{
	_hands[RIGHT].pos.set(event.getPos());
	_hands[RIGHT].push = false;
}

void PuzzleApp::mouseDown( MouseEvent event )
{
//	_pos_cursor.set(event.getPos());
	_hands[RIGHT].push = true;
}

void PuzzleApp::mouseWheel( MouseEvent event )
{
	console() << event.getWheelIncrement() <<endl;
}

void PuzzleApp::keyDown( KeyEvent event )
{
	if (event.getChar() == VK_ESCAPE)
		quit();
}
