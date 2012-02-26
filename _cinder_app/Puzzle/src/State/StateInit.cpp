#include "States.h"
#include "PuzzleApp.h"
#include "cinder/Utilities.h"

namespace
{
	string welcome("welcome to Kinect show!");
}

void StateInit::enter()
{
	_app.selectRandomImage();
}

void StateInit::update()
{
	_app.changeToState(_app._state_countdown);
}

void StateInit::draw()
{
	gl::draw(_app._tex_selected);
//	gl::drawStringCentered(welcome, )
}

void StateInit::exit()
{

}
