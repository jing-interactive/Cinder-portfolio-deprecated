#include "States.h"
#include "PuzzleApp.h"
#include "cinder/Utilities.h"
#include "cinder/Timeline.h"

namespace
{
	Vec2f pos;
	Color8u clr(255,255,255);
	Font font;
	const int n_countdown = 20;
	string welcome = toUtf8(L"∫œ”∞¡ÙƒÓ");
	Anim<float> alpha;
}

void StateTakephoto::enter()
{
	resetTimer();
	pos.set(_app.getWindowSize()/2);
	if (!font)
		font = Font("STHupo", 64);
	timeline().apply(&alpha, 1.0f, 0.0f, 4);
}

void StateTakephoto::update()
{
	if (getElapsedSeconds() > n_countdown)
		_app.changeToState(_app._state_sharepic);
}

void StateTakephoto::draw()
{
	gl::color(1,1,1);
	gl::draw(_app._tex_selected, _app.getWindowBounds());
	gl::drawStringCentered(welcome, pos, ColorA(1,1,1,alpha), font);
}

void StateTakephoto::exit()
{

}
