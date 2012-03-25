#include "States.h"
#include "PuzzleApp.h"
#include "cinder/Utilities.h"
#include <cinder/ImageIo.h>

namespace
{
	Vec2f pos;
	Color8u clr(255,255,255);
	const int n_countdown = 60;
	string welcome = toUtf8(L"’’∆¨∑÷œÌ");
	string icon_names[]={"/UI/sina-weibo.png", "/UI/mobile-icon.png"};
	gl::Texture icons[2];
	const int ICON_W = 128;
	const int ICON_H = 128;
	Vec2i poses[2]={Vec2i(40,100), Vec2i(40,300)};
	Rectf areas[2]={Rectf(poses[0].x, poses[0].y, poses[0].x+ICON_W, poses[0].y+ICON_H), 
		Rectf(poses[1].x, poses[1].y, poses[1].x+ICON_W, poses[1].y+ICON_H)};
}

void StateSharepic::enter()
{
	resetTimer();
	pos.set(_app.getWindowSize()/2);
	for (int i=0;i<2;i++)
	{
		icons[i] = loadImage(_app.getAppPath()/icon_names[i]);
	}
}

void StateSharepic::update()
{
	if (getElapsedSeconds() > n_countdown)
		_app.changeToState(_app._state_idle);
}

void StateSharepic::draw()
{
	gl::color(1,1,1);
	gl::draw(_app._tex_selected, _app.getWindowBounds());
	gl::drawStringCentered(welcome, pos, clr, _app.fnt_big);

	for (int i=0;i<2;i++)
	{
		gl::draw(icons[i], areas[i]);
	}
	
}

void StateSharepic::exit()
{

}
