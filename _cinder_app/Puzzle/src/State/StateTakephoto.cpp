#include "States.h"
#include "PuzzleApp.h"
#include "cinder/Utilities.h"
#include "cinder/Timeline.h"
#include "cinder/ImageIo.h"
#include "Sprite.h"
#include "Hand.h"

string playerName("test.jpg.png");

void setPlayerName( const string& str )
{
	playerName = str;
}

namespace
{
	Vec2f pos;
	Color8u clr(255,255,255);
	const int sec_show_count = 5;
	const int n_countdown = 15;
	string welcome = toUtf8(L"∫œ”∞¡ÙƒÓ");
	Anim<float> alpha;
	shared_ptr<Sprite> player;
	const int PHOTO_W = 640;
	const int PHOTO_H = 480;
}

void StateTakephoto::enter()
{
	resetTimer();
	pos.set(_app.getWindowSize()/2);
	timeline().apply(&alpha, 1.0f, 0.0f, 4);
	Sprite* spr = Sprite::createFromImage(
		loadImage(_app.getAppPath()/"photo"/playerName), 
		pos.x, pos.y, PHOTO_W, PHOTO_H);
	player = shared_ptr<Sprite>(spr);
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
	gl::drawStringCentered(welcome, pos, ColorA(1,1,1,alpha), _app.fnt_big);

	int ct = 10 - (getElapsedSeconds()-sec_show_count);
	if (ct <= 10 && ct > 0)
		gl::drawStringCentered(toString(ct), Vec2i(80,40), ColorA(1,1,1), _app.fnt_big);

	if (_app._hands[RIGHT]->state != Hand::NORMAL)
	{
		resetTimer();//we are active

		if (abs(_app._rotate) > FLT_EPSILON)//rotation
		{
			player->_degree.value() += _app._rotate;
			_app._rotate = 0;//reset
			player->_scale = lerp(player->_scale, _app._scale, 0.2f);
		}
		else
		{
			//translation
			player->_center = lerp<Vec2f>(player->_center.value(), _app._hands[RIGHT]->pos, 0.15f);
		}
	}
	player->draw();

	if (ct == 0)
	{//at the last second
		_app._img_sharing = _app.copyWindowSurface();
	}
}

void StateTakephoto::exit()
{

}
