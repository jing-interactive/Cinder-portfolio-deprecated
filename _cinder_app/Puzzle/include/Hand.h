#include "cinder/color.h"

struct Hand
{
	enum State
	{
		NORMAL,
		CLICK,
		DRAG,
	};
	Hand(const ci::Color8u&);
	void draw();
	ci::Vec2i pos;
	ci::Color8u clr;
	State state;
};