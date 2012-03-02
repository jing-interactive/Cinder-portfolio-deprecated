#include "UIElement.h"
#include "cinder/app/AppBasic.h"
#include "cinder/Timeline.h"

UIElement::UIElement(int id, int x, int y, int width, int height, gl::Texture tex )
:_id(id), 
_tex(tex),
_area(x,y,x+width,y+height),
_pos(x,y),
_state(NORMAL)
{
//	App::get()->registerMouseMove(this, &UIElement::mouseMove);
//	App::get()->registerMouseDown(this, &UIElement::mouseDown);
}

bool UIElement::isPointIn( const Vec2f &pt )
{
	return _area.contains( pt );
}

bool UIElement::mouseMove( app::MouseEvent event )
{
	if (isPointIn(event.getPos()))
		_state = OVER;
	else
		_state = NORMAL;
	return false;
}

bool UIElement::mouseDown( app::MouseEvent event )
{	
	if (isPointIn(event.getPos()))
		_state = CLICK;
	else
		_state = NORMAL;
	return false;
}

void UIElement::draw()
{	
	if (_tex)
	{
		gl::color(1,1,1);
		gl::draw( _tex, _area);
	}
	if (_state != NORMAL)
	{	
		gl::color(0.0, 0.1, 1, 0.2);
		gl::drawSolidRoundedRect(_area, 10);
	}
	if(_state == CLICK)
	{
		gl::color(1,1,1);
		gl::drawStrokedRoundedRect(_area, 10);
	}
}
