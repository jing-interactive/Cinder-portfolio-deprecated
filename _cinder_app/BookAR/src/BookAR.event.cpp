#include "BookAR.h"
#include "UI/UIElement.h"
#include <boost/foreach.hpp>

void BookAR::mouseDown( MouseEvent event )
{
	BOOST_FOREACH(shared_ptr<UIElement> e, _buttons)
	{
		e->mouseDown(event);
	}

	BOOST_FOREACH(shared_ptr<UIElement> e, _thumbs)
	{
		e->mouseDown(event);
	}
}

void BookAR::mouseMove( MouseEvent event )
{
	BOOST_FOREACH(shared_ptr<UIElement> e, _buttons)
	{
		e->mouseMove(event);
	}

	BOOST_FOREACH(shared_ptr<UIElement> e, _thumbs)
	{
		e->mouseMove(event);
	}
}

void BookAR::keyDown( KeyEvent event )
{
// 	if (event.getChar() == 'f' )
// 		setFullScreen( ! isFullScreen() 

	if (event.getChar() == KeyEvent::KEY_ESCAPE)
		quit();
}
