#include "BookARApp.h"
#include "UIElement.h"
#include <boost/foreach.hpp>

void BookARApp::mouseDown( MouseEvent event )
{
	BOOST_FOREACH(shared_ptr<UIElement> e, _buttons)
	{
		e->mouseDown(event);
	}
}

void BookARApp::mouseMove( MouseEvent event )
{
	BOOST_FOREACH(shared_ptr<UIElement> e, _buttons)
	{
		e->mouseMove(event);
	}
}

void BookARApp::keyDown( KeyEvent event )
{
// 	if (event.getChar() == 'f' )
// 		setFullScreen( ! isFullScreen() 

	if (event.getChar() == KeyEvent::KEY_ESCAPE)
		quit();
}
