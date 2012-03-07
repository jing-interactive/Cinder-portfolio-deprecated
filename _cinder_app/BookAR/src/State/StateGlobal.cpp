#include "States.h"
#include "../BookAR.h"
#include <cinder/Utilities.h>
#include "../UI/UIElement.h"
#include <boost/foreach.hpp>

namespace
{
}

void StateGlobal::enter()
{

}

void StateGlobal::update()
{

}

void StateGlobal::draw()
{
	// clear out the window with black
	gl::clear(); 

	gl::enableAlphaBlending();
	gl::disableDepthWrite();

	gl::setMatricesWindow(getWindowSize());
	gl::color(1,1,1); 
	gl::draw( _app._tex_iphone4, getWindowBounds() );

	BOOST_FOREACH(shared_ptr<UIElement> e, _app._buttons)
	{
		e->draw();
	}
	BOOST_FOREACH(shared_ptr<UIElement> e, _app._thumbs)
	{
		e->draw();
	}
}

void StateGlobal::exit()
{

}
