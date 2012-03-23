#include <cinder/MayaCamUI.h>
#include "LedMatrixApp.h"
#include "LedManager.h"
#include "LedState.h"
#include "Config.h"

void LedMatrixApp::draw()
{
	gl::clear();

	if (show_3d)
	{
		gl::enableDepthWrite();
		gl::enableDepthRead();
		gl::enableAlphaBlending();
		gl::setMatrices( maya_cam->getCamera() );
		gl::setViewport(Area(100,0, getWindowWidth(), getWindowHeight()));

		gl::pushModelView();
		gl::scale(Vec3f(20,20,20));
		gl::drawCoordinateFrame();
		gl::popModelView();

		//all the led state draws nothing
		for (int i=0;i<2;i++)
			LedManager::get(i).draw3d();
	}

	{//2d mapping
		gl::enableAlphaBlending();
		gl::disableDepthRead();
		gl::setMatricesWindow(getWindowSize());

		for (int dev=0;dev<2;dev++)
		{
			bool idle = LedState::isIdleState(current_states[dev]->_type);
			if (current_states[dev]->_type == T_0 ||current_states[dev]->_type == T_1)
			{
			//	gl::disableAlphaBlending();
				LedManager::get(dev).draw2d(getElapsedSeconds(), !idle, true);
			}
			else
			{
				LedManager::get(dev).draw2d(getElapsedSeconds());
			}
		}
	}
}