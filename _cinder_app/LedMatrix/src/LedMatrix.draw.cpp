#include <cinder/MayaCamUI.h>
#include "LedMatrixApp.h"
#include "LedManager.h"
#include "LedState.h"

void LedMatrixApp::draw()
{
	gl::clear();

	if (show_3d)
	{
		gl::enableDepthWrite();
		gl::enableDepthRead();
		gl::enableAlphaBlending();
		gl::setMatrices( maya_cam->getCamera() );

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

		for (int i=0;i<2;i++)
			LedManager::get(i).draw2d(getElapsedSeconds());
	}
}