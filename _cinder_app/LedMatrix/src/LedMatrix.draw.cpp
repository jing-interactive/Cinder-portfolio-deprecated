#include <cinder/MayaCamUI.h>
#include "LedMatrixApp.h"
#include "LedManager.h"
#include "LedState.h"

void LedMatrixApp::draw()
{
	gl::enableDepthWrite();
	gl::enableDepthRead();
	gl::enableAlphaBlending();
	gl::clear();
	gl::setMatrices( maya_cam->getCamera() );

	gl::pushModelView();
	gl::scale(Vec3f(20,20,20));
	gl::drawCoordinateFrame();
	gl::popModelView();

	for (int i=0;i<2;i++)
	{
		current_states[i]->draw();

		LedManager::get(i).draw3d();
		LedManager::get(i).draw2d();
	}
}