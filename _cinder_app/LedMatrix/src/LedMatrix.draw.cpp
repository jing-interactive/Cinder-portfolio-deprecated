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

	//TODO: support two device
	for (int i=0;i<1;i++)
	{
		current_states[i]->draw();

		LedManager::get(i).draw3d();
		LedManager::get(i).draw2d();
	}
}