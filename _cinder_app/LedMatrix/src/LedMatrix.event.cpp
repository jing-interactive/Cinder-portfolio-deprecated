#include "LedMatrixApp.h"
#include <cinder/MayaCamUI.h>

void LedMatrixApp::mouseDown( MouseEvent event )
{
	maya_cam->mouseDown( event.getPos() );
}

void LedMatrixApp::mouseDrag( MouseEvent event )
{
	maya_cam->mouseDrag( event.getPos(), event.isLeftDown(), event.isMiddleDown(), event.isRightDown());
}

void LedMatrixApp::keyUp( KeyEvent event )
{
	if (event.getCode() == KeyEvent::KEY_SPACE)
		show_3d = !show_3d;
}
