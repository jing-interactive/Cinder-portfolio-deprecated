#include "BookARApp.h"

void BookARApp::mouseDown( MouseEvent event )
{
}


void BookARApp::keyDown( KeyEvent event )
{
	if (event.getChar() == 'f' )
		setFullScreen( ! isFullScreen() );

	if (event.getChar() == KeyEvent::KEY_ESCAPE)
		quit();
}
