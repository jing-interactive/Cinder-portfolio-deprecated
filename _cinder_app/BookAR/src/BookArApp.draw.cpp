#include "BookARApp.h"

void BookARApp::draw()
{
	// clear out the window with black
	gl::clear( Color( 0, 0, 0 ) ); 

	if ( _tex_bg ) 
	{
		gl::disableDepthWrite();
		gl::pushMatrices();
		gl::setMatricesWindow(getWindowSize());
		gl::color(Color8u(255,255,255));
		gl::draw( _tex_bg, getWindowBounds() );
		gl::popMatrices();
	}

	if (_n_trackables > 0)
	{
		std::lock_guard<mutex> lock(_mtx_ar);
		// now draw 3D cube at marker location with proper scale
		gl::enableDepthWrite();
		gl::pushMatrices();						
		glMatrixMode( GL_PROJECTION );
		glLoadMatrixd(_mat_proj);
		glMatrixMode( GL_MODELVIEW );
		glLoadMatrixd( _mat_modelview );
		// scale is 80mm
		// make cube resting on top of marker instead of inside
		glDisable(GL_TEXTURE_2D);
		gl::drawCube(Vec3f(0.0f,0.0f,0.0f), Vec3f(_cube_scale,_cube_scale,_cube_scale));
		gl::popMatrices();

		gl::pushMatrices();
		_tex_android.enableAndBind();
		glBegin(GL_QUADS);
		glTexCoord2f(0,0);
		//glColor3f(1,0,0);
		glVertex3f(cameraXToScreenX(_pts_corner[0].x),cameraYToScreenY(_pts_corner[0].y),1);
		glTexCoord2f(1,0);
		//glColor3f(0,1,0);
		glVertex3f(cameraXToScreenX(_pts_corner[1].x),cameraYToScreenY(_pts_corner[1].y),1);
		glTexCoord2f(1,1);
		//glColor3f(0,0,1);
		glVertex3f(cameraXToScreenX(_pts_corner[2].x),cameraYToScreenY(_pts_corner[2].y),1);
		glTexCoord2f(0,1);
		//glColor3f(1,1,1);
		glVertex3f(cameraXToScreenX(_pts_corner[3].x),cameraYToScreenY(_pts_corner[3].y),1);
		glEnd();
		gl::popMatrices();
	}

	params::InterfaceGl::draw();
}
