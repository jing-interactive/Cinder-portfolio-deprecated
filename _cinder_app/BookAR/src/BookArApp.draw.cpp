#include "BookARApp.h"
#include "cinder/params/Params.h"
#include "cinder/gl/gl.h"
#include "UIElement.h"
#include <boost/foreach.hpp>

void BookARApp::draw()
{
	// clear out the window with black
	gl::clear( Color( 0, 0, 0 ) ); 

	gl::enableAlphaBlending();
	gl::disableDepthWrite();

	gl::setMatricesWindow(getWindowSize());
	gl::draw( _tex_iphone4, getWindowBounds() );

	if (_capture_visible && _tex_bg ) 
	{
		gl::color(1,1,1);		gl::draw( _tex_bg, _area_capture);
	} 

	BOOST_FOREACH(shared_ptr<UIElement> e, _buttons)
	{
		e->draw();
	}

	gl::color(1,1,1);

	if (_n_trackables > 0)
	{
		std::lock_guard<mutex> lock(_mtx_ar);

		if (_2dbook_visible)
		{//rendering.2d
			gl::disableDepthWrite();
		//	gl::setMatricesWindow(Vec2i(CAM_W,CAM_H));

			_tex_posters[_obj_id].enableAndBind();			
//			gl::pushMatrices();			
			{
				glMatrixMode(GL_PROJECTION);
				glLoadIdentity();
				glMatrixMode(GL_MODELVIEW);
				glLoadIdentity();
				gl::setViewport(_area_capture + Vec2i(0,-13));

				glBegin(GL_QUADS);
				if (_obj_id == N_MODELS-1)
					gl::color(Color8u(255,255,255));
				else
					gl::color(Color8u(0,0,0));
#if 0
				glTexCoord2f(0.0f, 0.0f);
				glVertex3f(-1,-1,0.5);
				glTexCoord2f(1.0f, 0.0f);
				glVertex3f(1,-1,0.5);
				glTexCoord2f(1.0f, 1.0f);
				glVertex3f(1,1,0.5);
				glTexCoord2f(0.0f, 1.0f);
				glVertex3f(-1,1,0.5);
#else
				glTexCoord2f(0.0f, 0.0f);
				glVertex3f(cameraXToScreenX(_pts_corner[0].x),cameraYToScreenY(_pts_corner[0].y),0.5);
				glTexCoord2f(1.0f, 0.0f);
				glVertex3f(cameraXToScreenX(_pts_corner[1].x),cameraYToScreenY(_pts_corner[1].y),0.5);
				glTexCoord2f(1.0f, 1.0f);
				glVertex3f(cameraXToScreenX(_pts_corner[2].x),cameraYToScreenY(_pts_corner[2].y),0.5);
				glTexCoord2f(0.0f, 1.0f);
				glVertex3f(cameraXToScreenX(_pts_corner[3].x),cameraYToScreenY(_pts_corner[3].y),0.5);
#endif
				glEnd();
			}
//			gl::popMatrices();
			_tex_posters[_obj_id].disable();
		}

		if (_3dbook_visible && _obj_id != N_MODELS-1)
		{//rendering.3d			
#if 1
			glMatrixMode( GL_PROJECTION );
			glLoadMatrixd(_mat_proj);
			glMatrixMode( GL_MODELVIEW );
			glLoadMatrixd( _mat_modelview );
			gl::enableDepthWrite();
			gl::enableDepthRead();
			gl::disableAlphaBlending();

			gl::pushModelView();
			gl::color(Color8u(255,255,255));

			if( _mesh_book )
			{
				gl::translate(_mesh_translate);
				gl::scale(Vec3f(_cube_scale,_cube_scale,_cube_scale));
				gl::rotate(Vec3f(90,0,0));
				gl::draw( _mesh_book );
			}
			gl::popModelView();

#else
			// now draw 3D cube at marker location with proper scale
			gl::enableDepthWrite();
			gl::enableDepthRead();
			gl::pushMatrices();				
			glMatrixMode( GL_PROJECTION );
			glLoadMatrixd(_mat_proj);
			glMatrixMode( GL_MODELVIEW );
			glLoadMatrixd( _mat_modelview );
			gl::BoolState b1(GL_LIGHTING);

			glEnable(GL_LIGHTING);
			glEnable(GL_LIGHT0);

			GLfloat lightPosition[] = { -_light_dir.x, -_light_dir.y, -_light_dir.z, 0.0f };
			glLightfv( GL_LIGHT0, GL_POSITION, lightPosition );
			glMaterialfv( GL_FRONT, GL_DIFFUSE,	_cube_clr );
			gl::color( _cube_clr );
			// scale is 80mm
			// make cube resting on top of marker instead of inside
			//glDisable(GL_TEXTURE_2D);
			gl::drawCube(Vec3f(0.0f,0.0f,0.0f), Vec3f(_cube_scale,_cube_scale,_cube_scale));
			gl::popMatrices();
#endif
		}
	}
	params::InterfaceGl::draw();
}
