#include "States.h"
#include "../BookAR.h"
#include "../ARTracker/ARTracker.h"
#include <cinder/Utilities.h>
#include <cinder/ip/Grayscale.h>
#include "../Content/ContentManager.h"
#include "../Content/Content.h"

namespace
{
	float cameraXToScreenX(float cx)
	{
		return lmap<float>(cx, 0, BookAR::CAM_W, -1, 1);
	}

	float cameraYToScreenY(float cy)
	{
		return lmap<float>(cy, 0, BookAR::CAM_H, 1, -1);
	}

	int tid = -1;
}

void StateTracking::enter()
{
	_app._current_content.reset();
}

void StateTracking::update()
{
	static float sin_counter = 0.0f;
	Capture& cap = _app._capture;
	shared_ptr<class ARTracker>& tracker = _app._ar_tracker;

	if( cap && cap.checkNewFrame() ) 
	{
		Surface8u& frame_clr = cap.getSurface();
		Channel8u frame_gray = Channel8u( cap.getWidth(), cap.getHeight() );

		ip::grayscale( frame_clr, &frame_gray );

		int numActiveTrackables = tracker->update(frame_clr.getData());
		{
			std::lock_guard<mutex> lock(_app._mtx_ar);
			_app._n_trackables = numActiveTrackables;
			if (_app._n_trackables > 0)
			{
				sin_counter += 0.5f;
				tid = tracker->getID(0);
				string name = tracker->getName(0);
				_app._current_content = _app._content_mgr->getContentByName(name);
				//_app.updateData(_app._img_posters[tid], _app._mesh_book, 200*sinf(sin_counter));
				console() << name << std::endl;

				tracker->getProjectionMat(_app._mat_proj);

				tracker->getModelViewMat(0, _app._mat_modelview);
				tracker->getCorners(0, _app._pts_corner);
			}
		}
		if (_app._n_trackables == 0)
			sin_counter = 0;
		_app._tex_bg = gl::Texture(frame_clr);
	}
}

void StateTracking::draw()
{
	gl::enableAlphaBlending();
	gl::disableDepthWrite();

	gl::setMatricesWindow(getWindowSize());

	if (_app._capture_visible && _app._tex_bg) 
	{
		gl::color(1,1,1);		gl::draw( _app._tex_bg, _app._area_capture);
	} 

	gl::color(1,1,1);

	if (_app._n_trackables > 0)
	{
		std::lock_guard<mutex> lock(_app._mtx_ar);

		if (_app._2dbook_visible)
		{//rendering.2d
			gl::disableDepthWrite();
			//	gl::setMatricesWindow(Vec2i(CAM_W,CAM_H));

			_app._current_content->getTexture().enableAndBind();			
			//			gl::pushMatrices();			
			{
				glMatrixMode(GL_PROJECTION);
				glLoadIdentity();
				glMatrixMode(GL_MODELVIEW);
				glLoadIdentity();
				gl::setViewport(_app._area_capture + Vec2i(0,93));

				glBegin(GL_QUADS);
				gl::color(1,1,1);
				glTexCoord2f(0.0f, 0.0f);
				glVertex3f(cameraXToScreenX(_app._pts_corner[0].x),cameraYToScreenY(_app._pts_corner[0].y),0.5);
				glTexCoord2f(1.0f, 0.0f);
				glVertex3f(cameraXToScreenX(_app._pts_corner[1].x),cameraYToScreenY(_app._pts_corner[1].y),0.5);
				glTexCoord2f(1.0f, 1.0f);
				glVertex3f(cameraXToScreenX(_app._pts_corner[2].x),cameraYToScreenY(_app._pts_corner[2].y),0.5);
				glTexCoord2f(0.0f, 1.0f);
				glVertex3f(cameraXToScreenX(_app._pts_corner[3].x),cameraYToScreenY(_app._pts_corner[3].y),0.5);
				glEnd();
			}
			//			gl::popMatrices();
			_app._current_content->getTexture().disable();
		}
#ifndef NEW_CONTENT_SYSTEM
		if (_app._3dbook_visible && tid != BookAR::N_MODELS-1)
		{//rendering.3d	
			glMatrixMode( GL_PROJECTION );
			glLoadMatrixd(_app._mat_proj);
			glMatrixMode( GL_MODELVIEW );
			glLoadMatrixd( _app._mat_modelview );
			gl::enableDepthWrite();
			gl::enableDepthRead();
			gl::disableAlphaBlending();

			gl::pushModelView();
			gl::color(Color8u(255,255,255));

			if( _app._mesh_book )
			{
				gl::translate(_app._mesh_translate);
				gl::scale(Vec3f(_app._cube_scale,_app._cube_scale,_app._cube_scale));
				gl::rotate(Vec3f(90,0,0));
				gl::draw( _app._mesh_book );
			}
			gl::popModelView();
		}
#endif
	}
}

void StateTracking::exit()
{

}

void StateTracking::mouseUp( cinder::app::MouseEvent event )
{

}
