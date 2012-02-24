#ifdef USING_ARTK
#include <ARToolKitPlus/TrackerSingleMarker.h>
#endif
#include "BookARApp.h"
#include "cinder/ip/Grayscale.h"
#include "ARTracker.h"

#if 0
float BookARApp::cameraXToScreenX(float cx)
{
	const int APP_W_2 = APP_W/2;
	float temp = lmap<float>(cx, 0, CAM_W, -(APP_W_2-SPAC_LEFT)/APP_W_2, (APP_W_2-SPAC_RIGHT)/APP_W_2);
	return temp;
}

float BookARApp::cameraYToScreenY(float cy)
{
	const int APP_H_2 = APP_H/2;
	float temp = lmap<float>(cy, 0, CAM_H, (APP_H_2-SPAC_UP)/APP_H_2, -(APP_H_2-SPAC_DOWN)/APP_H_2);
	return temp;
}
#else
float BookARApp::cameraXToScreenX(float cx)
{
	return lmap<float>(cx, 0, CAM_W, -1, 1);
}

float BookARApp::cameraYToScreenY(float cy)
{
	return lmap<float>(cy, 0, CAM_H, 1, -1);
}
#endif

void BookARApp::updateData(const ci::Surface32f& image, gl::VboMesh& mesh, float max_height)
{
	uint32_t book_w = image.getWidth();
	uint32_t book_h = image.getHeight();

	if (!mesh || mesh.getNumVertices() != book_w * book_h)
	{//needs refresh
		gl::VboMesh::Layout layout;
		layout.setDynamicColorsRGB();
		layout.setDynamicPositions();
		_mesh_book = gl::VboMesh( book_w * book_h, 0, layout, GL_POINTS );
	}
	Surface32f::ConstIter pixelIter = image.getIter();
	gl::VboMesh::VertexIter vertexIter( mesh );

	while( pixelIter.line() ) {
		while( pixelIter.pixel() ) {
			Color color( pixelIter.r(), pixelIter.g(), pixelIter.b() );
			float height = color.dot( Color( 0.3333f, 0.3333f, 0.3333f ) );

			// the x and the z coordinates correspond to the pixel's x & y
			float x = pixelIter.x() - book_h / 2.0f;
			float z = pixelIter.y() - book_h / 2.0f;

			vertexIter.setPosition( x, height * max_height, z );
			vertexIter.setColorRGB( color );
			++vertexIter;
		}
	}
}

void BookARApp::update()
{	
	static float sin_counter = 0.0f;
	if( _capture && _capture.checkNewFrame() ) 
	{
		Surface8u& frame_clr = _capture.getSurface();
		Channel8u frame_gray = Channel8u( _capture.getWidth(), _capture.getHeight() );

		ip::grayscale( frame_clr, &frame_gray );

		if (_using_sdar)
		{
			_ar_tracker->update(frame_clr.getData());
			int numActiveTrackables = _ar_tracker->getNumTracked();
			{
				std::lock_guard<mutex> lock(_mtx_ar);
				_n_trackables = numActiveTrackables;
				if (_n_trackables > 0)
				{
					sin_counter += 0.5f;
					_obj_id = _ar_tracker->getID(0);

					updateData(_img_posters[_obj_id], _mesh_book, 200*sinf(sin_counter));
					console() << _n_trackables << std::endl;

					_ar_tracker->getProjectionMat(_mat_proj);

					_ar_tracker->getModelViewMat(0, _mat_modelview);
					_ar_tracker->getCorners(0, _pts_corner); 
				}
			}
		}
#ifdef USING_ARTK
		else
		{
			ARToolKitPlus::ARMarkerInfo* m_infos;
			int numActiveTrackables;
			std::vector<int> markerId = _artk_tracker->calc(frame_gray.getData(), &m_infos, &numActiveTrackables);

			int best_id = _artk_tracker->selectBestMarkerByCf();
			{
				std::lock_guard<mutex> lock(_mtx_ar);
				_n_trackables = numActiveTrackables;

				for (int j = 0; j < numActiveTrackables; j++) 
				{
					if (m_infos[j].id == best_id)
					{		
						for(int i=0; i<4; i++) 
							_pts_corner[i] = ci::Vec2f(m_infos[j].vertex[i][0], m_infos[j].vertex[i][1]);
						_mat_proj = Matrix44d(_artk_tracker->getProjectionMatrix());
						_mat_modelview = Matrix44d(_artk_tracker->getModelViewMatrix());
						break;
					}
				}
			}
		}
#else
		else
		{
			_n_trackables = 0;
		}
#endif
		if (_n_trackables == 0)
			sin_counter = 0;
		_tex_bg = gl::Texture(frame_clr);
	}
}
