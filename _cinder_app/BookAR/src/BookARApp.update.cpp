#ifdef USING_ARTK
#include <ARToolKitPlus/TrackerSingleMarker.h>
#endif

#include "BookARApp.h"
#include "cinder/ip/Grayscale.h"
#include "../../../_common/SDAR/SDAR.h"

float BookARApp::cameraXToScreenX(float cx)
{
	return cx*2.0f/CAM_W-1.0f;
}

float BookARApp::cameraYToScreenY(float cy)
{
	return 1.0f-cy*2.0f/CAM_H;
}

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
			int numActiveTrackables = SDARTrack(frame_clr.getData(), _capture.getWidth()*3);
			{
				std::lock_guard<mutex> lock(_mtx_ar);
				_n_trackables = numActiveTrackables;
				if (_n_trackables > 0)
				{
					sin_counter += 0.5f;
					unsigned int obj_id = getActiveTrackableID(0);
					updateData(_img_posters[obj_id], _mesh_book, 200*sinf(sin_counter));
					console() << _n_trackables << std::endl;

					_mat_proj = Matrix44d(getProjectionMatrix(_proj_near,_proj_far));

					for (int tid = 0;tid<_n_trackables;tid++)
					{
						_mat_modelview = Matrix44d(getModelViewMatrix(tid));

						for(int i=0; i<4; i++)
						{
							_pts_corner[i].x = getVertexX(tid, i);
							_pts_corner[i].y = getVertexY(tid, i);
						}
					}
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
