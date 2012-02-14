#include <ARToolKitPlus/TrackerSingleMarker.h>
#include "BookARApp.h"
#include "cinder/ip/Grayscale.h"
#include "../../_common/SDAR/ModelDLL.h"

ci::Vec2f toCinder(const IPoint& rhs)
{
	return ci::Vec2f(rhs.x, rhs.y);;
}

void BookARApp::update()
{	
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
					console() << _n_trackables << std::endl;

					_mat_proj = Matrix44d(getProjectionMatrix(_proj_near,_proj_far));

					for (int tid = 0;tid<_n_trackables;tid++)
					{
						_mat_modelview = Matrix44d(getModelViewMatrix(tid));

						for(int i=0; i<4; i++) 
							_pts_corner[i] = toCinder(getCorner(tid, i));
					}
				}
			}
		}
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

		_tex_bg = gl::Texture(frame_clr);
	}
}
