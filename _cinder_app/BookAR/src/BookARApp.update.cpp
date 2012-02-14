#include <ARToolKitPlus/TrackerSingleMarker.h>
#include "BookARApp.h"
#include "cinder/params/Params.h"

#include "../../_common/SDAR/ModelDLL.h"

ci::Vec2f toCinder(const IPoint& rhs)
{
	return ci::Vec2f(rhs.x, rhs.y);;
}

void BookARApp::update()
{	
	if( _capture && _capture.checkNewFrame() ) 
	{
		Surface8u& frame = _capture.getSurface();

		if (_ar_engine == ENGINE_SNDA)
		{
			int numActiveTrackables = SDARTrack(frame.getData(), _capture.getWidth()*3);
			{
				std::lock_guard<mutex> lock(_mtx_ar);
				_n_trackables = numActiveTrackables;
				if (_n_trackables > 0)
				{
					console() << _n_trackables << std::endl;

					_mat_proj = Matrix44d(getProjectionMatrix(10,1000));

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
			ARToolKitPlus::ARMarkerInfo* arr_marker_infos;
			int numActiveTrackables;
			std::vector<int> markerId = _artk_tracker.calc(grayPixels, &arr_marker_infos, &numActiveTrackables);

			if (!markerId.empty())
			{
				int best_id = _artk_tracker.selectBestMarkerByCf();
				for (int j = 0; j < numActiveTrackables; j++) 
				{
					if (arr_marker_infos[j].id == best_id)
					{					
						std::lock_guard<mutex> lock(_mtx_ar);
						_n_trackables = numActiveTrackables;
				//		memcpy(mask_points, arr_marker_infos[j].vertex, sizeof(arr_marker_infos[j].vertex));
						_mat_proj = Matrix44d(_artk_tracker->getProjectionMatrix());
						_mat_modelview = Matrix44d(_artk_tracker->getModelViewMatrix());
						break;
					}
				}
			}
		}

		_tex_bg = gl::Texture(frame);
	}
}
