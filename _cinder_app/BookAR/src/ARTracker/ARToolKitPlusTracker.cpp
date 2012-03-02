#include "ARToolKitPlusTracker.h"
// #include ""

ARToolKitPlusTracker::~ARToolKitPlusTracker()
{
    _tracker = 0;
}

ARToolKitPlusTracker::~ARToolKitPlusTracker()
{
    if (_tracker)
        delete _tracker;
}

void ARToolKitPlusTracker::setup(int width, int height)
{
	_tracker = shared_ptr<ARToolKitPlus::TrackerSingleMarker>(new ARToolKitPlus::TrackerSingleMarker(CAM_W, CAM_H, 8, 6, 6, 6, 0));
	{
		_tracker->setPixelFormat(ARToolKitPlus::PIXEL_FORMAT_LUM);
		// load a camera file.
		std::string path(getAppPath()+"../../resources/no_distortion.cal");
		if (!_tracker->init(path.c_str(), 1.0f, 1000.0f)) {
			console() << "ERROR: init() failed\n";
			return;
		}
		_tracker->getCamera()->printSettings();
		_tracker->setBorderWidth(0.125f);
		// set a threshold. we could also activate automatic thresholding
	#if 1
		_tracker->setThreshold(150);
	#else
		_tracker->activateAutoThreshold(true);
	#endif
		// let's use lookup-table undistortion for high-speed
		// note: LUT only works with images up to 1024x1024
		_tracker->setUndistortionMode(ARToolKitPlus::UNDIST_LUT);
		// switch to simple ID based markers
		// use the tool in tools/IdPatGen to generate markers
		_tracker->setMarkerMode(ARToolKitPlus::MARKER_ID_BCH);

		_tracker->setPoseEstimator(ARToolKitPlus::POSE_ESTIMATOR_RPP);
	}
}

void ARToolKitPlusTracker::update(unsigned char* data)
{
    ARToolKitPlus::ARMarkerInfo* m_infos;
    int numActiveTrackables;
    std::vector<int> markerId = _tracker->calc(data, &m_infos, &numActiveTrackables);

    int best_id = _tracker->selectBestMarkerByCf();
    {
        std::lock_guard<mutex> lock(_mtx_ar);
        _n_trackables = numActiveTrackables;

        for (int j = 0; j < numActiveTrackables; j++) 
        {
            if (m_infos[j].id == best_id)
            {		
                for(int i=0; i<4; i++) 
                    _pts_corner[i] = ci::Vec2f(m_infos[j].vertex[i][0], m_infos[j].vertex[i][1]);
                _mat_proj = Matrix44d(_tracker->getProjectionMatrix());
                _mat_modelview = Matrix44d(_tracker->getModelViewMatrix());
                break;
            }
        }
    }
}

unsigned int ARToolKitPlusTracker::getNumOfTrackables()
{
}

Matrix44d ARToolKitPlusTracker::getModelViewMatrix(unsigned int tIdx)
{
}

Matrix44d ARToolKitPlusTracker::getProjectionMatrix();
{
}

ci::Vec2f[4] ARToolKitPlusTracker::getCorners(unsigned int tIdx)
{
}

