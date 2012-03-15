#include "LedMatrixApp.h"
#include "OscMessage.h"

using namespace ci;

namespace
{
	const int N_SKELETON = 6;
}

void LedMatrixApp::onKinect( const osc::Message* msg )
{
	if (msg->getAddress() == "/start")
	{
		int dev_id = msg->getArgAsInt32(0);
		if (single_session[dev_id].empty())
			return;
		lock_guard<mutex> lock(mtx_kinect_queues[dev_id]);
		kinect_queues[dev_id].push_back(single_session[dev_id]);
		single_session[dev_id].clear();
	}
	else
	if (msg->getAddress() == "/contour")
	{
		Vec3f pos(msg->getArgAsFloat(2), msg->getArgAsFloat(3),
			msg->getArgAsFloat(6));
		int blob_id = msg->getArgAsInt32(0);
 		int dev_id = blob_id/N_SKELETON; 
		single_session[dev_id].push_back(pos);
	}
}


bool LedMatrixApp::getNewCenter( vector<Vec3i>& center, int dev )
{
	bool updated = !centers[dev].empty();
	if (updated)
	{
		center = centers[dev];
	}
	return updated;
}
