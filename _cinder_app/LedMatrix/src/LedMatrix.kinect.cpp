#include "LedMatrixApp.h"
#include "OscMessage.h"

using namespace ci;

void LedMatrixApp::onKinect( const osc::Message* msg )
{
	if (msg->getAddress() == "/start")
	{
		int dev_id = msg->getArgAsInt32(0);
		if (one_msg[dev_id].empty())
			return;
		lock_guard<mutex> lock(mtx_kinect_queues[dev_id]);
		kinect_queues[dev_id].push_back(one_msg[dev_id]);
		one_msg[dev_id].clear();
	}
	else
	if (msg->getAddress() == "/contour")
	{
		Vec3f pos(msg->getArgAsFloat(2), msg->getArgAsFloat(3),
			msg->getArgAsFloat(5));
 		int dev_id = msg->getArgAsInt32(7); 
		one_msg[dev_id].push_back(pos);
	}
}
