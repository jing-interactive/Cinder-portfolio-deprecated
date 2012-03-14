#include "LedMatrixApp.h"
#include "LedState.h"
#include "LedManager.h"
#include "Config.h"

namespace
{
	double time_last_signal_came[N_DEVICES]={0};
}

void LedMatrixApp::update()
{
	int win_w = getWindowWidth();
	int win_h = getWindowHeight();
	double elapsed = getElapsedSeconds();

	for (int dev_id=0;dev_id<2;dev_id++)
	{
		bool isIdle = LedState::isIdleState(current_states[dev_id]->_type);
		if (!isIdle && elapsed - time_last_signal_came[dev_id] > SEC_TURN_IDLE)
		{ 
//			LedManager::get(dev_id).fadeIn(2);
//				.finishFn(std::bind(&LedMatrixApp::changeToRandomIdleState, this, dev_id));
			changeToRandomIdleState(dev_id);
			isIdle = false;
		}
		centers[dev_id].clear();
		mtx_kinect_queues[dev_id].lock();
		if (kinect_queues[dev_id].empty())
		{//unlock ASAP
			mtx_kinect_queues[dev_id].unlock();
		}
		else
		{//some one is moving before Kinect
			if (isIdle)
			{
//				LedManager::get(dev_id).fadeIn(2);
//					.finishFn(std::bind(&LedMatrixApp::changeToRandomInteractiveState, this, dev_id));
				changeToRandomInteractiveState(dev_id);
			}

			time_last_signal_came[dev_id] = elapsed;//update time

			vector<Vec3f> raw_centers = kinect_queues[dev_id].front();

			while (!kinect_queues[dev_id].empty())
			{
				raw_centers = kinect_queues[dev_id].front();
				kinect_queues[dev_id].pop_front();
			}
			mtx_kinect_queues[dev_id].unlock();//unlock

			int n_raws = raw_centers.size();
			for (int i=0;i<n_raws;i++)
			{
				const Vec3f& ct = raw_centers[i];
				int x = static_cast<int>(ct.x*LedManager::W);
				int y = static_cast<int>(ct.y*LedManager::H);
				int z = (int)lmap<float>(ct.z, Z_NEAR, Z_FAR, LedManager::Z-1, 0);
				centers[dev_id].push_back(Vec3i(x,y,z));
			}
			//send to interactive state
			//for [centers] no mutex is needed
		}
		LedManager::get(dev_id).reset();
		current_states[dev_id]->update();
	}
}
