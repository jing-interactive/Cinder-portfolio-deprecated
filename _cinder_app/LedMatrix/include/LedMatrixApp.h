#pragma once

#include <cinder/app/AppBasic.h>
#include <cinder/gl/gl.h>
#include <cinder/Thread.h>
#include "../../../_common/State.h"
#include <deque>

using namespace ci;
using namespace ci::app;
using namespace std;

enum{
	N_DEVICES = 2
};

namespace cinder {
	class MayaCamUI;
	namespace osc {
		class Listener;
		class Message;
	}
}

class LedMatrixApp : public AppBasic{
public:
	void setup();
	void mouseDown( MouseEvent event );
	void mouseDrag( MouseEvent event );
	void update();
	void draw();

public://camera
	shared_ptr<MayaCamUI> maya_cam;

public://osc
	void onKinect(const osc::Message* msg);
	bool getNewCenter(Vec3i& center, int dev);//try get
	shared_ptr<osc::Listener> listener;
	deque<vector<Vec3f>> kinect_queues[N_DEVICES];
	vector<Vec3f> one_msg[N_DEVICES];
	mutex mtx_kinect_queues[N_DEVICES];
	vector<Vec3i> centers[N_DEVICES];

public://state
	void changeToState(struct LedState* new_state);
	void changeToRandomIdleState(int dev_id);
	void changeToRandomInteractiveState(int dev_id);
	shared_ptr<struct LedState> current_states[N_DEVICES];

private:
	void changeToStateAmong( int dev_id, enum StateType idle_states[3]);
	void setupStates();
};
