#pragma once

#include "cinder/app/AppBasic.h"
#include "cinder/osc/OscMessage.h"

using namespace ci;
using namespace ci::app;
using namespace std;

enum T_HAND{
	LEFT = 0,
	RIGHT,
	N_HANDS,
};

struct KinectRoutine {
	KinectRoutine();

	void mouseWheel( MouseEvent event );

	void mouseMove( MouseEvent event );
	void mouseDown( MouseEvent event );
	void mouseUp( MouseEvent event );
	void mouseDrag( MouseEvent event );

	void update();
	void draw();

	void onOscMessage(const osc::Message* msg);
  
	shared_ptr<struct Hand> _hands[N_HANDS];

	float _scale;
	float _rotate;
};