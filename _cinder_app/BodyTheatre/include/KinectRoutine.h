#pragma once

#include "cinder/app/AppBasic.h"
#include "cinder/osc/OscMessage.h"
#include "Hand.h"

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

	bool isPlayerIdValid(int id);

	void mouseWheel( MouseEvent event );

	void mouseMove( MouseEvent event );
	void mouseDown( MouseEvent event );
	void mouseUp( MouseEvent event );
	void mouseDrag( MouseEvent event );

	void update();
	void draw();

	//if filter_id is assigned, only msg containing (*filter_id) is valid
	void onOscMessage(const osc::Message* msg, int* filter_id = NULL);
  
	Hand _hands[N_HANDS];

	float _scale;
	float _rotate;
};