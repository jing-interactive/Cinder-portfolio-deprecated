#pragma once

#include "cinder/app/AppBasic.h"
#include "cinder/gl/gl.h"
#include "cinder/osc/OscListener.h"
#include "cinder/Shape2d.h"
#include "cinder/Thread.h"
#include "cinder/delaunay/delaunay.h"

using namespace ci;
using namespace ci::app;
using namespace std;

#define N_PLAYERS 8

struct BodyTheatreApp : public AppBasic {
	void setup();
	void mouseDown( MouseEvent event );	
	void keyDown( KeyEvent event );
	void update();
	void draw();
	void onOscMessage( const osc::Message* msg );

	void shutdown();

	shared_ptr<osc::Listener> _listener;

	struct Player* players;

	mutex _mtx_player;
};