#pragma once

#include "cinder/app/AppBasic.h"
#include "cinder/gl/gl.h"
#include "cinder/osc/OscListener.h"
#include "cinder/Shape2d.h"
#include "cinder/Thread.h"
#include "cinder/delaunay/delaunay.h"
#include "Player.h"

using namespace ci;
using namespace ci::app;
using namespace std;

typedef map<int, Player> PlayerMap;

struct BodyTheatreApp : public AppBasic {
	void setup();
	void mouseDown( MouseEvent event );	
	void keyDown( KeyEvent event );
	void update();
	void draw();
	void onOscMessage( const osc::Message* msg );

	void shutdown();

	shared_ptr<osc::Listener> _listener;

	PlayerMap players;

	mutex _mtx_player;
};