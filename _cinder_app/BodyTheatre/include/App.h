#pragma once

#include "cinder/app/AppBasic.h"
#include "cinder/gl/gl.h"
#include "cinder/osc/OscListener.h"
#include "cinder/Shape2d.h"
#include "cinder/Thread.h"
#include <opencv2/opencv.hpp>

using namespace ci;
using namespace ci::app;
using namespace std; 

#define N_PLAYERS 8
#define INVALID_IDX -1

struct TrackedNode
{
	struct PathNode* _ref;
	Vec2f _offset;
	void moveTo(const Vec2f& target);
};

struct BodyTheatreApp : public AppBasic {
	void setup();
	void prepareSettings( Settings *settings );

	void mouseWheel( MouseEvent event );

	void mouseDown( MouseEvent event );	
	void mouseDrag( MouseEvent event );
	void mouseUp( MouseEvent event );	
	void mouseMove( MouseEvent event );

	void keyDown( KeyEvent event );

	void update();

	void doSelectNodes( vector<TrackedNode>& tracked, ci::Vec3f posW, const struct Hand& hand );

	void draw();
	void onOscMessage( const osc::Message* msg );

	void shutdown();

	shared_ptr<osc::Listener> _listener;

	struct Player* players;
	vector<TrackedNode> activeNodes[2];

	int _activeIdx;

	mutex _mtx_player;

	shared_ptr<struct KinectRoutine> _routine;
};