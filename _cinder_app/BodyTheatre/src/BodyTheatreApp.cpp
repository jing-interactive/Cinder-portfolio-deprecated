#include "cinder/app/AppBasic.h"
#include "cinder/gl/gl.h"
#include "cinder/osc/OscListener.h"
#include "cinder/Shape2d.h"
#include "cinder/Thread.h"
#include "cinder/delaunay/delaunay.h"
#include <boost/foreach.hpp>
#include "cinder/Perlin.h"

using namespace ci;
using namespace ci::app;
using namespace std;

Perlin perlin;

struct Player
{
	int id;
	vector<ci::Vec2f>		mPoints;
	// Triangles created from points
	vector<Triangle>		mTriangles;

	void setup(const osc::Message* msg)
	{
		const int IDX_NUM_PTS = 7;
		int n_pts = msg->getArgAsInt32(IDX_NUM_PTS);
		id = msg->getArgAsInt32(0);
		mPoints.resize(n_pts);

		for (int i=0;i<n_pts;i++)
		{
			float x = msg->getArgAsFloat(IDX_NUM_PTS+i*2+1);
			float y = msg->getArgAsFloat(IDX_NUM_PTS+i*2+2);
			x *= getWindowWidth();
			y *= getWindowHeight();
			Vec2f p(x,y);
			mPoints[i] = p;
		}
		mTriangles = Triangle::triangulate( mPoints);
	}

	void draw()
	{
		// Draw triangles
		glLineWidth( 2.0f );
		for ( vector<Triangle>::const_iterator triIt = mTriangles.begin(); triIt != mTriangles.end(); ++triIt ) 
		{
			glBegin(GL_TRIANGLES );

			float sc = 0.01f;
			float r = perlin.fBm(triIt->mIndex[0]*0.1f, getElapsedFrames() * sc, id*0.01)+0.5f;

			gl::color(r, r, r*2);
			gl::vertex( triIt->a() );

			r = perlin.fBm(triIt->mIndex[1]*0.1f, getElapsedFrames() * sc, id*0.01)+0.5f;
			gl::color(r, r, r*2);
			gl::vertex( triIt->b() );

			r = perlin.fBm(triIt->mIndex[2]*0.1f, getElapsedFrames() * sc, id*0.01)+0.5f;
			gl::vertex( triIt->c() );
			glEnd();
			gl::drawSolidCircle( triIt->getCentroid(), 1.0f, 12 );
		}
		glEnd();
	}
};

typedef map<int, Player> PlayerMap;

struct BodyTheatreApp : public AppBasic {
	void setup();
	void mouseDown( MouseEvent event );	
	void keyDown( KeyEvent event );
	void update();
	void draw();
	void onOscMessage( const osc::Message* msg );

	shared_ptr<osc::Listener> _listener;

	PlayerMap players;

	mutex _mtx_player;
};

void BodyTheatreApp::setup()
{
	_listener = shared_ptr<osc::Listener>(new osc::Listener);
	_listener->setup(7777);
	_listener->registerMessageReceived(this, &BodyTheatreApp::onOscMessage);

	perlin.setSeed(clock());
}

void BodyTheatreApp::mouseDown( MouseEvent event )
{
}

void BodyTheatreApp::update()
{
}

void BodyTheatreApp::draw()
{
	gl::enableAlphaBlending();
	gl::clear( Color( 0.3f, 0.3f, 0.3f ) );
	{
		lock_guard<mutex> lock(_mtx_player);
		BOOST_FOREACH(PlayerMap::value_type pair, players)
		{
			pair.second.draw();
		}
	}	
}

void BodyTheatreApp::onOscMessage( const osc::Message* msg )
{
	console() << msg->getAddress() <<endl;

	if (msg->getAddress() == "/start")
	{
	}
	else if (msg->getAddress() == "/contour")
	{
		int id = msg->getArgAsInt32(0);
		if (players.find(id) == players.end())
		{
			Player ply;

			ply.setup(msg);

			lock_guard<mutex> lock(_mtx_player);
			players[id] = ply;
		}
	}
	else if (msg->getAddress() == "/kinect")
	{

	}
}

void BodyTheatreApp::keyDown( KeyEvent event )
{
	switch (event.getCode())
	{
	case KeyEvent::KEY_ESCAPE:
		{
			quit();
		}break;
	case  KeyEvent::KEY_f:
		{
			setFullScreen(!isFullScreen());
		}break;
	}
}


CINDER_APP_BASIC( BodyTheatreApp, RendererGl )
