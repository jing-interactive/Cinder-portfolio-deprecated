#include "Player.h"
#include "cinder/app/App.h"
#include "cinder/Perlin.h"
#include "cinder/Triangulate.h"

using namespace ci::app;

namespace
{
	Perlin perlin;
}

Player::Player()
{
	lastUpdateTime = 0;
}

void Player::setup(const osc::Message* msg)
{
	lastUpdateTime = getElapsedSeconds();

	const int IDX_NUM_PTS = 7;
	int n_pts = msg->getArgAsInt32(IDX_NUM_PTS);
	id = msg->getArgAsInt32(0);

	shape.clear();

	for (int i=0;i<n_pts;i++)
	{
		float x = msg->getArgAsFloat(IDX_NUM_PTS+i*2+1);
		float y = msg->getArgAsFloat(IDX_NUM_PTS+i*2+2);
		x *= getWindowWidth();
		y *= getWindowHeight();
		Vec2f p(x,y);

		if (i == 0)
			shape.moveTo(x, y);
		else
			shape.lineTo(x, y);
	}
	shape.close();

	mesh = Triangulator(shape).calcMesh(Triangulator::WINDING_NONZERO);
}

void Player::draw()
{
	if (getElapsedSeconds() - lastUpdateTime > 2)
		return;

	glLineWidth( 1.0f );
	gl::color( Color( 0.8f, 0.4f, 0.0f ) );
	gl::draw(mesh);

	gl::enableWireframe();
	gl::color( Color::white() );
	gl::draw( mesh );
	gl::disableWireframe();
}
