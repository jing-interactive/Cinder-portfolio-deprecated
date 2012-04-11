#include "Player.h"
#include "cinder/Perlin.h"

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
	mPoints.resize(n_pts);

	shape.clear();

	for (int i=0;i<n_pts;i++)
	{
		float x = msg->getArgAsFloat(IDX_NUM_PTS+i*2+1);
		float y = msg->getArgAsFloat(IDX_NUM_PTS+i*2+2);
		x *= getWindowWidth();
		y *= getWindowHeight();
		Vec2f p(x,y);
		mPoints[i] = p;

		if (i == 0)
			shape.moveTo(x, y);
		else
			shape.lineTo(x, y);
	}
	shape.close();
	mTriangles = Triangle::triangulate( mPoints);
}

void Player::draw()
{
	if (getElapsedSeconds() - lastUpdateTime > 2)
		return;

#if 1
	glLineWidth( 4.0f );
	gl::draw(shape);

#else
	// Draw triangles
	glLineWidth( 2.0f );
	for ( vector<Triangle>::const_iterator triIt = mTriangles.begin(); triIt != mTriangles.end(); ++triIt ) 
	{
		glBegin(GL_TRIANGLES );

		float sc = 0.01f;
		float r = perlin.fBm(triIt->mIndex[0]*sc, getElapsedFrames() * sc, id*0.01)+0.5f;

		gl::color(r, r, r*2);
		gl::vertex( triIt->a() );

		r = perlin.fBm(triIt->mIndex[1]*sc, getElapsedFrames() * sc, id*0.01)+0.5f;
		gl::color(r, r, r*2);
		gl::vertex( triIt->b() );

		r = perlin.fBm(triIt->mIndex[2]*sc, getElapsedFrames() * sc, id*0.01)+0.5f;
		gl::vertex( triIt->c() );
		glEnd();
		gl::drawSolidCircle( triIt->getCentroid(), 1.0f, 12 );
	}
	glEnd();
#endif
}
