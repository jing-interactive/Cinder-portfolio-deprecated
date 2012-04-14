#include "cinder/app/App.h"
#include "cinder/Perlin.h"
#include "cinder/Triangulate.h"
#include <boost/foreach.hpp>
#include "cinder/Rand.h"
#include "CinderOpenCV.h"
#include "OpenCV.h"
#include "BlobTracker.h"
#include "PathNode.h"

using namespace ci::app;

namespace
{
	Perlin perlin;
	const int TIME_BEFORE_SPLIT = 2;
	const int TIME_TURN_INVISIBLE = 2;
	const int N_SPLITS = 25;
}

PathNode::PathNode()
{
	_rot = 0;
	_z = 0;
}

PathNode::PathNode( const Path2d& pathW )
{
	_rot = 0;
	_z = 0;
	setup(pathW);
}

void PathNode::draw()
{
	gl::color(_clr);
	gl::pushModelView();
	gl::multModelView(transform);
	gl::draw(_mesh);
	gl::popModelView();
}

void PathNode::setup( const Path2d& pathW )
{
	Rectf box = pathW.calcBoundingBox();
	_size.set(box.getWidth(), box.getHeight());

	Vec2f centerW = box.getCenter();

	_path.clear();
	int n_pts = pathW.getNumPoints();
	for (int i=0;i<n_pts;i++)
	{
		const Vec2f& ptW = pathW.getPoint(i);
		Vec2f ptL = ptW - centerW;
		if (i==0)
			_path.moveTo(ptL);
		else
			_path.lineTo(ptL);
	}
	_path.close();

	//build mesh
	TriMesh2d tri = Triangulator(_path).calcMesh();
	_mesh = gl::VboMesh(tri); 
	_pos = centerW;

	_clr = ColorA(Rand::randFloat(), Rand::randFloat(), Rand::randFloat(), 0.7f);
}

void PathNode::moveTo( const Vec2f& target, float duration)
{
	timeline().apply(&_pos, target, duration, EaseOutSine());
	timeline().apply(&_rot, Rand::randFloat(-90, 90), duration, EaseInOutQuad());
}

void PathNode::update()
{
	transform.setToIdentity();
	transform.translate(Vec3f(_pos.value(), 0));
	transform.rotate( Vec3f(0.0f, 0.0f, toRadians(_rot.value())) );
}

bool PathNode::isWorldPointInside(const ci::Vec3f& posW)
{
	// apply inverse transformation to convert mouse from world space to object space
	Vec3f mouseObjectPos = transform.inverted().transformPoint( posW );

	// now you can use Rect.contains() like this:
	return _path.contains( Vec2f(mouseObjectPos.x, mouseObjectPos.y));
}

void PathNode::drawOutline()
{	
	gl::pushModelView();
	gl::multModelView(transform);
	gl::draw(_path);
	gl::popModelView();
}

float PathNode::distance( const ci::Vec2f& posScr )
{
	return _pos.value().distance(posScr);
}
