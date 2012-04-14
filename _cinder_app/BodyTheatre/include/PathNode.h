#pragma once

#include "cinder/Vector.h"
#include "cinder/Shape2d.h"
#include "cinder/TriMesh.h" 
#include "cinder/gl/Vbo.h"
#include "cinder/Matrix.h"
#include "cinder/Timeline.h"

using namespace ci;
using namespace std;

struct PathNode
{
	PathNode();

	PathNode(const Path2d& pathW);
	//from a worldPath, and convert to local path
	void setup(const Path2d& pathW);
	void moveTo(const ci::Vec2f& target, float duration);
	void update();
	bool isWorldPointInside(const ci::Vec3f& posW);
	float distance(const ci::Vec2f& posScr);
	Matrix44f transform;

	Path2d _path;
	gl::VboMesh _mesh;
	Anim<ci::Vec2f> _pos;
	ci::Vec2f _size;
	Anim<float> _rot;
	int _z;
	void draw();
	void drawOutline();
	ColorA _clr;
};