#pragma once

#include "cinder/Vector.h"
#include "cinder/osc/OscListener.h"
#include "cinder/delaunay/delaunay.h"

using namespace ci;
using namespace std;

struct Player
{
	int id;
	vector<ci::Vec2f>		mPoints;
	// Triangles created from points
	vector<Triangle>		mTriangles;

	void setup(const osc::Message* msg);

	void draw();
};
