#pragma once

#include "cinder/Vector.h"
#include "cinder/osc/OscListener.h"
#include "cinder/Shape2d.h"
#include "cinder/TriMesh.h" 

using namespace ci;
using namespace std;

struct Player
{
	Player();
	void setup(const osc::Message* msg);
	void draw();
private:
	int id;
	int lastUpdateTime;
	vector<Path2d> shapes;
};
