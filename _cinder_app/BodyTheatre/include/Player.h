#pragma once

#include "cinder/Vector.h"
#include "cinder/osc/OscListener.h"
#include "cinder/Shape2d.h"
#include "cinder/TriMesh.h" 
#include "opencv2/opencv.hpp"
#include "cinder/Timeline.h"
#include "cinder/gl/Vbo.h"
#include "cinder/Matrix.h"
#include "PathNode.h"

using namespace ci;
using namespace std;

struct Player
{
	enum{//state
		T_ENTER, 
		T_SPLITTING,
		T_SPLITTED,
		T_SHARE,
		T_INVAILD,
	};
	Player();
	void setup(const osc::Message* msg);
	void update();
	void draw();

	void postWeibo();

	void drawOutline();
	void split(int n_splits);
	bool isAlive() const
	{
		return alive;
	}
	void drawTiming( float elpased, std::string info );
	vector<PathNode> nodes;
	int state;

private:
	bool alive;
	int id;
	float lastUpdateTime;
	float birthTime;//for spit usage

	float whole_alpha;//[0,1)
	ci::Vec2f center;
	Path2d whole;
	float prevLife;

	std::vector<Surface8u> captures;
	
	std::vector<cv::Point> points;
};
