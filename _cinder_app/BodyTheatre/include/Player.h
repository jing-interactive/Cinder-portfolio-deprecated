#pragma once

#include "cinder/Vector.h"
#include "cinder/osc/OscListener.h"
#include "cinder/Shape2d.h"
#include "cinder/TriMesh.h" 
#include "opencv2/opencv.hpp"
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
	Path2d _path;
	Anim<ci::Vec2f> _pos;
	ci::Vec2f _size;
	Anim<float> _rot;
	int _z;
	void draw();
};

struct Player
{
	enum{//state
		T_ENTER,
		T_SPLITTING,
		T_SPLITTED,
		T_CAPTURE,
		T_SHARE,
		T_INVAILD,
	};
	Player();
	void setup(const osc::Message* msg);
	void draw();
	void split(int n_splits);
private:
	int state;
	bool visible;
	int id;
	float lastUpdateTime;
	float birthTime;//for spit usage

	float whole_alpha;//[0,1)
	ci::Vec2f center;
	Path2d whole;
	vector<PathNode> nodes;
	std::vector<cv::Point> points;
};
