#pragma once
//used at StateFollowingState and StateBreathe.cpp
#include <vector>

struct Line
{
	enum LineState
	{
		T_COME,
		T_GO,
	};
	std::vector<float> snake_z;//vector of z, just like 1-D snake eater
	float speed;
	float decay;
	bool visible;
	LineState state;

	Line();
	void reset();
	void update(int dev);
	void setTarget(const ci::Vec3i& pos);
private: 
	ci::Vec3i target;
	int x,y;
	int life;
};