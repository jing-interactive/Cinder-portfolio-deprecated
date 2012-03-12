#pragma once
//used at StateFollowingState and StateBreathe.cpp
#include <vector>

struct Line
{
	enum Mode
	{
		T_BULLETS,//always the same direction
		T_BOUNCING,//will change direction
	};

	enum{
		T_COME,
		T_STAY,
		T_GO
	};
	int direction;
	Line();
	void setMaxSpeed(float speed);
	void setMode(Mode mode);
	void reset();
	void update(int dev);
	void setTarget(const ci::Vec3i& pos);
	std::vector<float> snake_z;//vector of z, just like 1-D snake eater

	int bouncing_times;
private:
	float decay;
	bool visible;
	Mode mode;
	float max_speed;
	float speed;
	ci::Vec3i target;
	int x,y;
};