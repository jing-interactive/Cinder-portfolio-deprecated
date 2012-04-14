#include "cinder/app/App.h"
#include "cinder/Perlin.h"
#include "cinder/Triangulate.h"
#include <boost/foreach.hpp>
#include "cinder/Rand.h"
#include "CinderOpenCV.h"
#include "OpenCV.h"
#include "BlobTracker.h"
#include "Player.h"
#include "cinder/Font.h"
#include "cinder/Utilities.h"

using namespace ci::app;

namespace
{
	Perlin perlin;
	const int TIME_BEFORE_SPLIT = 2;
	const int TIME_BEFORE_WEIBO = 10;

	const int TIME_TURN_INVISIBLE = 2;
	const int N_SPLITS = 15;
	Font fnt_big;
	Font fnt_small;
}

Player::Player()
{
	alive = false;
	lastUpdateTime = 0;
	birthTime = 0;
	state = T_INVAILD;
}

void Player::setup(const osc::Message* msg)
{
	if (!fnt_small)
	{
		fnt_big = Font("STHupo", 64);
		fnt_small = Font("YouYuan", 32);
	}
	lastUpdateTime = getElapsedSeconds();

	if (!alive)
	{
		alive = true;
		birthTime = getElapsedSeconds();
		state = T_ENTER;
		whole_alpha = 0.6f;
//		nodes.clear();
	}

	//1. build from osc
	const int IDX_NUM_PTS = 7;
	int n_pts = msg->getArgAsInt32(IDX_NUM_PTS);
	id = msg->getArgAsInt32(0);
	points.clear();
	whole.clear();
	for (int i=0;i<n_pts;i++)
	{
		float x = msg->getArgAsFloat(IDX_NUM_PTS+i*2+1);
		float y = msg->getArgAsFloat(IDX_NUM_PTS+i*2+2);
		x *= getWindowWidth();
		y *= getWindowHeight();
		if (i == 0)
			whole.moveTo(x,y);
		else
			whole.lineTo(x,y);
		points.push_back(cv::Point(x,y));
	}
	whole.close();
	
	center = whole.calcBoundingBox().getCenter();
}

void Player::draw()
{
	if (!alive)
		return;

	if (getElapsedSeconds() - lastUpdateTime > TIME_TURN_INVISIBLE)
	{//i am dying in the sun
		alive = false;
	}

	int life = getElapsedSeconds() - birthTime;

	switch (state)
	{
	case T_ENTER:
		if (life > TIME_BEFORE_SPLIT)
		{//how old am I
			split(20);
			BOOST_FOREACH(PathNode& p, nodes)
			{
				ci::Vec2f diff = p._pos.value() - center;
				ci::Vec2f target = p._pos.value() + diff*Rand::randFloat(-0.1f, 1.3f);
				const int Spacing = 10;
				target.x = constrain<float>(target.x, Spacing, getWindowWidth()-Spacing);
				target.y = constrain<float>(target.y, Spacing, getWindowHeight()-Spacing);
				p.moveTo(target, Rand::randFloat(2,4));
			}
			state = T_SPLITTING;
		}
		else
		{
			drawTiming(TIME_BEFORE_SPLIT - life, toUtf8(L"先摆个姿势 "));
			gl::color(ColorA(1,1,1,whole_alpha));
			gl::drawSolid(whole);
		}
		break;
	case T_SPLITTING:
		state = T_SPLITTED;
		break;
	case T_SPLITTED:
		if (life > TIME_BEFORE_WEIBO)
		{
			state = T_CAPTURE;
		}
		else
		{
			drawTiming(TIME_BEFORE_WEIBO - life, toUtf8(L"搭建倒计时 "));
			BOOST_FOREACH(PathNode& s, nodes)
			{
				s.draw();
			}
		}
		break;
	case T_CAPTURE:
		break;
	case T_SHARE:
		break;
	default:
		break;
	}
}

void Player::split( int n_splits )
{
	nodes.clear();

	//1. opencv
	Rectf box = whole.calcBoundingBox();

	cv::Mat frame = cv::Mat::zeros(cv::Size(getWindowWidth(),getWindowHeight()), CV_8U);
	const int count = points.size();
	const cv::Point* pts = &points[0];
	cv::fillPoly(frame, &pts, &count, 1, cv::Scalar(255));
	float radius = box.getHeight()*2;
	for (int i=0;i<N_SPLITS;i++)
	{
		Vec2f shuffle = Rand::randVec2f()*(box.getWidth()+box.getHeight())/4;
		Vec2i ct(box.getCenter() + shuffle);
		float theta;
		if (Rand::randBool())
			theta = Rand::randPosNegFloat(-0.3f, 0.3f);
		else
			theta = M_PI/2 + Rand::randPosNegFloat(-0.3f, 0.3f);
		Vec2i p1(radius*cos(theta), radius*sin(theta));
		Vec2i p2(-radius*cos(theta), -radius*sin(theta));

		cv::line(frame,  toOcv(ct + p1), toOcv(ct + p2),
			CV_RGB(0,0,0), 2);
	}

	//2. cinder
	vector<vBlob> blobs;
	vFindBlobs(frame, blobs, 30);

	//TODO: optimize
	BOOST_FOREACH(vBlob b, blobs)
	{
		Path2d path;

		int blob_pts = b.pts.size();
		for (int i=0;i<blob_pts;i++)
		{
			ci::Vec2f p = fromOcv(b.pts[i]);
			if (i == 0)
				path.moveTo(p);
			else
				path.lineTo(p);
		}
		path.close();

		nodes.push_back(PathNode(path));
	}

	// 	cvNamedWindow("xx");
	// 	cvShowImage("xx", TO_IPL(frame));
	// 	cvWaitKey(1);

	/*	mesh = Triangulator(shape).calcMesh(Triangulator::WINDING_NONZERO);*/
}

void Player::update()
{
	BOOST_FOREACH(PathNode& p, nodes)
		p.update();
}

void Player::drawOutline()
{
	if (getElapsedSeconds() - lastUpdateTime > TIME_TURN_INVISIBLE)
	{//i am dying in the sun
		alive = false;
	}

	if (alive)
		gl::drawSolid(whole);
}

void Player::drawTiming( int secRemaining, std::string info )
{
	char buf[100];
	sprintf(buf, "%s%d", info.c_str(), secRemaining); 
	gl::drawString(buf, Vec2f(40,40), ColorA::black(), fnt_small);
}
