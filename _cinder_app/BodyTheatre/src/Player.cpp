#include "Player.h"
#include "cinder/app/App.h"
#include "cinder/Perlin.h"
#include "cinder/Triangulate.h"
#include <opencv2/opencv.hpp>
#include <boost/foreach.hpp>
#include "cinder/Rand.h"
#include "CinderOpenCV.h"

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

	shapes.resize(1);
	shapes.back().clear();

	std::vector<cv::Point> points;

	for (int i=0;i<n_pts;i++)
	{
		float x = msg->getArgAsFloat(IDX_NUM_PTS+i*2+1);
		float y = msg->getArgAsFloat(IDX_NUM_PTS+i*2+2);
		x *= getWindowWidth();
		y *= getWindowHeight();
		ci::Vec2f p(x,y);

		if (i == 0)
			shapes.back().moveTo(p);
		else
			shapes.back().lineTo(p);

		points.push_back(cv::Point(x,y));
	}
	shapes.back().close();

	Rectf box = shapes.back().calcBoundingBox();

	cv::Mat frame = cv::Mat::zeros(cv::Size(getWindowWidth(),getWindowHeight()), CV_8U);
	const int count = points.size();
 	const cv::Point* pts = &points[0];
	cv::fillPoly(frame, &pts, &count, 1, cv::Scalar(255));
	float radius = box.getHeight()*2;
	for (int i=0;i<5;i++)
	{
		Vec2i ct(Rand::randFloat(box.x1, box.x2), Rand::randFloat(box.y1, box.y2));
		float theta(Rand::randFloat(3.14f));
		Vec2i p1(radius*cos(theta), radius*sin(theta));
		Vec2i p2(-radius*cos(theta), -radius*sin(theta));

		cv::line(frame,  toOcv(ct + p1), toOcv(ct + p2),
			CV_RGB(0,0,0));
	}

	cvNamedWindow("xx");
	cvShowImage("xx", &(IplImage)frame);
	cvWaitKey(1);

	/*	mesh = Triangulator(shape).calcMesh(Triangulator::WINDING_NONZERO);*/
}

void Player::draw()
{
	if (getElapsedSeconds() - lastUpdateTime > 2)
		return;

	glLineWidth( 1.0f );
	gl::color( Color( 0.8f, 0.4f, 0.0f ) );

	//gl::draw(shapes.back());

// 	BOOST_FOREACH(Path2d& s, shapes)
// 		gl::draw(s); 
}
