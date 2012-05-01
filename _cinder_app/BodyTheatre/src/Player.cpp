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
#include "cinder/BSpline.h"
#include "cinder/gl/Texture.h"
#include "CiTool.h"
#include "Config.h"
#include "cinder/Thread.h"

using namespace ci::app;

namespace
{
	Perlin perlin;

	//UI related
	Font fnt_big;
	Font fnt_small;
	Surface8u icons[2];
	Surface8u profile;
	vector<string> weibos;

	const int N_WEIBOS = 3;

	shared_ptr<thread> thread_weibo;
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
		string icon_files[] = {"PHOTO.png", "sina-weibo.png", };
		for (int i=0;i<2;i++)
		{
			icons[i] = loadImage(loadAsset(icon_files[i]));
		}
		profile = loadImage(loadAsset("profile.png"));
		weibos.push_back(WEIBO_0);
		weibos.push_back(WEIBO_1);
		weibos.push_back(WEIBO_2);
	}
	lastUpdateTime = getElapsedSeconds();

	if (!alive)
	{
		alive = true;
		birthTime = getElapsedSeconds();
		state = T_ENTER;
		whole_alpha = 0.6f;
		captures.clear();
//		nodes.clear();
	}

	if (state == T_ENTER)
	{
		//1. build from osc
		const int IDX_NUM_PTS = 7;
		int n_pts = msg->getArgAsInt32(IDX_NUM_PTS);
		id = msg->getArgAsInt32(0);
		points.clear();
		vector<ci::Vec2f>		mPoints;

		//whole.clear();
		for (int i=0;i<n_pts;i++)
		{
			float x = msg->getArgAsFloat(IDX_NUM_PTS+i*2+1);
			float y = msg->getArgAsFloat(IDX_NUM_PTS+i*2+2);
			x *= (getWindowWidth()*BLOB_SCALE);
			y *= (getWindowHeight()*BLOB_SCALE); 
			mPoints.push_back(ci::Vec2f(x,y));
			points.push_back(cv::Point(x,y));
		} 

		whole = Path2d(BSpline2f(mPoints, 3, true, true));

		center = whole.calcBoundingBox().getCenter();
	}
}

void Player::draw()
{
	if (!alive)
		return;

	float life = getElapsedSeconds() - birthTime;

	if (state != T_SHARE && getElapsedSeconds() - lastUpdateTime > TIME_TURN_INVISIBLE)
	{//i am dying in the sun
		if (life > TIME_TOTAL*0.7f)//enough time
			state = T_SHARE;
		else
		{
			state = T_ENTER;
			alive = false;
			nodes.clear();
		}
	}

	switch (state)
	{
	case T_ENTER:
		if (life > TIME_BEFORE_SPLIT)
		{//how old am I
			split(N_SPLITS);
			BOOST_FOREACH(PathNode& p, nodes)
			{
				ci::Vec2f diff = p._pos.value() - center;
				ci::Vec2f target = p._pos.value() + diff*Rand::randFloat(-0.1f, 1.3f);
				const int Spacing = 10;
				target.x = constrain<float>(target.x, Spacing, getWindowWidth()-Spacing);
				target.y = constrain<float>(target.y, Spacing, getWindowHeight()-Spacing);
				p.moveTo(target, Rand::randFloat(TIME_SPLITTING_PAUSE*0.7f,TIME_SPLITTING_PAUSE));
			}
			state = T_SPLITTING;
		}
		else
		{
			gl::color(ColorA(0.3,0.7,0.7,whole_alpha));
			gl::drawSolid(whole);
		}
		break;
	case T_SPLITTING:
	case T_SPLITTED:
		if (life - TIME_SPLITTING_PAUSE > TIME_BEFORE_SPLIT*0.7f)
			state = T_SPLITTED;
		if (life - TIME_SPLITTING_PAUSE > TIME_TOTAL)
		{
			state = T_SHARE;
			thread_weibo = shared_ptr<thread>( new thread( &Player::postWeibo, this ) );
		}
		else
		{
			BOOST_FOREACH(PathNode& s, nodes)
			{
				s.draw();
			}
		}
		break;
	case T_SHARE:
		gl::color(ColorA::white());
		gl::draw(profile, Vec2f(40,40));
		gl::drawString(toUtf8(L"作品正发送至新浪微博，关注我 @vinjn 即可查看:)"), Vec2f(40,400), ColorA::black(), fnt_small);
		break;
	default:
		break;
	}

	if (state != T_SHARE)
	{
		float sec_per_frame = SEC_PER_FRAME_1;
		if (state == T_SPLITTED)
			sec_per_frame = SEC_PER_FRAME_2;

		drawTiming(life, "");
		if ((int)(life/sec_per_frame+0.1f) != (int)(prevLife/sec_per_frame+0.1f))
		{
			Surface8u cap = copyWindowSurface();
			captures.push_back(cap);
		}
	}

	prevLife = life;
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
	for (int i=0;i<n_splits;i++)
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
	vFindBlobs(frame, blobs, MIN_BLOB_SIZE);

	//TODO: optimize
	BOOST_FOREACH(vBlob b, blobs)
	{
		Path2d path;

		int blob_pts = b.pts.size();

		if (blob_pts < 4)
			continue;

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

#define K  20

float mapped_x(float x)
{
	return lmap<float>(x, 0, TIME_TOTAL, K, getWindowWidth()-K);
}

void Player::drawTiming(float elapsed, std::string info )
{
	gl::color(0.3f, 0, 0, 0.2f);
	gl::drawSolidRoundedRect(Rectf(K,K,mapped_x(TIME_TOTAL),K*2), K/2); 

	if (elapsed > TIME_BEFORE_SPLIT && elapsed < TIME_BEFORE_SPLIT+TIME_SPLITTING_PAUSE)
		elapsed = TIME_BEFORE_SPLIT;
	else if (elapsed > TIME_BEFORE_SPLIT+TIME_SPLITTING_PAUSE)
		elapsed -= TIME_SPLITTING_PAUSE;

	gl::color(0.5f, 0.5f, 0.5f, 0.7f);
	gl::drawSolidRoundedRect(Rectf(K,K,mapped_x(elapsed),K*2), K/2);

	float icons_x[]={mapped_x(TIME_BEFORE_SPLIT)+K, mapped_x(TIME_TOTAL)-K};
	gl::color(ColorA::white());
	for (int i=0;i<2;i++)
	{
		gl::draw(icons[i], Rectf(icons_x[i]-32, 0, icons_x[i]+32,64));
	}
}

void Player::postWeibo()
{
	int frm = getElapsedFrames();
	fs::path folder = getTemporaryDirectory()/toString(frm); 
	writeImages(captures, folder);
	char param[512];
	sprintf(param, "-delay 500 -loop 0 -type Palette %s/*.jpg anim.gif", folder.string().c_str());
	//	execute("gen_gif.bat", folder.string());
	execute("d:/EverBox/APP/ImageMagick-6.7.6-5/conv.exe", param);
	string weibo = weibos[rand()%N_WEIBOS];
	weibo += string("##");
	weibo += toString(frm);
	sprintf(param, "-u \"%s:%s\" -F \"pic=@%s\" -F \"status=%s\" \"http://api.t.sina.com.cn/statuses/upload.xml?source=3709681010\"", 
		weibo_usr.c_str(), weibo_pwd.c_str(), "anim.gif", weibo.c_str());
	//	execute("d:/EverBox/APP/curl-7.25.0-sspi-zlib-static-bin-w32/curl.exe", param);
	string cmd = string("d:/EverBox/APP/curl-7.25.0-sspi-zlib-static-bin-w32/curl.exe ")+param;
	::system(cmd.c_str());

	alive = false;
}
