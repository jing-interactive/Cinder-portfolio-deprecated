#include "App.h"
#include "cinder/Perlin.h"
#include "KinectRoutine.h"
#include "Player.h"
#include "boost/foreach.hpp"
#include "CiTool.h"
#include "Config.h"

void TrackedNode::moveTo( const Vec2f& target )
{
	Vec2f myTarget(target + _offset);
	_ref->_pos.value() = lerp(_ref->_pos.value(), myTarget, 0.2f);
}

namespace
{
	const float NEAR_DIST = 50;
}

void BodyTheatreApp::update()
{
	if (_activeIdx != INVALID_IDX)
	{
		struct Player& the_player = players[_activeIdx];
		the_player.update();
		if (!the_player.isAlive())
			_activeIdx = INVALID_IDX;

		if (the_player.state == Player::T_SPLITTED)
		{
			for (int i=0;i<2;i++)
			{
				const Hand& hand = _routine->_hands[i];
				vector<TrackedNode>& tracked = activeNodes[i];
				ci::Vec3f posW = screenToWorld(hand.pos);
				//select nodes

				if (hand.state == Hand::CLICK)
				{//select
					doSelectNodes(tracked, posW, hand);
				}
				else if (hand.state == Hand::DRAG)
				{//move selected nodes
					if (tracked.empty())
					{
						doSelectNodes(tracked, posW, hand);
					}
					BOOST_FOREACH(TrackedNode& n, tracked)
					{
						n.moveTo(hand.pos);
					}
				}
				else
				{//de-select
					tracked.clear();
				}
			}

		}
	}
	_routine->update();
}


void BodyTheatreApp::doSelectNodes( vector<TrackedNode>& tracked, ci::Vec3f posW, const Hand& hand ) 
{
	tracked.clear();
	vector<PathNode>& nodes = players[_activeIdx].nodes;
	//if direct hits
	BOOST_FOREACH(PathNode& n, nodes)
	{
		if (n.isWorldPointInside(posW))
		{
			TrackedNode tn;
			tn._ref = &n;
			tn._offset = n._pos.value() - hand.pos;
			tracked.push_back(tn);
			break;
		}
	}
	//else check distance
	if (tracked.empty())
	{
		BOOST_FOREACH(PathNode& n, nodes)
		{
			if (n.distance(hand.pos) < NEAR_DIST)
			{
				TrackedNode tn;
				tn._ref = &n;
				tn._offset = n._pos.value() - hand.pos;
				tracked.push_back(tn);
			}
		}
	}
}