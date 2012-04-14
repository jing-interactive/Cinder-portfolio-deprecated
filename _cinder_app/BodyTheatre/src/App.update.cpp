#include "App.h"
#include "cinder/Perlin.h"
#include "KinectRoutine.h"
#include "Player.h"
#include "boost/foreach.hpp"
#include "CiTool.h"

void TrackedNode::moveTo( const Vec2f& target )
{
	Vec2f myTarget(target + _offset);
	_ref->_pos.value() = lerp(_ref->_pos.value(), myTarget, 0.2f);
}

namespace
{
	const float NEAR_DIST = 40;
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
			const Hand& right = _routine->_hands[RIGHT];
			ci::Vec3f posW = screenToWorld(right.pos);
			//select nodes

			if (right.state == Hand::CLICK)
			{//select
				activeNodes.clear();
				vector<PathNode>& nodes = the_player.nodes;
				BOOST_FOREACH(PathNode& n, nodes)
				{
					if (n.isWorldPointInside(posW))
					{
						TrackedNode tn;
						tn._ref = &n;
						tn._offset = n._pos.value() - right.pos;
						activeNodes.push_back(tn);
						break;
					}
				}
				if (activeNodes.empty())
				{
					BOOST_FOREACH(PathNode& n, nodes)
					{
						if (n.distance(right.pos) < NEAR_DIST)
						{
							TrackedNode tn;
							tn._ref = &n;
							tn._offset = n._pos.value() - right.pos;
							activeNodes.push_back(tn);
						}
					}
				}
			}
			else if (right.state == Hand::DRAG)
			{//move selected nodes
				BOOST_FOREACH(TrackedNode& n, activeNodes)
				{
					n.moveTo(right.pos);
				}
			}
			else
			{//de-select
				activeNodes.clear();
			}

			//rotation
			if (right.state != Hand:NORMAL)
			{

			}
		}
	}
	_routine->update();
}