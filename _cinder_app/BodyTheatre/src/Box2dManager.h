#ifndef __BOX2DMANAGER_H__
#define __BOX2DMANAGER_H__

#include "../../stdinc.h"
#include "../../Utility/singleton.h"

#define PTM_RATIO (32.0f*CC_CONTENT_SCALE_FACTOR())

const short CATEGORY_MONSTER = 0x0001;		 // 0000000000000001 in binary
const short CATEGORY_SCENERY = 0x0002;		 // 0000000000000010 in binary
const short CATEGORY_PLAYER_WATER = 0x0004;	 // 0000000000000100 in binary
const short CATEGORY_PLAYER_ICE = 0x0008;	 // 0000000000001000 in binary
const short CATEGORY_PLAYER_FIRE = 0x0010;	 // 0000000000010000 in binary
const short CATEGORY_PLAYER_ACID = 0x0020;	 // 0000000000100000 in binary
const short CATEGORY_PLAYER_LITTLEWATER = 1<<6;	 // 0000000000100000 in binary

const short GROUP_PLAYER = -1;
const short GROUP_MONSTER = -2;
const short GROUP_SCENERY = 1;

/*
// When converting from Cocos2D coord to Box2D coord:
float x_box2D = x_cocos2D / PTM_RATIO;
float y_box2D = y_cocos2D / PTM_RATIO;
// When converting from Box2D coord to Cocos2D coord:
float x_cocos2D = x_box2D * PTM_RATIO;
float y_cocos2D = y_box2D * PTM_RATIO;
*/

#define TO_BOX2D(num) (num/PTM_RATIO)
#define TO_CC2D(num) (num*PTM_RATIO)

inline b2Vec2 _b2Vec2(const cocos2d::CCPoint& p)
{
	return b2Vec2(TO_BOX2D(p.x), TO_BOX2D(p.y));
}

inline cocos2d::CCPoint _CCPoint(const b2Vec2& p)
{
	return ccp(TO_CC2D(p.x), TO_CC2D(p.y));
}

class Box2dManager : public Singleton<Box2dManager>
{
protected:
	b2World* world;
	b2DebugDraw* debugDraw;
	std::vector<b2Body*> to_delete;
public:
	Box2dManager();
	virtual ~Box2dManager();

	//for static scenery
	b2Body* addStaticBox(cocos2d::CCPoint pos, cocos2d::CCSize size, uint16 categoryBits = CATEGORY_SCENERY);
	b2Body* addStaticConvex(cocos2d::CCPoint pos, const ccVertex2F* vertices, int32 verticesCount, uint16 categoryBits = CATEGORY_SCENERY);

	//for dynamic enemy and bullet
	b2Body* addKinematicBox(cocos2d::CCPoint pos, cocos2d::CCSize size, cocos2d::CCPoint vel, uint16 categoryBits = CATEGORY_MONSTER);
	b2Body* addDynamicBox(cocos2d::CCPoint pos, cocos2d::CCSize size, cocos2d::CCPoint vel, uint16 categoryBits = CATEGORY_MONSTER);
	b2Body* addDynamicCircle(cocos2d::CCPoint pos, float size, cocos2d::CCPoint vel, uint16 categoryBits = CATEGORY_PLAYER_WATER);

	b2Body* addKinematicConvex(cocos2d::CCPoint pos, const ccVertex2F* vertices, int32 verticesCount,  cocos2d::CCPoint vel, uint16 categoryBits = CATEGORY_MONSTER);

	void destroyBody(b2Body* pBody);

	void draw();
	void update(cocos2d::ccTime dt);

	bool getRayCastPoint(const b2Vec2& start, const b2Vec2& end, b2Vec2& hitPoint, b2Fixture** m_fixture = NULL);
/*
enum
{
	e_shapeBit				= 0x0001, ///< draw shapes
	e_jointBit				= 0x0002, ///< draw joint connections
	e_aabbBit				= 0x0004, ///< draw axis aligned bounding boxes
	e_pairBit				= 0x0008, ///< draw broad-phase pairs
	e_centerOfMassBit		= 0x0010, ///< draw center of mass frame
};
*/
	void setDebugDrawFlags(uint32 flags, bool isSet = true);
};

#endif //__BOX2DMANAGER_H__