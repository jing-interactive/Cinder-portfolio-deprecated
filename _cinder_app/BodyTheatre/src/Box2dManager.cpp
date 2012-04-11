#include "Box2dManager.h"
#include "GLES-Render.h"
#include "../Actor/Enemy.h"
#include "../Weapon/WeaponManager.h"
#include "ParticleSystemBox2d.h"
#include "../Level/Level.h"
#include "../Actor/PlayerActor.h"

USING_NS_CC;

class GlobalContactListener : public b2ContactListener, public Singleton<GlobalContactListener>
{
public:
	void BeginContact(b2Contact* contact);
	void EndContact(b2Contact* contact);
	void PreSolve(b2Contact* contact, const b2Manifold* oldManifold);
	void PostSolve(b2Contact* contact, const b2ContactImpulse* impulse);
};

Box2dManager::~Box2dManager()
{
	CC_SAFE_DELETE(world);
	CC_SAFE_DELETE(debugDraw);
}

Box2dManager::Box2dManager()
{
	b2Vec2 gravity = b2Vec2(0.0f, -9.8f);
	bool doSleep = true;
	world = new b2World(gravity, doSleep);
	debugDraw = new GLESDebugDraw;

	CCSize sz = CCDirector::sharedDirector()->getWinSizeInPixels();

	// 	addStaticBox(ccp(sz.width/2, 0), CCSize(sz.width,2)); 
	// 	addStaticBox(ccp(sz.width/2, sz.height), CCSize(sz.width,2)); 
	// 	addStaticBox(ccp(0, sz.height/2), CCSize(2,sz.height));
	// 	addStaticBox(ccp(sz.width-0, sz.height/2), CCSize(2,sz.height)); //Right edge

	world->SetDebugDraw(debugDraw);
	world->SetContactListener(GlobalContactListener::GetInstance());
}

void Box2dManager::update( ccTime dt )
{
	//DestroyBody
	int n = to_delete.size();
	for (int i=0;i<n;i++)
		world->DestroyBody(to_delete[i]);
	to_delete.clear();

	const int32 velocityIterations = 6;
	const int32 positionIterations = 2;
	float32 timeStep = 1.0f / BOX2D_UPDATE_FPS;

	if (dt < 0.001f)//if dt is equal to 0(this will happen when resume the game), set timeStep 0, then box2d will do nothing
	{
		timeStep = 0;
	}
	world->Step(timeStep, velocityIterations, positionIterations);
}

void Box2dManager::draw()
{
	world->DrawDebugData();
}

b2Body* Box2dManager::addStaticBox(CCPoint pos, CCSize size, uint16 categoryBits)
{
	b2BodyDef bodyDef;
	bodyDef.type = b2_staticBody;
	bodyDef.position = _b2Vec2(pos);

	b2Body *body = world->CreateBody(&bodyDef);

	b2PolygonShape shape;
	shape.SetAsBox(TO_BOX2D(size.width), TO_BOX2D(size.height));

	b2FixtureDef fixtureDef;
	fixtureDef.shape = &shape;
	fixtureDef.density = 1.0f;
	fixtureDef.friction = 0.5f;
	fixtureDef.restitution = 0.1f;
	fixtureDef.filter.categoryBits = categoryBits;

	body->CreateFixture(&fixtureDef);

	return body;
}

b2Body* Box2dManager::addKinematicBox(cocos2d::CCPoint pos, cocos2d::CCSize size, cocos2d::CCPoint vel, uint16 categoryBits)
{
	b2BodyDef bodyDef;
	bodyDef.type = b2_kinematicBody;
	bodyDef.position = _b2Vec2(pos);

	b2Body* body = world->CreateBody(&bodyDef);

	b2PolygonShape shape;
	shape.SetAsBox(TO_BOX2D(size.width), TO_BOX2D(size.height));

	b2FixtureDef fixtureDef;
	fixtureDef.shape = &shape;
	fixtureDef.density = 1.0f;
	fixtureDef.friction = 0.3f;
	fixtureDef.restitution = 0.3f;
	fixtureDef.filter.categoryBits = categoryBits;

	body->CreateFixture(&fixtureDef);
	body->SetLinearVelocity(_b2Vec2(vel));

	return body;
}

b2Body* Box2dManager::addDynamicBox(CCPoint pos, CCSize size, CCPoint vel, uint16 categoryBits )
{
	static int counter = 0;
	counter ++;
	b2BodyDef bodyDef;
	bodyDef.type = b2_dynamicBody;
	bodyDef.position = _b2Vec2(pos);

	b2Body* body = world->CreateBody(&bodyDef);

	b2PolygonShape shape;
	shape.SetAsBox(TO_BOX2D(size.width) / 2, TO_BOX2D(size.height) / 2);

	b2FixtureDef fixtureDef;
	fixtureDef.shape = &shape;
	fixtureDef.density = 1.0f;
	fixtureDef.friction = 0.3f;
	fixtureDef.restitution = 0.3f;
	fixtureDef.filter.categoryBits = categoryBits;

	body->CreateFixture(&fixtureDef);

	body->SetLinearVelocity(_b2Vec2(vel));

	return body;
}

b2Body* Box2dManager::addKinematicConvex( cocos2d::CCPoint pos, const ccVertex2F* vertices, int32 verticesCount, cocos2d::CCPoint vel, uint16 categoryBits /*= CATEGORY_SCENERY*/ )
{
	b2BodyDef bodyDef;
	bodyDef.type = b2_kinematicBody;
	bodyDef.position = _b2Vec2(pos);

	b2Body *body = world->CreateBody(&bodyDef);


	b2PolygonShape shape;

	b2Vec2* verticesArray = new b2Vec2[verticesCount];
	CC_ASSERT(verticesArray != NULL);
	for(int i = 0; i < verticesCount; ++i)
		verticesArray[i] = b2Vec2(TO_BOX2D(vertices[i].x), TO_BOX2D(vertices[i].y));

	shape.Set(verticesArray, verticesCount);
	delete [] verticesArray;

	b2FixtureDef fixtureDef;
	fixtureDef.shape = &shape;
	fixtureDef.density = 1.0f;
	fixtureDef.friction = 1.0f;
	fixtureDef.restitution = 0.1f;
	fixtureDef.filter.categoryBits = categoryBits;

	body->CreateFixture(&fixtureDef);
	body->SetLinearVelocity(_b2Vec2(vel));
	return body;

}

b2Body* Box2dManager::addDynamicCircle(CCPoint pos, float size, CCPoint vel, uint16 categoryBits)
{
	static int counter = 0;
	counter ++;
	b2BodyDef bodyDef;
	bodyDef.type = b2_dynamicBody;
	bodyDef.position = _b2Vec2(pos);

	b2Body* body = world->CreateBody(&bodyDef);

	b2CircleShape shape;
	shape.m_radius = TO_BOX2D(size);

	b2FixtureDef fixtureDef;
	fixtureDef.shape = &shape;
	fixtureDef.density = 1.0f;
	fixtureDef.friction = 0.3f;
	fixtureDef.restitution = 0.2f;
	fixtureDef.filter.categoryBits = categoryBits;
	fixtureDef.filter.groupIndex = GROUP_PLAYER;

	body->CreateFixture(&fixtureDef);
	body->SetLinearVelocity(_b2Vec2(vel));

	return body;
}

void Box2dManager::setDebugDrawFlags(uint32 flags, bool isSet )
{
	if (isSet)
		debugDraw->AppendFlags(flags);
	else
		debugDraw->ClearFlags(flags);
}

void Box2dManager::destroyBody(b2Body* pBody)
{
	CC_ASSERT(world != NULL);

	if (std::find(to_delete.begin(), to_delete.end(), pBody) == to_delete.end())
		to_delete.push_back(pBody);
}

b2Body* Box2dManager::addStaticConvex( cocos2d::CCPoint pos, const ccVertex2F* vertices, int32 verticesCount, uint16 categoryBits /*= CATEGORY_MONSTER*/ )
{
	b2BodyDef bodyDef;
	bodyDef.type = b2_staticBody;
	bodyDef.position = _b2Vec2(pos);

	b2Body *body = world->CreateBody(&bodyDef);

	b2PolygonShape shape;

	CC_ASSERT(verticesCount <= 8);

	b2Vec2* verticesArray = new b2Vec2[verticesCount];
	CC_ASSERT(verticesArray != NULL);
	for(int i = 0; i < verticesCount; ++i)
		verticesArray[i] = b2Vec2(TO_BOX2D(vertices[i].x), TO_BOX2D(vertices[i].y));

	shape.Set(verticesArray, verticesCount);
	delete [] verticesArray;

	b2FixtureDef fixtureDef;
	fixtureDef.shape = &shape;
	fixtureDef.density = 1.0f;
	fixtureDef.friction = 1.0f;
	fixtureDef.restitution = 0.1f;
	fixtureDef.filter.categoryBits = categoryBits;

	body->CreateFixture(&fixtureDef);
	return body;
}

class RayCastCallback : public b2RayCastCallback, public Singleton<RayCastCallback>
{
public:
	RayCastCallback()
	{
		m_fixture = NULL;
	}

	void reset()
	{
		m_fixture = NULL;
	}

	bool success()
	{
		//TODO: more considerations
		return m_fixture != NULL;
	}

	float32 ReportFixture(b2Fixture* fixture, const b2Vec2& point,const b2Vec2& normal, float32 fraction)
	{
		m_fixture = fixture;
		m_point = point;
		m_normal = normal;
		m_fraction = fraction;
		return fraction;
	}
	b2Fixture* m_fixture;
	b2Vec2 m_point;
	b2Vec2 m_normal;
	float32 m_fraction;
};

bool Box2dManager::getRayCastPoint(const b2Vec2& start, const b2Vec2& end, b2Vec2& hitPoint, b2Fixture** m_fixture)
{
	RayCastCallback* cb = RayCastCallback::GetInstance();
	cb->reset();
	world->RayCast(cb, start, end);

	if (cb->success())
	{
		hitPoint = cb->m_point;
		if (m_fixture != NULL)
			*m_fixture = cb->m_fixture;
		return true;
	}
	else
	{
		return false;
	}
}

struct ContactInfoHelper
{
	//to make life easier
	ContactInfoHelper(b2Contact* contact)
	{
		f1 = contact->GetFixtureA();//f1 is enemy/scenery
		f2 = contact->GetFixtureB();//f2 is bullet

		cat1 = f1->GetFilterData().categoryBits;
		cat2 = f2->GetFilterData().categoryBits;

		b1 = f1->GetBody();
		b2 = f2->GetBody();
	}
	b2Fixture* f1;
	b2Fixture* f2;
	uint16 cat1, cat2;
	b2Body* b1;
	b2Body* b2;
};

void GlobalContactListener::BeginContact(b2Contact* contact)
{
	B2_NOT_USED(contact);
}

void GlobalContactListener::EndContact(b2Contact* contact)
{
	B2_NOT_USED(contact);
}

void GlobalContactListener::PreSolve(b2Contact* contact, const b2Manifold* oldManifold)
{
	ContactInfoHelper info(contact);

	if (info.cat2 == CATEGORY_PLAYER_LITTLEWATER)
	{
	//	info.b2->SetActive(false);
		contact->SetEnabled(false);
		return;
	}

	b2WorldManifold worldManifold;
	contact->GetWorldManifold(&worldManifold);

	//CCLOG("%d : %d", info.cat1, info.cat2);
	AdditionalInfo* addit_info = (AdditionalInfo*)info.b2->GetUserData();
	if (!addit_info)
		return;

	addit_info->setHitting();
	
	switch (info.cat1)
	{
	case CATEGORY_MONSTER:
		{
			if (addit_info->hit_ground/* && addit_info->hit_enemy*/)//particle on the ground can't attack monsters
			{
			//	contact->SetEnabled(false);
				break;
			}
			addit_info->hit_enemy = true;
			//damage
			Enemy* enemy = (Enemy*)info.b1->GetUserData();
			if (enemy != NULL)
			{
				int nWeaponID;

				switch(info.cat2)
				{
				case CATEGORY_PLAYER_WATER://water
					nWeaponID = WEAPON_WATER;
					break;
				case CATEGORY_PLAYER_ICE://ice
					nWeaponID = WEAPON_ICE;
					break;
				case CATEGORY_PLAYER_FIRE://fire
					nWeaponID = WEAPON_FIRE;
					break;
				case CATEGORY_PLAYER_ACID://acid
					nWeaponID = WEAPON_ACID;
					break;
				default: //bad
					CC_ASSERT(0);
				}
				if (CCRANDOM_0_1() > 0.98f)
				{
					CCPoint pos = _CCPoint(worldManifold.points[0]);
					CCPoint vel = _CCPoint(worldManifold.normal);
					getPlayerActor()->getLittleWater()->shootAt(pos, vel*4, 0.01,1);
				}
				enemy->Damage(nWeaponID);
			}
		}break;
	case CATEGORY_SCENERY:
		{
			if (addit_info != NULL)
			{
				addit_info->hit_ground = true;
				addit_info->batch_id = AdditionalInfo::WATER_ABOVE_GROUND;//[important]

				//slow particles down
				b2Vec2 v = addit_info->body->GetLinearVelocity();
				v.x *= 0.9f;
				addit_info->body->SetLinearVelocity(v);
			}

		}break;
	default: break;
	} 
}

void GlobalContactListener::PostSolve(b2Contact* contact, const b2ContactImpulse* impulse)
{
	B2_NOT_USED(contact);
	B2_NOT_USED(impulse);
}

