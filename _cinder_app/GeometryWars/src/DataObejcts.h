#pragma once

#include "cinder/Vector.h"
#include "cinder/Color.h"

static int objectIdCounter = 0;
	
class ColorMaskArea {
public:
	ColorMaskArea()
	{
		objectId = objectIdCounter++;
		lifetime = 0.0f;
		baseSize = 10.0f;
		update( 0.0f );
	}
	
	void update( const float deltaTime )
	{
		lifetime += deltaTime;
		size = baseSize + lifetime * growthRate;
		color.a = 1.0f - lifetime / maxLifetime;
	}
	
	bool isExpired() const { return lifetime >= maxLifetime; }
	
	bool operator==( ColorMaskArea* other ) const { return other->objectId == objectId; }
	
	int objectId;
	ci::Vec2i position;
	ci::ColorA color;
	float lifetime;
	float growthRate;
	float size;
	float baseSize;
	float maxLifetime;
};
	
class Character {
public:
	Character()
	{
		scale = ci::Vec3f::one() * 2.5f;
	}
	
	ci::Vec3f position;
	ci::Vec3f velocity;
	ci::Vec3f rotation;
	ci::Vec3f scale;
	ci::ColorA color;
	
	void update( const float deltaTime )
	{
		position += velocity * deltaTime;
		rotation += ci::Vec3f( 10, 10, 10 ) * deltaTime;
	}
};
