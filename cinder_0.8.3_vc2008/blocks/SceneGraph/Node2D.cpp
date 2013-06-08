#include "Node2D.h"

using namespace ci;

namespace ph { namespace nodes {

Node2D::Node2D(void)
{
	mPosition	= Vec2f::zero();
	mRotation	= Quatf::identity();
	mScale		= Vec2f::one();
	mAnchor		= Vec2f::zero();

	mAnchorIsPercentage = false;
}

Node2D::~Node2D(void)
{
}

Vec2f Node2D::screenToParent( const Vec2f &pt ) const
{
	Vec2f p = pt;

	Node2DRef node = getParent<Node2D>();
	if(node) p = node->screenToObject(p);

	return p;
}

Vec2f Node2D::screenToObject( const Vec2f &pt ) const
{
	// near plane intersection 
	Vec3f p0 = unproject(pt.x, pt.y, 0.0f);
	// far plane intersection 
	Vec3f p1 = unproject(pt.x, pt.y, 1.0f);

	// find (x, y) coordinates 
	float t = (0.0f - p0.z) / (p1.z - p0.z);
	float x = (p0.x + t * (p1.x - p0.x));
	float y = (p0.y + t * (p0.y - p1.y));

	return Vec2f(p0.x, p0.y);
}

Vec2f Node2D::parentToScreen( const Vec2f &pt ) const
{
	Vec2f p = pt;

	Node2DRef node = getParent<Node2D>();
	if(node) p = node->objectToScreen(p);

	return p;
}

Vec2f Node2D::parentToObject( const Vec2f &pt ) const
{
	Vec3f p = mTransform.inverted(5.96e-8f).transformPointAffine( Vec3f(pt, 0.0f) );
	return Vec2f(p.x, p.y);
}

Vec2f Node2D::objectToParent( const Vec2f &pt ) const
{
	Vec3f p = mTransform.transformPointAffine( Vec3f(pt, 0.0f) );
	return Vec2f(p.x, p.y);
}

Vec2f Node2D::objectToScreen( const Vec2f &pt ) const
{
	return project(pt);
}

void Node2D::transform() const
{
    // construct transformation matrix
    mTransform.setToIdentity();
    mTransform.translate( ci::Vec3f( mPosition, 0.0f ) );
    mTransform *= mRotation.toMatrix44();
    mTransform.scale( ci::Vec3f( mScale, 1.0f ) );

    if( mAnchorIsPercentage )
        mTransform.translate( ci::Vec3f( -mAnchor * getSize(), 0.0f ) );
    else
        mTransform.translate( ci::Vec3f( -mAnchor, 0.0f ) );

    // update world matrix (TODO will not work with cached matrix!)
    Node2DRef parent = getParent<Node2D>();
    if(parent)
        mWorldTransform = parent->mWorldTransform * mTransform;
    else mWorldTransform = mTransform;

    // TODO set mIsTransformInvalidated to false once the world matrix stuff has been rewritten
}


} }