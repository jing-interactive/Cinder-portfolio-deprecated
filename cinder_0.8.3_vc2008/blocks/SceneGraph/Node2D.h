#pragma once

#include "NodeGL.h"

namespace ph { namespace nodes {

// Basic support for 2D nodes
typedef std::shared_ptr<class Node2D> Node2DRef;

class Node2D :
    public NodeGL
{
public:
    Node2D(void);
    virtual ~Node2D(void);

    // getters and setters
    virtual ci::Vec2f	getPosition() const { return mPosition; }
    virtual void		setPosition( float x, float y ){ mPosition = ci::Vec2f(x, y); invalidateTransform(); }
    virtual void		setPosition( const ci::Vec2f &pt ){ mPosition = pt; invalidateTransform(); }

    virtual ci::Quatf	getRotation() const { return mRotation; }
    virtual void		setRotation( float radians ){ mRotation.set( ci::Vec3f::zAxis(), radians ); invalidateTransform(); }
    virtual void		setRotation( const ci::Quatf &rot ){ mRotation = rot; invalidateTransform(); }

    virtual ci::Vec2f	getScale() const { return mScale; }
    virtual void		setScale( float scale ){ mScale = ci::Vec2f(scale, scale); invalidateTransform(); }
    virtual void		setScale( float x, float y ){ mScale = ci::Vec2f(x, y); invalidateTransform(); }
    virtual void		setScale( const ci::Vec2f &scale ){ mScale = scale; invalidateTransform(); }

    virtual ci::Vec2f	getAnchor() const { return mAnchorIsPercentage ? mAnchor * getSize() : mAnchor; }
    virtual void		setAnchor( float x, float y ){ mAnchor = ci::Vec2f(x, y); mAnchorIsPercentage = false; invalidateTransform(); }
    virtual void		setAnchor( const ci::Vec2f &pt ){ mAnchor = pt; mAnchorIsPercentage = false; invalidateTransform(); }

    virtual	ci::Vec2f	getAnchorPercentage() const { return mAnchorIsPercentage ? mAnchor : mAnchor / getSize(); }
    virtual	void		setAnchorPercentage(float px, float py){ mAnchor = ci::Vec2f(px, py); mAnchorIsPercentage = true; invalidateTransform(); }
    virtual	void		setAnchorPercentage(const ci::Vec2f &pt){ mAnchor = pt; mAnchorIsPercentage = true; invalidateTransform(); }

    // 
    virtual float		getWidth() const { return mWidth; }
    virtual float		getScaledWidth() const { return mWidth * mScale.x; }
    virtual float		getHeight() const { return mHeight; }
    virtual float		getScaledHeight() const { return mHeight * mScale.y; }
    virtual ci::Vec2f	getSize() const { return ci::Vec2f(mWidth, mHeight); }
    virtual ci::Vec2f	getScaledSize() const { return getSize() * mScale; }
    virtual ci::Rectf	getBounds() const { return ci::Rectf(ci::Vec2f::zero(), getSize()); }
    virtual ci::Rectf	getScaledBounds() const { return ci::Rectf(ci::Vec2f::zero(), getScaledSize()); }

    virtual void		setWidth(float w){ mWidth=w; }
    virtual void		setHeight(float h){ mHeight=h; }
    virtual void		setSize(float w, float h){ mWidth=w; mHeight=h; }
    virtual void		setSize( const ci::Vec2i &size ){ mWidth=(float)size.x; mHeight=(float)size.y; }
    virtual void		setBounds( const ci::Rectf &bounds ){ mWidth=bounds.getWidth(); mHeight=bounds.getHeight(); }

    // conversions from screen to world to object coordinates and vice versa
    virtual ci::Vec2f screenToParent( const ci::Vec2f &pt ) const ;
    virtual ci::Vec2f screenToObject( const ci::Vec2f &pt ) const ;
    virtual ci::Vec2f parentToScreen( const ci::Vec2f &pt ) const ;
    virtual ci::Vec2f parentToObject( const ci::Vec2f &pt ) const ;
    virtual ci::Vec2f objectToParent( const ci::Vec2f &pt ) const ;
    virtual ci::Vec2f objectToScreen( const ci::Vec2f &pt ) const ;

    // stream support
    virtual inline std::string toString() const { return "Node2D"; }
protected: 
    ci::Vec2f	mPosition;
    ci::Quatf	mRotation;
    ci::Vec2f	mScale;
    ci::Vec2f	mAnchor;

    bool		mAnchorIsPercentage;

    float		mWidth;
    float		mHeight;

    // required function (see: class Node)
    virtual void transform() const {
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

        // TODO set mIsTransformValidated to false once the world matrix stuff has been rewritten
    }
};

}}