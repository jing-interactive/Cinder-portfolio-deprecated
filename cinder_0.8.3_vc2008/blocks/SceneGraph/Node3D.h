#pragma once

#include "cinder/gl/Material.h"
#include "NodeGL.h"

namespace ph { namespace nodes {

// Basic support for 3D nodes
typedef std::shared_ptr<class Node3D> Node3DRef;

class Node3D :
    public NodeGL
{
public:
    Node3D(void);
    virtual ~Node3D(void);

    //! the drawMesh function only draws a mesh without binding textures and shaders
    virtual void		drawWireframe(){}
    virtual void		treeDrawWireframe();

    // getters and setters
    virtual ci::Vec3f	getPosition() const { return mPosition; }
    virtual void		setPosition( float x, float y, float z ){ mPosition = ci::Vec3f(x, y, z); invalidateTransform(); }
    virtual void		setPosition( const ci::Vec3f &pt ){ mPosition = pt; invalidateTransform(); }

    virtual ci::Quatf	getRotation() const { return mRotation; }
    virtual void		setRotation( float radians ){ mRotation.set( ci::Vec3f::yAxis(), radians ); invalidateTransform(); }
    virtual void		setRotation( const ci::Vec3f &radians ){ mRotation.set( radians.x, radians.y, radians.z ); invalidateTransform(); }
    virtual void		setRotation( const ci::Vec3f &axis, float radians ){ mRotation.set( axis, radians ); invalidateTransform(); }
    virtual void		setRotation( const ci::Quatf &rot ){ mRotation = rot; invalidateTransform(); }

    virtual ci::Vec3f	getScale() const { return mScale; }
    virtual void		setScale( float scale ){ mScale = ci::Vec3f(scale, scale, scale); invalidateTransform(); }
    virtual void		setScale( float x, float y, float z ){ mScale = ci::Vec3f(x, y, z); invalidateTransform(); }
    virtual void		setScale( const ci::Vec3f &scale ){ mScale = scale; invalidateTransform(); }

    virtual ci::Vec3f	getAnchor() const { return mAnchor; }
    virtual void		setAnchor( float x, float y, float z ){ mAnchor = ci::Vec3f(x, y, z); invalidateTransform(); }
    virtual void		setAnchor( const ci::Vec3f &pt ){ mAnchor = pt; invalidateTransform(); }

    // stream support
    virtual inline std::string toString() const { return "Node3D"; }
protected: 
    ci::Vec3f	mPosition;
    ci::Quatf	mRotation;
    ci::Vec3f	mScale;
    ci::Vec3f	mAnchor;

    ci::gl::Material	mMaterial;

    // required function (see: class Node)
    virtual void transform() const;
};

} }