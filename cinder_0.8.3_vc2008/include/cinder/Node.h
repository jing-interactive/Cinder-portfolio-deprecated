// Copyright notice goes here //

#pragma once

#include "cinder/Cinder.h"
#include "cinder/app/AppBasic.h"

#include <boost/enable_shared_from_this.hpp>
#include <iostream>
#include <deque>

namespace cinder {

using namespace app;
using namespace std;

typedef boost::shared_ptr<class Node>	NodeRef;
typedef boost::weak_ptr<class Node>		NodeWeakRef;
typedef deque<NodeRef>					NodeList;

class Node
	: public boost::enable_shared_from_this<Node>
{
public:
	Node(void);
	Node(const string &name);
	virtual ~Node(void);
	
	//! sets the node's parent node (using weak reference to avoid objects not getting destroyed)
	void setParent(NodeRef node){ mParent = NodeWeakRef(node); };
	//! returns the node's parent node 
	NodeRef	getParent(){ return mParent.lock(); };
	//! returns the node's parent node (provide a templated function for easier down-casting of nodes)
	template <class T>
	boost::shared_ptr<T>	getParent(){ 
		return boost::shared_dynamic_cast<T>(mParent.lock()); 
	};
	//! returns a node higher up in the hierarchy of the desired type, if any
	template <class T>
	boost::shared_ptr<T>	getDeepParent(){ 
		boost::shared_ptr<T> node = boost::shared_dynamic_cast<T>(mParent.lock()); 
		if(node) return node;
		else if(mParent.lock()) return mParent.lock()->getDeepParent<T>();
		else return node;
	};

	// parent functions
	//! returns wether this node has a specific child
	bool hasChild(NodeRef node);
	//! adds a child to this node if it wasn't already a child of this node
	void addChild(NodeRef node); 
	//! removes a specific child from this node
	void removeChild(NodeRef node);
	//! removes all children of this node
	void removeChildren();
	//! puts a specific child on top of all other children of this node
	void putOnTop(NodeRef node);
	//! returns wether a specific child is on top of all other children
	bool isOnTop(NodeRef node);		
	//! puts a specific child below all other children of this node
	void moveToBottom(NodeRef node);

	// child functions
	//! removes this node from its parent
	void removeFromParent();
	//! puts this node on top of all its siblings
	void putOnTop();
	//! returns wether this node is on top of all its siblings
	bool isOnTop();		
	//! puts this node below all its siblings
	void moveToBottom();

	//! enables or disables visibility of this node (invisible nodes are not drawn and can not receive events, but they still receive updates)
	virtual void setVisible(bool visible=true){ bIsVisible = visible; };
	//! returns wether this node is visible
	virtual bool isVisible(){ return bIsVisible; };

	//! returns the transformation matrix of this node
	const Matrix44f& getTransform(){ return mTransform; };

	//! returns the transformation matrix of this node
	const string& getName(){ return name; };

	//! signal parent that this node has been clicked or activated
	virtual void selectChild(NodeRef node){};
	//! signal parent that this node has been released or deactivated
	virtual void deselectChild(NodeRef node){};

	// tree parse functions
	//! calls the setup() function of this node and all its decendants
	virtual void deepSetup();
	//! calls the shutdown() function of this node and all its decendants
	virtual void deepShutdown();
	//! calls the transform() function of this node and all its decendants
	virtual void deepTransform(const Matrix44f &world);
	//! calls the update() function of this node and all its decendants
	virtual void deepUpdate(double elapsed);
	//! calls the draw() function of this node and all its decendants
	virtual void deepDraw();

	virtual void setup(){};
	virtual void shutdown(){};
	virtual void update( double elapsed ){};
	virtual void draw(){};

	// supported events
	//! calls the mouseMove() function of this node and all its decendants until a TRUE is passed back
	virtual bool deepMouseMove( MouseEvent event );
	//! calls the mouseDown() function of this node and all its decendants until a TRUE is passed back
	virtual bool deepMouseDown( MouseEvent event );
	//! calls the mouseDrag() function of this node and all its decendants until a TRUE is passed back
	virtual bool deepMouseDrag( MouseEvent event );
	//! calls the mouseUp() function of this node and all its decendants until a TRUE is passed back
	virtual bool deepMouseUp( MouseEvent event );

	virtual bool mouseMove( MouseEvent event ){ return false; };
	virtual bool mouseDown( MouseEvent event ){ return false; };
	virtual bool mouseDrag( MouseEvent event ){ return false; };
	virtual bool mouseUp( MouseEvent event ){ return false; };
	
	//! calls the keyDown() function of this node and all its decendants until a TRUE is passed back
	virtual bool deepKeyDown( KeyEvent event );
	//! calls the keyUp() function of this node and all its decendants until a TRUE is passed back
	virtual bool deepKeyUp( KeyEvent event );

	virtual bool keyDown( KeyEvent event ){ return false; };
	virtual bool keyUp( KeyEvent event ){ return false; };

	//! calls the resize() function of this node and all its decendants until a TRUE is passed back
	bool deepResize( ResizeEvent event );

	virtual bool resize( ResizeEvent event ){ return false; };

	// stream support
	friend ostream& operator<<(ostream& s, Node& o){ return s << o.name; };
protected:
	bool		bIsVisible;

	NodeWeakRef	mParent;
	NodeList	mChildren;

	Matrix44f	mTransform;
	Matrix44f	mWorldTransform;
protected:
	//! helper function for coordinate transformations
	Vec3f	unproject( const Vec3f &pt );

	//! function that is called right before drawing this node
	virtual void begin(){};
	//! function that is called right after drawing this node
	virtual void end(){};

	//! required transform() function to populate the transform matrix
	virtual void transform() = 0;
private:
	static int refCount;
	const string name;
};

// Basic support for 2D nodes
typedef boost::shared_ptr<class Node2D> Node2DRef;

class Node2D :
	public Node
{
public:
	Node2D(void);
	Node2D(const string &name);
	virtual ~Node2D(void);

	// getters and setters
	virtual Vec2f	getPosition(){ return mPosition; };
	virtual void	setPosition( float x, float y ){ mPosition = Vec2f(x, y); };
	virtual void	setPosition( const Vec2f &pt ){ mPosition = pt; };

	virtual Quatf	getRotation(){ return mRotation; };
	virtual void	setRotation( float radians ){ mRotation.set( Vec3f::zAxis(), radians ); };
	virtual void	setRotation( const Quatf &rot ){ mRotation = rot; };

	virtual Vec2f	getScale(){ return mScale; };
	virtual void	setScale( float scale ){ mScale = Vec2f(scale, scale); };
	virtual void	setScale( float x, float y ){ mScale = Vec2f(x, y); };
	virtual void	setScale( const Vec2f &scale ){ mScale = scale; };

	virtual Vec2f	getAnchor(){ return mAnchor; }
	virtual void	setAnchor( float x, float y ){ mAnchor = Vec2f(x, y); };
	virtual void	setAnchor( const Vec2f &pt ){ mAnchor = pt; };

	//
	virtual	Vec2f	getAnchorPercentage();
	virtual	void	setAnchorPercentage(float px, float py){ mAnchor = Vec2f(px, py) * getSize(); };
	virtual	void	setAnchorPercentage(const Vec2f &pt){ mAnchor = pt * getSize(); };

	// 
	virtual float getWidth(){ return mWidth; };
	virtual float getScaledWidth(){ return mWidth * mScale.x; };
	virtual float getHeight(){ return mHeight; };
	virtual float getScaledHeight(){ return mHeight * mScale.y; };
	virtual Vec2f getSize(){ return Vec2f(mWidth, mHeight); };
	virtual Vec2f getScaledSize(){ return getSize() * mScale; };
	virtual Rectf getBounds(){ return Rectf(Vec2f::zero(), getSize()); };
	virtual Rectf getScaledBounds(){ return Rectf(Vec2f::zero(), getScaledSize()); };

	virtual void	setWidth(float w){ mWidth=w; };
	virtual void	setHeight(float h){ mHeight=h; };
	virtual void	setSize(float w, float h){ mWidth=w; mHeight=h; };
	virtual void	setSize( const Vec2i &size ){ mWidth=size.x; mHeight=size.y; };
	virtual void	setBounds( const Rectf &bounds ){ mWidth=bounds.getWidth(); mHeight=bounds.getHeight(); };

	//
	void setViewport(int w, int h);
	void resetViewport();

	//
	void enableScissor(float x, float y, float w, float h);
	void disableScissor();	

	// conversions from screen to world to object coordinates and vice versa
	virtual Vec2f screenToParent( const Vec2f &pt );
	virtual Vec2f screenToObject( const Vec2f &pt );
	virtual Vec2f parentToScreen( const Vec2f &pt );
	virtual Vec2f parentToObject( const Vec2f &pt );
	virtual Vec2f objectToParent( const Vec2f &pt );
	virtual Vec2f objectToScreen( const Vec2f &pt );
protected: 
	Vec2f		mPosition;
	Quatf		mRotation;
	Vec2f		mScale;
	Vec2f		mAnchor;

	float		mWidth;
	float		mHeight;

	// required function (see: class Node)
	virtual void transform(){
		mTransform.setToIdentity();
		mTransform.translate( Vec3f( mPosition, 0.0f ) );
		mTransform *= mRotation.toMatrix44();
		mTransform.scale( Vec3f( mScale, 1.0f ) );
		mTransform.translate( Vec3f( -mAnchor, 0.0f ) );
	};
private:
	// for restoring current viewport
	GLboolean	bRestoreViewport;
	GLint		mStoredViewport[4];

	// for restoring current scissor box
	bool		bScissorEnabled;
	GLboolean	bRestoreScissor;
	GLint		mStoredScissor[4];
};

// Basic support for 3D nodes
typedef boost::shared_ptr<class Node3D> Node3DRef;

class Node3D :
	public Node
{
public:
	Node3D(void);
	Node3D(const string &name);
	virtual ~Node3D(void);

	// getters and setters
	virtual Vec3f	getPosition(){ return mPosition; };
	virtual void	setPosition( float x, float y, float z ){ mPosition = Vec3f(x, y, z); };
	virtual void	setPosition( const Vec3f &pt ){ mPosition = pt; };

	virtual Quatf	getRotation(){ return mRotation; };
	virtual void	setRotation( float radians ){ mRotation.set( Vec3f::zAxis(), radians ); };
	virtual void	setRotation( const Vec3f &axis, float radians ){ mRotation.set( axis, radians ); };
	virtual void	setRotation( const Quatf &rot ){ mRotation = rot; };

	virtual Vec3f	getScale(){ return mScale; };
	virtual void	setScale( float scale ){ mScale = Vec3f(scale, scale, scale); };
	virtual void	setScale( float x, float y, float z ){ mScale = Vec3f(x, y, z); };
	virtual void	setScale( const Vec3f &scale ){ mScale = scale; };

	virtual Vec3f	getAnchor(){ return mAnchor; }
	virtual void	setAnchor( float x, float y, float z ){ mAnchor = Vec3f(x, y, z); };
	virtual void	setAnchor( const Vec3f &pt ){ mAnchor = pt; };
protected: 
	Vec3f		mPosition;
	Quatf		mRotation;
	Vec3f		mScale;
	Vec3f		mAnchor;

	// required function (see: class Node)
	virtual void transform(){
		mTransform.setToIdentity();
		mTransform.translate( mPosition );
		mTransform *= mRotation.toMatrix44();
		mTransform.scale( mScale );
		mTransform.translate( -mAnchor );
	};
};

} // namespace cinder
