/*
Copyright (C) 2010-2012 IJsfontein BV, Amsterdam

Permission is hereby granted, free of charge, to any person obtaining a copy of
this software and associated documentation files (the "Software"), to deal in
the Software without restriction, including without limitation the rights to
use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies
of the Software, and to permit persons to whom the Software is furnished to do
so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#pragma once

#include "cinder/AxisAlignedBox.h"
#include "cinder/Camera.h"
#include "cinder/Cinder.h"
#include "cinder/Color.h"
#include "cinder/Frustum.h"
#include "cinder/Matrix.h"
#include "cinder/Quaternion.h"
#include "cinder/Vector.h"
#include "cinder/app/AppBasic.h"
#include "cinder/gl/GlslProg.h"
#include "cinder/gl/Material.h"

#include <boost/enable_shared_from_this.hpp>

#include <iostream>
#include <deque>
#include <map>

namespace is 
{
typedef boost::shared_ptr<class Node>		NodeRef;
typedef boost::weak_ptr<class Node>			NodeWeakRef;

typedef std::deque<NodeRef>					NodeList;
typedef std::map<unsigned int, NodeWeakRef>	NodeMap;

class Node
	: public boost::enable_shared_from_this<Node>
{
public:
	Node(void);
	virtual ~Node(void);

	// UNIQUE IDENTIFIER & PICKING SUPPORT
	//! returns the node's unique identifier
	unsigned int		getUniqueId() const { return mId; };
	//! returns the node's unique color
	ci::Color			getUniqueColor() const { return idToColor(mId); };
	//! translates integer to a color
	static ci::Color	idToColor(unsigned int id) {	return ci::Color((id & 0xFF) / 255.0f, ((id >> 8) & 0xFF) / 255.0f, ((id >> 16) & 0xFF) / 255.0f); };
	//! translates a color to an integer
	static unsigned int	colorToId(ci::Color color) { return colorToId( (unsigned char)(color.r * 255), (unsigned char)(color.g * 255), (unsigned char)(color.b * 255) ); };
	//! translates a color to an integer
	static unsigned int	colorToId(unsigned char r, unsigned char g, unsigned char b) { return r + (g << 8) + (b << 16); };
	//! finds a node by its unique identifier (fast)
	static NodeRef		findNode(unsigned int id) { return nodeLookupTable[id].lock(); };
	
	// PARENT RETRIEVAL
	//! returns the node's parent node 
	NodeRef	getParent() const { return mParent.lock(); };
	//! returns the node's parent node (provide a templated function for easier down-casting of nodes)
	template <class T>
	boost::shared_ptr<T>	getParent() const { 
		return boost::shared_dynamic_cast<T>(mParent.lock()); 
	};
	//! returns a node higher up in the hierarchy of the desired type, if any
	template <class T>
	boost::shared_ptr<T>	getTreeParent() const { 
		boost::shared_ptr<T> node = boost::shared_dynamic_cast<T>(mParent.lock()); 
		if(node) return node;
		else if(mParent.lock()) return mParent.lock()->getTreeParent<T>();
		else return node;
	};

	// CHILD FUNCTIONS
	//! returns wether this node has a specific child
	bool hasChild(NodeRef node) const;
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

	//! removes this node from its parent
	void removeFromParent();
	//! puts this node on top of all its siblings
	void putOnTop();
	//! returns wether this node is on top of all its siblings
	bool isOnTop();		
	//! puts this node below all its siblings
	void moveToBottom();

	// VISIBILITY
	//! enables or disables visibility of this node (invisible nodes are not drawn and can not receive events, but they still receive updates)
	virtual void setVisible(bool visible=true) { bIsVisible = visible; };
	//! returns wether this node is visible
	virtual bool isVisible() const { return bIsVisible; };
	//! toggle visibility (hide if visible, show if invisible)
	virtual bool toggleVisible() { setVisible(!bIsVisible); return bIsVisible; };

	// FRUSTUM CULLING SUPPORT
	//! sets whether or not this node has been culled. This is not the same as the 'visible' flag, because culling is 
	//! a separate optimization process, whereas setting the visibility is a logical decision
	void setCulled(bool culled=true) { bIsCulled = culled; };
	//! returns wether this node is visible
	bool isCulled() const { return bIsCulled; };
	//! returns the object's axis aligned bounding box in object coordinates
	ci::AxisAlignedBox3f getBoundingBox() const { return mBoundingBox; }; 
	//! returns the object's axis aligned bounding box in object coordinates transformed by the supplied matrix
	ci::AxisAlignedBox3f getBoundingBox( const ci::Matrix44f &transform ) const { return mBoundingBox.transformed(transform); };
	//! returns an axis aligned bounding box in world coordinates that precisely contains the node and all its children
	ci::AxisAlignedBox3f getTreeBoundingBox();
	//! sets the bounding box and signals the node that the bounding box has changed
	void setBoundingBox( const ci::AxisAlignedBox3f &box ) { mBoundingBox = box; invalidateBoundingBox(); };
	//! sets the bounding box and signals the node that the bounding box has changed
	void setBoundingBox( const ci::Vec3f &min, const ci::Vec3f &max ) { mBoundingBox = ci::AxisAlignedBox3f(min, max); invalidateBoundingBox(); };
	//! signals the node that the bounding box has changed. Will also recursively invalidate its parent and children
	void invalidateBoundingBox();
	//! draws the node's tree bounding box
	void drawBoundingBox() { ci::gl::drawStrokedCube( getTreeBoundingBox() ); };
	//! performs culling on this node and all its children
	void treeCull( ci::Frustumf &frustum );

	//! 
	virtual void setClickable(bool clickable=true) { bIsClickable = clickable; };
	//! returns wether this node is clickable
	virtual bool isClickable() const { return bIsClickable; };

	//! returns the transformation matrix of this node
	const ci::Matrix44f& getTransform() const { return mTransform; };
	//! returns the accumulated transformation matrix of this node
	const ci::Matrix44f& getWorldTransform() const { return mWorldTransform; };

	//! signal parent that this node has been clicked or activated
	virtual void selectChild(NodeRef node) {};
	//! signal parent that this node has been released or deactivated
	virtual void deselectChild(NodeRef node) {};

	// NODE EVENTS
	virtual void setup() {};
	virtual void shutdown() {};
	virtual void update( double elapsed ) {};
	virtual void draw() {};

	virtual bool mouseMove( ci::app::MouseEvent event ) { return false; };
	virtual bool mouseDown( ci::app::MouseEvent event ) { return false; };
	virtual bool mouseDrag( ci::app::MouseEvent event ) { return false; };
	virtual bool mouseUp( ci::app::MouseEvent event ) { return false; };

	// support for easy picking system
	virtual bool mouseUpOutside( ci::app::MouseEvent event ) { return false; };

	virtual bool keyDown( ci::app::KeyEvent event ) { return false; };
	virtual bool keyUp( ci::app::KeyEvent event ) { return false; };

	virtual bool resize( ci::app::ResizeEvent event ) { return false; };

	// TREE TRAVERSING 
	//! calls the setup() function of this node and all its decendants
	virtual void treeSetup();
	//! calls the shutdown() function of this node and all its decendants
	virtual void treeShutdown();
	//! calls the update() function of this node and all its decendants
	virtual void treeUpdate(double elapsed);
	//! calls the draw() function of this node and all its decendants
	virtual void treeDraw();

	//! calls the mouseMove() function of this node and all its decendants until a TRUE is passed back
	virtual bool treeMouseMove( ci::app::MouseEvent event );
	//! calls the mouseDown() function of this node and all its decendants until a TRUE is passed back
	virtual bool treeMouseDown( ci::app::MouseEvent event );
	//! calls the mouseDrag() function of this node and all its decendants until a TRUE is passed back
	virtual bool treeMouseDrag( ci::app::MouseEvent event );
	//! calls the mouseUp() function of this node and all its decendants until a TRUE is passed back
	virtual bool treeMouseUp( ci::app::MouseEvent event );
	
	//! calls the keyDown() function of this node and all its decendants until a TRUE is passed back
	virtual bool treeKeyDown( ci::app::KeyEvent event );
	//! calls the keyUp() function of this node and all its decendants until a TRUE is passed back
	virtual bool treeKeyUp( ci::app::KeyEvent event );

	//! calls the resize() function of this node and all its decendants until a TRUE is passed back
	virtual bool treeResize( ci::app::ResizeEvent event );

	//! recursively draws the bounding box of the node and its children
	virtual void treeDrawBoundingBox();

	// STREAM SUPPORT
	//! override this function to give your node a proper name of its own
	virtual inline std::string toString() { return "Node"; };
	//!
	friend std::ostream& operator<<(std::ostream& s, Node& o) { return s << "[" << o.toString() << "]"; };

protected:
	//! this node's parent node, if any
	NodeWeakRef		mParent;
	//! this node's children, if any
	NodeList		mChildren;

	//! this node's combined transformations (position, rotation, scale)
	ci::Matrix44f	mTransform;
	//! this node's transformations, combined with its parent's transformations, etc.
	ci::Matrix44f	mWorldTransform;

protected:
	//! sets the node's parent node (using weak reference to avoid objects not getting destroyed)
	void setParent(NodeRef node);

	//! helper function for coordinate transformations
	ci::Vec3f	unproject( const ci::Vec3f &pt );

	//! function that is called right before drawing this node
	virtual void begin() {};
	//! function that is called right after drawing this node
	virtual void end() {};

	//! required transform() function to populate the transform matrix
	virtual void transform() = 0;

private:
	//! state variables
	bool		bIsVisible;
	bool		bIsCulled;
	bool		bIsClickable;

	//! unique identifier of this node
	const unsigned int mId;

	//! if TRUE, will recalculate mTreeBoundingBox when calling 'getTreeBoundingBox()' 
	bool bIsBoundingBoxInvalid;
	//! stores the bounding box in OBJECT SPACE coordinates
	ci::AxisAlignedBox3f	mBoundingBox;
	//! stores the combined bounding box of this node and all its children in WORLD SPACE coordinates
	ci::AxisAlignedBox3f	mTreeBoundingBox;

private:
	//! refCount is used to count the number of Node instances for debugging purposes
	static int			refCount;
	//! idCount is used to generate new unique id's
	static unsigned int idCount;
	//! nodeLookupTable allows us to quickly find a Node by id
	static NodeMap		nodeLookupTable;
};

// Basic shader support (will probably be changed in the near future)
typedef boost::shared_ptr<class NodeGL> NodeGLRef;

class NodeGL :
	public Node
{
public:
	NodeGL(void) {};
	virtual ~NodeGL(void) {};	

	// shader support
	void	setShaderUniform( const std::string &name, int data ) { if(mShader) mShader.uniform(name, data); };
	void	setShaderUniform( const std::string &name, const ci::Vec2i &data ) { if(mShader) mShader.uniform(name, data); };
	void	setShaderUniform( const std::string &name, const int *data, int count ) { if(mShader) mShader.uniform(name, data, count); };		
	void	setShaderUniform( const std::string &name, const ci::Vec2i *data, int count ) { if(mShader) mShader.uniform(name, data, count); };	
	void	setShaderUniform( const std::string &name, float data ) { if(mShader) mShader.uniform(name, data); };
	void	setShaderUniform( const std::string &name, const ci::Vec2f &data ) { if(mShader) mShader.uniform(name, data); };
	void	setShaderUniform( const std::string &name, const ci::Vec3f &data ) { if(mShader) mShader.uniform(name, data); };
	void	setShaderUniform( const std::string &name, const ci::Vec4f &data ) { if(mShader) mShader.uniform(name, data); };
	void	setShaderUniform( const std::string &name, const ci::Color &data ) { if(mShader) mShader.uniform(name, data); };
	void	setShaderUniform( const std::string &name, const ci::ColorA &data ) { if(mShader) mShader.uniform(name, data); };
	void	setShaderUniform( const std::string &name, const ci::Matrix33f &data, bool transpose = false ) { if(mShader) mShader.uniform(name, data, transpose); };
	void	setShaderUniform( const std::string &name, const ci::Matrix44f &data, bool transpose = false ) { if(mShader) mShader.uniform(name, data, transpose); };
	void	setShaderUniform( const std::string &name, const float *data, int count ) { if(mShader) mShader.uniform(name, data, count); };
	void	setShaderUniform( const std::string &name, const ci::Vec2f *data, int count ) { if(mShader) mShader.uniform(name, data, count); };
	void	setShaderUniform( const std::string &name, const ci::Vec3f *data, int count ) { if(mShader) mShader.uniform(name, data, count); };
	void	setShaderUniform( const std::string &name, const ci::Vec4f *data, int count ) { if(mShader) mShader.uniform(name, data, count); };

	void	loadShader( const std::string &vert, const std::string &frag );

	void	bindShader() { if(mShader) mShader.bind(); };
	void	unbindShader() { if(mShader) mShader.unbind(); };

	// stream support
	virtual inline std::string toString() { return "NodeGL"; };
protected:
	ci::gl::GlslProg	mShader;
};

// Basic support for 2D nodes
typedef boost::shared_ptr<class Node2D> Node2DRef;

class Node2D :
	public NodeGL
{
public:
	Node2D(void);
	virtual ~Node2D(void);

	// getters and setters
	virtual ci::Vec2f	getPosition() const { return mPosition; };
	virtual void		setPosition( float x, float y ) { mPosition = ci::Vec2f(x, y); };
	virtual void		setPosition( const ci::Vec2f &pt ) { mPosition = pt; };

	virtual ci::Quatf	getRotation() const { return mRotation; };
	virtual void		setRotation( float radians ) { mRotation.set( ci::Vec3f::zAxis(), radians ); };
	virtual void		setRotation( const ci::Quatf &rot ) { mRotation = rot; };

	virtual ci::Vec2f	getScale() const { return mScale; };
	virtual void		setScale( float scale ) { mScale = ci::Vec2f(scale, scale); };
	virtual void		setScale( float x, float y ) { mScale = ci::Vec2f(x, y); };
	virtual void		setScale( const ci::Vec2f &scale ) { mScale = scale; };

	virtual ci::Vec2f	getAnchor() const { return mAnchor; }
	virtual void		setAnchor( float x, float y ) { mAnchor = ci::Vec2f(x, y); bIsAnchorPercentage = false; };
	virtual void		setAnchor( const ci::Vec2f &pt ) { mAnchor = pt; bIsAnchorPercentage = false; };

	//
	virtual	ci::Vec2f	getAnchorPercentage() const;
	virtual	void		setAnchorPercentage(float px, float py) { mAnchor = ci::Vec2f(px, py) * getSize(); bIsAnchorPercentage = true; };
	virtual	void		setAnchorPercentage(const ci::Vec2f &pt) { mAnchor = pt * getSize(); bIsAnchorPercentage = true; };

	// 
	virtual float		getWidth() const { return mWidth; };
	virtual float		getScaledWidth() const { return mWidth * mScale.x; };
	virtual float		getHeight() const { return mHeight; };
	virtual float		getScaledHeight() const { return mHeight * mScale.y; };
	virtual ci::Vec2f	getSize() const { return ci::Vec2f(mWidth, mHeight); };
	virtual ci::Vec2f	getScaledSize() const { return getSize() * mScale; };
	virtual ci::Rectf	getBounds() const { return ci::Rectf(ci::Vec2f::zero(), getSize()); };
	virtual ci::Rectf	getScaledBounds() const { return ci::Rectf(ci::Vec2f::zero(), getScaledSize()); };

	inline	void	setWidth(float w) { setSize(w, mHeight); };
	inline	void	setHeight(float h) { setSize(mWidth, h); };
			void	setSize(float w, float h);
	inline	void	setSize( const ci::Vec2i &size ) { setSize( (float)size.x, (float)size.y ); };
	inline	void	setBounds( const ci::Rectf &bounds ) { setSize( bounds.getWidth(), bounds.getHeight() ); };

	//
	void setViewport(int w, int h);
	void resetViewport();

	//
	void enableScissor(float x, float y, float w, float h);
	void disableScissor();	

	// conversions from screen to world to object coordinates and vice versa
	virtual ci::Vec2f screenToParent( const ci::Vec2f &pt );
	virtual ci::Vec2f screenToObject( const ci::Vec2f &pt );
	virtual ci::Vec2f parentToScreen( const ci::Vec2f &pt );
	virtual ci::Vec2f parentToObject( const ci::Vec2f &pt );
	virtual ci::Vec2f objectToParent( const ci::Vec2f &pt );
	virtual ci::Vec2f objectToScreen( const ci::Vec2f &pt );

	// stream support
	virtual inline std::string toString() { return "Node2D"; };
protected: 
	ci::Vec2f	mPosition;
	ci::Quatf	mRotation;
	ci::Vec2f	mScale;
	ci::Vec2f	mAnchor;

	bool		bIsAnchorPercentage;

	float		mWidth;
	float		mHeight;

	// required function (see: class Node)
	virtual void transform() {
		// construct transformation matrix
		mTransform.setToIdentity();
		mTransform.translate( ci::Vec3f( mPosition, 0.0f ) );
		mTransform *= mRotation.toMatrix44();
		mTransform.scale( ci::Vec3f( mScale, 1.0f ) );
		mTransform.translate( ci::Vec3f( -mAnchor, 0.0f ) );

		// update world matrix
		ci::Matrix44f temp(mWorldTransform);
		Node2DRef parent = getParent<Node2D>();
		if(parent)
			mWorldTransform = parent->mWorldTransform * mTransform;
		else mWorldTransform = mTransform;

		// update bounding box on request
		if(temp != mWorldTransform)
			invalidateBoundingBox();
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
	public NodeGL
{
public:
	Node3D(void);
	virtual ~Node3D(void);

	//! the drawMesh function only draws a mesh without binding textures and shaders
	virtual void		drawWireframe() {};
	//! support for drawing a whole tree of nodes as wireframes
	virtual void		treeDrawWireframe();

	// getters and setters
	virtual ci::Vec3f	getPosition() const { return mPosition; };
	virtual void		setPosition( float x, float y, float z ) { mPosition = ci::Vec3f(x, y, z); };
	virtual void		setPosition( const ci::Vec3f &pt ) { mPosition = pt; };

	virtual ci::Quatf	getRotation() const { return mRotation; };
	virtual void		setRotation( float radians ) { mRotation.set( ci::Vec3f::yAxis(), radians ); };
	virtual void		setRotation( const ci::Vec3f &radians ) { mRotation.set( radians.x, radians.y, radians.z ); };
	virtual void		setRotation( const ci::Vec3f &axis, float radians ) { mRotation.set( axis, radians ); };
	virtual void		setRotation( const ci::Quatf &rot ) { mRotation = rot; };

	virtual ci::Vec3f	getScale() const { return mScale; };
	virtual void		setScale( float scale ) { mScale = ci::Vec3f(scale, scale, scale); };
	virtual void		setScale( float x, float y, float z ) { mScale = ci::Vec3f(x, y, z); };
	virtual void		setScale( const ci::Vec3f &scale ) { mScale = scale; };

	virtual ci::Vec3f	getAnchor() const { return mAnchor; }
	virtual void		setAnchor( float x, float y, float z ) { mAnchor = ci::Vec3f(x, y, z); };
	virtual void		setAnchor( const ci::Vec3f &pt ) { mAnchor = pt; };

	// stream support
	virtual inline std::string toString() { return "Node3D"; };
protected: 
	ci::Vec3f	mPosition;
	ci::Quatf	mRotation;
	ci::Vec3f	mScale;
	ci::Vec3f	mAnchor;

	ci::gl::Material	mMaterial;

	// required function (see: class Node)
	virtual void transform() {
		// construct transformation matrix
		mTransform.setToIdentity();
		mTransform.translate( mPosition );
		mTransform *= mRotation.toMatrix44();
		mTransform.scale( mScale );
		mTransform.translate( -mAnchor );

		// update world matrix
		ci::Matrix44f temp(mWorldTransform);
		Node3DRef parent = getParent<Node3D>();
		if(parent)
			mWorldTransform = parent->mWorldTransform * mTransform;
		else mWorldTransform = mTransform;

		// update bounding box on request
		if(temp != mWorldTransform) 
			invalidateBoundingBox();
	};
};

} // namespace cinder