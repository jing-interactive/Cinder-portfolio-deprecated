// Copyright notice goes here //

#include "cinder/Node.h"

using namespace ci;
using namespace std;

int Node::refCount = 0;

Node::Node(void)
	: name("Node")
{
	// default constructor for [Node]

	refCount++;

	mTransform.setToIdentity();
	mWorldTransform.setToIdentity();

	bIsVisible = true;
}

Node::Node(const string &name)
	: name(name)
{
	// constructor for a differently named node 
	// (wish there was a way to avoid having basically the same constructors)

	refCount++;

	mTransform.setToIdentity();
	mWorldTransform.setToIdentity();

	bIsVisible = true;
}

Node::~Node(void)
{
	// remove all children safely
	removeChildren();

	//
	refCount--;
}

void Node::removeFromParent()
{
	NodeRef node = mParent.lock();
	if(node) node->removeChild( shared_from_this() );
}

void Node::addChild(NodeRef node)
{
	if(!hasChild(node))
	{
		// remove child from current parent
		NodeRef parent = node->getParent();
		if(parent) parent->removeChild(node);

		// add to children
		mChildren.push_back(node);

		// set parent
		node->setParent( shared_from_this() );
	}
}

void Node::removeChild(NodeRef node)
{
	NodeList::iterator itr = std::find(mChildren.begin(), mChildren.end(), node);
	if(itr != mChildren.end()) 
	{
		// reset parent
		(*itr)->setParent( NodeRef() );

		// remove from children
		mChildren.erase(itr);
	}
}

void Node::removeChildren()
{
	NodeList::iterator itr;
	for(itr=mChildren.begin();itr!=mChildren.end();)
	{
		// reset parent
		(*itr)->setParent( NodeRef() );

		// remove from children
		itr = mChildren.erase(itr);
	}
}

bool Node::hasChild(NodeRef node)
{
	NodeList::iterator itr = std::find(mChildren.begin(), mChildren.end(), node);
	return(itr != mChildren.end());
}

void Node::putOnTop()
{
	NodeRef parent = getParent();
	if(parent) parent->putOnTop( shared_from_this() );
}

void Node::putOnTop(NodeRef node)
{
	// remove from list
	NodeList::iterator itr = std::find(mChildren.begin(), mChildren.end(), node);
	if(itr==mChildren.end()) return;

	mChildren.erase(itr);

	// add to end of list
	mChildren.push_back(node);
}

bool Node::isOnTop()
{
	NodeRef parent = getParent();
	if(parent) return parent->isOnTop( shared_from_this() );
	else return false;
}

bool Node::isOnTop(NodeRef node)
{
	if(mChildren.empty()) return false;
	if(mChildren.back() == node) return true;
	return false;
}

void Node::moveToBottom()
{
	NodeRef parent = getParent();
	if(parent) parent->moveToBottom( shared_from_this() );
}

void Node::moveToBottom(NodeRef node)
{
	// remove from list
	NodeList::iterator itr = std::find(mChildren.begin(), mChildren.end(), node);
	if(itr==mChildren.end()) return;

	mChildren.erase(itr);

	// add to start of list
	mChildren.push_front(node);
}

void Node::deepSetup()
{
	setup();

	NodeList nodes(mChildren);
	NodeList::iterator itr;
	for(itr=nodes.begin();itr!=nodes.end();++itr)
		(*itr)->deepSetup();
}

void Node::deepShutdown()
{
	NodeList nodes(mChildren);
	NodeList::reverse_iterator itr;
	for(itr=nodes.rbegin();itr!=nodes.rend();++itr)
		(*itr)->deepShutdown();

	shutdown();
}

void Node::deepTransform(const Matrix44f &world)
{
	// update transform matrix by calling derived class's function
	transform();

	// calculate world transform matrix
	mWorldTransform = world * mTransform;

	// do the same for all children
	NodeList nodes(mChildren);
	NodeList::iterator itr;
	for(itr=nodes.begin();itr!=nodes.end();++itr)
		(*itr)->deepTransform(mWorldTransform);
}

void Node::deepUpdate(double elapsed)
{
	// let derived class perform animation 
	update(elapsed);

	// update this node's children
	NodeList nodes(mChildren);
	NodeList::iterator itr;
	for(itr=nodes.begin();itr!=nodes.end();++itr)
		(*itr)->deepUpdate(elapsed);
}

void Node::deepDraw()
{
	if(!bIsVisible) return;

	// let derived class know we are about to draw stuff
	begin();

	// update transform matrix by calling derived class's function
	// (do it again here in case this node has no parent and thus wasn't updated)
	transform();

	// apply transform
	gl::pushModelView();

	// usual way to update modelview matrix
	gl::multModelView( mTransform );

	// the following call required a Cinder hack:
	//gl::setModelView( mWorldTransform );

	// draw this node by calling derived class
	draw();

	// draw this node's children
	NodeList::iterator itr;
	for(itr=mChildren.begin();itr!=mChildren.end();++itr)
		(*itr)->deepDraw();
	
	// restore transform
	gl::popModelView();

	// let derived class know we are done drawing
	end();
}

// Note: the scene graph implementation is currently not fast enough to support mouseMove events
//  when there are more than a few nodes. 
bool Node::deepMouseMove( MouseEvent event )
{
	if(!bIsVisible) return false;

	// test children first, from top to bottom
	NodeList nodes(mChildren);
	NodeList::reverse_iterator itr;
	bool handled = false;
	for(itr=nodes.rbegin();itr!=nodes.rend() && !handled;++itr)
		handled = (*itr)->deepMouseMove(event);

	// if not handled, test this node
	if(!handled) handled = mouseMove(event);

	return handled;
}//*/

bool Node::deepMouseDown( MouseEvent event )
{
	if(!bIsVisible) return false;

	// test children first, from top to bottom
	NodeList nodes(mChildren);
	NodeList::reverse_iterator itr;
	bool handled = false;
	for(itr=nodes.rbegin();itr!=nodes.rend()&&!handled;++itr)
		handled = (*itr)->deepMouseDown(event);

	// if not handled, test this node
	if(!handled) handled = mouseDown(event);

	return handled;
}

bool Node::deepMouseDrag( MouseEvent event )
{
	if(!bIsVisible) return false;

	// test children first, from top to bottom
	NodeList nodes(mChildren);
	NodeList::reverse_iterator itr;
	bool handled = false;
	for(itr=nodes.rbegin();itr!=nodes.rend()&&!handled;++itr)
		handled = (*itr)->deepMouseDrag(event);

	// if not handled, test this node
	if(!handled) handled = mouseDrag(event);

	return handled;
}

bool Node::deepMouseUp( MouseEvent event )
{
	// test children first, from top to bottom
	NodeList nodes(mChildren);
	NodeList::reverse_iterator itr;
	bool handled = false;
	for(itr=nodes.rbegin();itr!=nodes.rend()&&!handled;++itr)
		(*itr)->deepMouseUp(event); // don't care about 'handled' for now

	// if not handled, test this node
	if(!handled) handled = mouseUp(event);

	return handled;
}

bool Node::deepKeyDown( KeyEvent event )
{
	if(!bIsVisible) return false;

	// test children first, from top to bottom
	NodeList nodes(mChildren);
	NodeList::reverse_iterator itr;
	bool handled = false;
	for(itr=nodes.rbegin();itr!=nodes.rend()&&!handled;++itr)
		handled = (*itr)->deepKeyDown(event);

	// if not handled, test this node
	if(!handled) handled = keyDown(event);

	return handled;
}

bool Node::deepKeyUp( KeyEvent event )
{
	if(!bIsVisible) return false;

	// test children first, from top to bottom
	NodeList nodes(mChildren);
	NodeList::reverse_iterator itr;
	bool handled = false;
	for(itr=nodes.rbegin();itr!=nodes.rend()&&!handled;++itr)
		handled = (*itr)->deepKeyUp(event);

	// if not handled, test this node
	if(!handled) handled = keyUp(event);

	return handled;
}

bool Node::deepResize( ResizeEvent event )
{
	// test children first, from top to bottom
	NodeList nodes(mChildren);
	NodeList::reverse_iterator itr;
	bool handled = false;
	for(itr=nodes.rbegin();itr!=nodes.rend()&&!handled;++itr)
		handled = (*itr)->deepResize(event);

	// if not handled, test this node
	if(!handled) handled = resize(event);

	return handled;
}

/* transforms 'frustum coordinates' to object-space coordinates,
	where z is within range [0.0 - 1.0] from near-plane to far-plane */
Vec3f Node::unproject(const Vec3f &pt)
{
	// update viewport and projection matrix
	Matrix44f projection = gl::getProjection();
	Area viewport = gl::getViewport();

	// since we don't have GLU, we have to do the calculation ourselves:
	// -find the inverse modelview-projection-matrix
	Matrix44f a = projection * mWorldTransform;
	a.invert();
	// -transform normalized coordinates [-1, 1]
	Vec4f in;
	in.x = (pt.x - viewport.getX1())/viewport.getWidth()*2.0f-1.0f;
	in.y = (pt.y - viewport.getY1())/viewport.getHeight()*2.0f-1.0f;
	in.z = 2.0f * pt.z - 1.0f;
	in.w = 1.0f;
	// -objects coordinates
	Vec4f out = a * in;
	if(out.w != 0.0f) out.w = 1.0f / out.w;
	// -calculate output
	Vec3f result;
	result.x = out.x * out.w;
	result.y = out.y * out.w;
	result.z = out.z * out.w;
	
	return result;
}

////////////////// Node2D //////////////////


Node2D::Node2D(void)
: Node("Node2D")
{
	mPosition	= Vec2f::zero();
	mRotation	= Quatf::identity();
	mScale		= Vec2f::one();
	mAnchor		= Vec2f::zero();

	bRestoreViewport = false;
	bScissorEnabled = false;
	bRestoreScissor = false;
}

Node2D::Node2D(const string &name)
: Node(name)
{
	mPosition	= Vec2f::zero();
	mRotation	= Quatf::identity();
	mScale		= Vec2f::one();
	mAnchor		= Vec2f::zero();

	bRestoreViewport = false;
	bScissorEnabled = false;
	bRestoreScissor = false;
}

Node2D::~Node2D(void)
{
}

Vec2f Node2D::getAnchorPercentage()
{
	Vec2f size = getSize();

	if(size.length() > 0.0f)
		return mAnchor / size;
	else
		return Vec2f::zero();
}

void Node2D::setViewport(int w, int h)
{
	// Set the viewport, so nothing will be drawn outside this node.
	// Assumes node is unrotated! Functionality has not been tested through-and-through,
	// use at your own risk.

	// calculate correct screen area
	Vec2f upperLeft = objectToScreen(Vec2f(0, 0));
	Vec2f lowerRight = objectToScreen(Vec2f((float) w, (float) h));
	Area viewport = Area((int) floorf(upperLeft.x), (int) floorf(upperLeft.y), 
		(int) ceilf(lowerRight.x), (int) ceilf(lowerRight.y));	

	// store current viewport
	if(!bRestoreViewport)
	{
		bRestoreViewport = true;
		glGetIntegerv(GL_VIEWPORT, mStoredViewport);
	}

	gl::setViewport(viewport);
}

void Node2D::resetViewport()
{	
	if(!bRestoreViewport) return;

	glViewport(mStoredViewport[0], mStoredViewport[1], mStoredViewport[2], mStoredViewport[3]);
	bRestoreViewport = false;
}

void Node2D::enableScissor(float x, float y, float w, float h)
{
	// Enable scissoring, so nothing will be drawn outside this node.
	// Assumes node is unrotated! Functionality has not been tested through-and-through,
	// use at your own risk.
	
	// calculate correct screen area
	Vec2f upperLeft = objectToScreen(Vec2f(x, y));
	Vec2f lowerRight = objectToScreen(Vec2f(x+w, y+h));

	// by rounding, we prevent pixels from bleeding at the edges
	Area bounds = Area((int) ceilf(upperLeft.x), (int) ceilf(upperLeft.y), 
		(int) floorf(lowerRight.x), (int) floorf(lowerRight.y));

	// store current scissor box
	if(!bScissorEnabled)
	{
		bRestoreScissor = glIsEnabled(GL_SCISSOR_TEST);
		glGetIntegerv(GL_SCISSOR_BOX, mStoredScissor);
	}

	// enable scissoring
	Area viewport = gl::getViewport();
	glScissor(bounds.x1, viewport.y2 - bounds.y2, bounds.getWidth(), bounds.getHeight());
	glEnable(GL_SCISSOR_TEST);

	bScissorEnabled = true;
}

void Node2D::disableScissor()
{
	if(!bScissorEnabled) return;

	if(bRestoreScissor)
		glScissor(mStoredScissor[0], mStoredScissor[1], mStoredScissor[2], mStoredScissor[3]);
	else
		glDisable(GL_SCISSOR_TEST);

	bScissorEnabled = false;
}

Vec2f Node2D::screenToParent( const Vec2f &pt )
{
	Vec2f p = pt;

	Node2DRef node = getParent<Node2D>();
	if(node) p = node->screenToObject(p);

	return p;
}

Vec2f Node2D::screenToObject( const Vec2f &pt )
{
	Vec3f p(pt.x, pt.y, 0.0f);

	/* adjust y (0,0 is lowerleft corner in OpenGL)	*/	
	Area viewport = gl::getViewport();
	p.y = (viewport.getHeight() - p.y);

	/* near plane intersection */
	p.z = 0.0f;
	Vec3f p0 = unproject(p);
	/* far plane intersection */
	p.z = 1.0f;
	Vec3f p1 = unproject(p);

	/* find (x, y) coordinates */
	float t = (0.0f - p0.z) / (p1.z - p0.z);
	p.x = (p0.x + t * (p1.x - p0.x));
	p.y = (p0.y + t * (p1.y - p0.y));
	p.z = 0.0f;

	return Vec2f(p.x, p.y);
}

Vec2f Node2D::parentToScreen( const Vec2f &pt )
{
	Vec2f p = pt;

	Node2DRef node = getParent<Node2D>();
	if(node) p = node->objectToScreen(p);

	return p;
}

Vec2f Node2D::parentToObject( const Vec2f &pt )
{
	Vec3f p = mTransform.inverted().transformPointAffine( Vec3f(pt, 0.0f) );
	return Vec2f(p.x, p.y);
}

Vec2f Node2D::objectToParent( const Vec2f &pt )
{
	Vec3f p = mTransform.transformPointAffine( Vec3f(pt, 0.0f) );
	return Vec2f(p.x, p.y);
}

Vec2f Node2D::objectToScreen( const Vec2f &pt )
{
	Matrix44f	projection = gl::getProjection();
	Area		viewport = gl::getViewport();

	Matrix44f a = projection * mWorldTransform;

	Vec4f in;
	in.x = pt.x;
	in.y = pt.y;
	in.z = 0.0f;
	in.w = 1.0f;

	Vec4f out = a * in;
	if(out.w != 0.0f) out.w = 1.0f / out.w;
	out.x *= out.w;
	out.y *= out.w;
	out.z *= out.w;

	Vec2f result;
	result.x = viewport.getX1() + viewport.getWidth() * (out.x + 1.0f) / 2.0f;
	result.y = viewport.getY1() + viewport.getHeight() * (1.0f - (out.y + 1.0f) / 2.0f);
	//result.z = (out.z + 1.0f) / 2.0f;

	return result;
}

////////////////// Node3D //////////////////

Node3D::Node3D(void)
: Node("Node3D")
{
	mPosition	= Vec3f::zero();
	mRotation	= Quatf::identity();
	mScale		= Vec3f::one();
	mAnchor		= Vec3f::zero();
}

Node3D::Node3D(const string &name)
: Node(name)
{
	mPosition	= Vec3f::zero();
	mRotation	= Quatf::identity();
	mScale		= Vec3f::one();
	mAnchor		= Vec3f::zero();
}

Node3D::~Node3D(void)
{
}