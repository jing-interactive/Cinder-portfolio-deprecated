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

#include "is/Node.h"

using namespace ci;
using namespace ci::app;
using namespace std;
using namespace is;

int				Node::refCount = 0;
unsigned int	Node::idCount = 1;
NodeMap			Node::nodeLookupTable;

Node::Node(void)
	: mId(idCount)
{
	// default constructor for [Node]
	refCount++;
	idCount++;
	
	mTransform.setToIdentity();
	mWorldTransform.setToIdentity();

	bIsVisible = true;	
	bIsCulled = false;
	bIsClickable = true;

	bIsBoundingBoxInvalid = true;

	mBoundingBox = ci::AxisAlignedBox3f( ci::Vec3f::zero(), ci::Vec3f::zero() );
	mTreeBoundingBox = mBoundingBox;

#ifdef _DEBUG
	console() << "Node " << mId << " created. " << refCount << " in total." << std::endl;
#endif
}

Node::~Node(void)
{
	// remove all children safely
	removeChildren();

	//
	refCount--;

	// remove from lookup table
	nodeLookupTable.erase(mId);

#ifdef _DEBUG
	console() << "Node " << mId << " destroyed. " << refCount << " remaining." << std::endl;
#endif
}

void Node::setParent(NodeRef node)
{ 
	mParent = NodeWeakRef(node); 

	// invalidate the bounding box
	invalidateBoundingBox();
}

void Node::removeFromParent()
{
	NodeRef node = mParent.lock();
	if(node) node->removeChild( shared_from_this() );
}

void Node::addChild(NodeRef node)
{
	if(node && !hasChild(node))
	{
		// remove child from current parent
		NodeRef parent = node->getParent();
		if(parent) parent->removeChild(node);

		// add to children
		mChildren.push_back(node);

		// set parent
		node->setParent( shared_from_this() );

		// invalidate the bounding box
		invalidateBoundingBox();

		// store nodes in lookup table if not done yet
		nodeLookupTable[mId] = NodeWeakRef( shared_from_this() );
		nodeLookupTable[node->mId] = NodeWeakRef( node );
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

		// invalidate the bounding box
		invalidateBoundingBox();
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

	// invalidate the bounding box
	invalidateBoundingBox();
}

bool Node::hasChild(NodeRef node) const
{
	NodeList::const_iterator itr = std::find(mChildren.begin(), mChildren.end(), node);
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

AxisAlignedBox3f Node::getTreeBoundingBox()
{
	if(!bIsBoundingBoxInvalid) return mTreeBoundingBox;

	mTreeBoundingBox = mBoundingBox.transformed( mWorldTransform );

	// union with bounding boxes of children, if any
	NodeList::iterator itr;
	for(itr=mChildren.begin();itr!=mChildren.end();++itr) 
		mTreeBoundingBox.include( (*itr)->getTreeBoundingBox() );

	bIsBoundingBoxInvalid = false;

	return mTreeBoundingBox;
}

void Node::invalidateBoundingBox()
{
	if(!bIsBoundingBoxInvalid) {
		bIsBoundingBoxInvalid = true;
		
		// invalidate parent as well
		NodeRef parent = getParent();
		if(parent) parent->invalidateBoundingBox();
	}
}

void Node::treeCull( Frustumf &frustum )
{
	bool visible = frustum.intersects( getTreeBoundingBox() );
	setCulled( !visible );

	if(visible) {
		NodeList::iterator itr;
		for(itr=mChildren.begin();itr!=mChildren.end();++itr) 
			(*itr)->treeCull(frustum);
	}
}

void Node::treeDrawBoundingBox()
{
	gl::drawStrokedCube( getTreeBoundingBox() );

	NodeList::iterator itr;
	for(itr=mChildren.begin();itr!=mChildren.end();++itr) 
		(*itr)->treeDrawBoundingBox();
}

void Node::treeSetup()
{
	setup();

	NodeList nodes(mChildren);
	NodeList::iterator itr;
	for(itr=nodes.begin();itr!=nodes.end();++itr)
		(*itr)->treeSetup();
}

void Node::treeShutdown()
{
	NodeList nodes(mChildren);
	NodeList::reverse_iterator itr;
	for(itr=nodes.rbegin();itr!=nodes.rend();++itr)
		(*itr)->treeShutdown();

	shutdown();
}

void Node::treeUpdate(double elapsed)
{
	// let derived class perform animation 
	update(elapsed);

	// update transform matrix by calling derived class's function
	transform();

	// update this node's children
	NodeList nodes(mChildren);
	NodeList::iterator itr;
	for(itr=nodes.begin();itr!=nodes.end();++itr)
		(*itr)->treeUpdate(elapsed);
}

void Node::treeDraw()
{
	if(!bIsVisible || bIsCulled) 
		return;

	// let derived class know we are about to draw stuff
	begin();

	// apply transform
	gl::pushModelView();

	// usual way to update modelview matrix
	gl::multModelView( mTransform );

	// draw this node by calling derived class
	draw();

	// draw this node's children
	NodeList::iterator itr;
	for(itr=mChildren.begin();itr!=mChildren.end();++itr)
		(*itr)->treeDraw();
	
	// restore transform
	gl::popModelView();

	// let derived class know we are done drawing
	end();
}
 
bool Node::treeMouseMove( MouseEvent event )
{
	/*// issue compiler warning if used
	#if _MSC_VER
        #pragma message ("Warning : Calling Node::treeMouseMove() is very slow on trees containing more than a few nodes.")
    #elif   __GNUC__
        #warning ("Warning : Calling Node::treeMouseMove() is very slow on trees containing more than a few nodes.")
    #endif
	//*/

	// don't process event if not visible or if culled
	if(!bIsVisible || bIsCulled) return false;

	// test children first, from top to bottom
	NodeList nodes(mChildren);
	NodeList::reverse_iterator itr;
	bool handled = false;
	for(itr=nodes.rbegin();itr!=nodes.rend() && !handled;++itr)
		handled = (*itr)->treeMouseMove(event);

	// if not handled, test this node
	if(!handled) handled = mouseMove(event);

	return handled;
}

bool Node::treeMouseDown( MouseEvent event )
{
	// don't process event if not visible or if culled
	if(!bIsVisible || bIsCulled) return false;

	// test children first, from top to bottom
	NodeList nodes(mChildren);
	NodeList::reverse_iterator itr;
	bool handled = false;
	for(itr=nodes.rbegin();itr!=nodes.rend()&&!handled;++itr)
		handled = (*itr)->treeMouseDown(event);

	// if not handled, test this node
	if(!handled) handled = mouseDown(event);

	return handled;
}

bool Node::treeMouseDrag( MouseEvent event )
{
	// don't process event if not visible or if culled
	if(!bIsVisible || bIsCulled) return false;

	// test children first, from top to bottom
	NodeList nodes(mChildren);
	NodeList::reverse_iterator itr;
	bool handled = false;
	for(itr=nodes.rbegin();itr!=nodes.rend()&&!handled;++itr)
		handled = (*itr)->treeMouseDrag(event);

	// if not handled, test this node
	if(!handled) handled = mouseDrag(event);

	return handled;
}

bool Node::treeMouseUp( MouseEvent event )
{
	// don't process event if not visible or if culled
	if(!bIsVisible || bIsCulled) return false;

	// test children first, from top to bottom
	NodeList nodes(mChildren);
	NodeList::reverse_iterator itr;
	bool handled = false;
	for(itr=nodes.rbegin();itr!=nodes.rend()&&!handled;++itr)
		(*itr)->treeMouseUp(event); // don't care about 'handled' for now

	// if not handled, test this node
	if(!handled) handled = mouseUp(event);

	return handled;
}

bool Node::treeKeyDown( KeyEvent event )
{
	// don't process event if not visible or if culled
	if(!bIsVisible || bIsCulled) return false;

	// test children first, from top to bottom
	NodeList nodes(mChildren);
	NodeList::reverse_iterator itr;
	bool handled = false;
	for(itr=nodes.rbegin();itr!=nodes.rend()&&!handled;++itr)
		handled = (*itr)->treeKeyDown(event);

	// if not handled, test this node
	if(!handled) handled = keyDown(event);

	return handled;
}

bool Node::treeKeyUp( KeyEvent event )
{
	// don't process event if not visible or if culled
	if(!bIsVisible || bIsCulled) return false;

	// test children first, from top to bottom
	NodeList nodes(mChildren);
	NodeList::reverse_iterator itr;
	bool handled = false;
	for(itr=nodes.rbegin();itr!=nodes.rend()&&!handled;++itr)
		handled = (*itr)->treeKeyUp(event);

	// if not handled, test this node
	if(!handled) handled = keyUp(event);

	return handled;
}

bool Node::treeResize( ResizeEvent event )
{
	// test children first, from top to bottom
	NodeList nodes(mChildren);
	NodeList::reverse_iterator itr;
	bool handled = false;
	for(itr=nodes.rbegin();itr!=nodes.rend()&&!handled;++itr)
		handled = (*itr)->treeResize(event);

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

////////////////// NodeGL //////////////////

void NodeGL::loadShader(const std::string &vert, const std::string &frag)
{
	try {
		mShader = gl::GlslProg( loadFile(vert), loadFile(frag) );
	}
	catch(const std::exception &e) {
		app::console() << "Could not load&compile shader (" << vert << "):" << e.what() << std::endl;
	}
}

////////////////// Node2D //////////////////


Node2D::Node2D(void)
{
	mPosition	= Vec2f::zero();
	mRotation	= Quatf::identity();
	mScale		= Vec2f::one();
	mAnchor		= Vec2f::zero();

	bIsAnchorPercentage = false;

	bRestoreViewport = false;
	bScissorEnabled = false;
	bRestoreScissor = false;
}

Node2D::~Node2D(void)
{
}

Vec2f Node2D::getAnchorPercentage() const
{
	Vec2f size = getSize();

	if(size.length() > 0.0f)
		return mAnchor / size;
	else
		return Vec2f::zero();
}

void Node2D::setSize(float w, float h)
{ 
	if(bIsAnchorPercentage) {
		ci::Vec2f p = getAnchorPercentage();
		mAnchor = ci::Vec2f(w * p.x, h * p.y); 
	}

	mWidth = w; 
	mHeight = h;  
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
{
	mPosition	= Vec3f::zero();
	mRotation	= Quatf::identity();
	mScale		= Vec3f::one();
	mAnchor		= Vec3f::zero();

	//mBoundingBox = AxisAlignedBox3f( ci::Vec3f::zero(), ci::Vec3f::zero() );
}

Node3D::~Node3D(void)
{
}

void Node3D::treeDrawWireframe()
{
	if(!isVisible()) return;

	// apply transform
	gl::pushModelView();

	// usual way to update modelview matrix
	gl::multModelView( mTransform );

	// draw this node by calling derived class
	drawWireframe();

	// draw this node's children
	NodeList::iterator itr;
	for(itr=mChildren.begin();itr!=mChildren.end();++itr) {
		// only call other Node3D's
		Node3DRef node = boost::shared_dynamic_cast<Node3D>(*itr);
		if(node) node->treeDrawWireframe();
	}
	
	// restore transform
	gl::popModelView();
}