#include "Node3D.h"

using namespace ci;

namespace ph { namespace nodes {

Node3D::Node3D(void)
{
	mPosition	= Vec3f::zero();
	mRotation	= Quatf::identity();
	mScale		= Vec3f::one();
	mAnchor		= Vec3f::zero();
}

Node3D::~Node3D(void)
{
}

void Node3D::treeDrawWireframe()
{
	if(!mIsVisible) return;

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
		Node3DRef node = std::dynamic_pointer_cast<Node3D>(*itr);
		if(node) node->treeDrawWireframe();
	}
	
	// restore transform
	gl::popModelView();
}

} }