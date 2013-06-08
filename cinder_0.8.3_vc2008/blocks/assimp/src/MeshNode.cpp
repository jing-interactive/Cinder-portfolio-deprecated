/*
 Copyright (C) 2011-2012 Gabor Papp

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU Lesser General Public License as published
 by the Free Software Foundation; either version 3 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU Lesser General Public License for more details.

 You should have received a copy of the GNU Lesser General Public License
 along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

#include "MeshNode.h"

using namespace std;

namespace cinder { namespace assimp {

MeshNode::MeshNode() :
	mScale( Vec3f::one() ),
	mInheritOrientation( true ),
	mInheritScale( true ),
	mNeedsUpdate( true )
{
}

MeshNode::MeshNode( const std::string& name ) :
	mName( name ),
	mScale( Vec3f::one() ),
	mInheritOrientation( true ),
	mInheritScale( true ),
	mNeedsUpdate( true )
{
}

void MeshNode::setParent( MeshNodeRef parent )
{
	mParent = parent;
	requestUpdate();
}

MeshNodeRef MeshNode::getParent() const
{
	return mParent;
}

void MeshNode::addChild( MeshNodeRef child )
{
	mChildren.push_back( child );
}

void MeshNode::setOrientation( const Quatf& q )
{
	mOrientation = q;
	mOrientation.normalize();
	requestUpdate();
}

const Quatf& MeshNode::getOrientation() const
{
	return mOrientation;
}

void MeshNode::setPosition( const Vec3f& pos )
{
	mPosition = pos;
	requestUpdate();
}

const Vec3f& MeshNode::getPosition() const
{
	return mPosition;
}

void MeshNode::setScale( const Vec3f& scale )
{
	mScale = scale;
	requestUpdate();
}

const Vec3f& MeshNode::getScale() const
{
	return mScale;
}

void MeshNode::setInheritOrientation( bool inherit )
{
	mInheritOrientation = inherit;
	requestUpdate();
}

bool MeshNode::getInheritOrientation() const
{
	return mInheritOrientation;
}

void MeshNode::setInheritScale( bool inherit )
{
	mInheritScale = inherit;
	requestUpdate();
}

bool MeshNode::getInheritScale() const
{
	return mInheritScale;
}

void MeshNode::setName( const string& name )
{
	mName = name;
}

const string& MeshNode::getName() const
{
	return mName;
}

void MeshNode::setInitialState()
{
	mInitialPosition = mPosition;
	mInitialOrientation = mOrientation;
	mInitialScale = mScale;
}

void MeshNode::resetToInitialState()
{
	mPosition = mInitialPosition;
	mOrientation = mInitialOrientation;
	mScale = mInitialScale;
	requestUpdate();
}

const Vec3f& MeshNode::getInitialPosition() const
{
	return mInitialPosition;
}

const Quatf& MeshNode::getInitialOrientation() const
{
	return mInitialOrientation;
}

const Vec3f& MeshNode::getInitialScale() const
{
	return mInitialScale;
}

const Quatf& MeshNode::getDerivedOrientation() const
{
	if ( mNeedsUpdate )
		update();
	return mDerivedOrientation;
}

const Vec3f& MeshNode::getDerivedPosition() const
{
	if ( mNeedsUpdate )
		update();
	return mDerivedPosition;
}

const Vec3f& MeshNode::getDerivedScale() const
{
	if ( mNeedsUpdate )
		update();
	return mDerivedScale;
}

const Matrix44f& MeshNode::getDerivedTransform() const
{
	if ( mNeedsUpdate )
		update();

    mDerivedTransform = Matrix44f::createScale( mDerivedScale );
    mDerivedTransform *= mDerivedOrientation.toMatrix44();
    mDerivedTransform.setTranslate( mDerivedPosition );

    return mDerivedTransform;
}

void MeshNode::update() const
{
    if ( mParent )
    {
        // update orientation
        const Quatf& parentOrientation = mParent->getDerivedOrientation();
        if ( mInheritOrientation )
        {
            // Combine orientation with that of parent
            mDerivedOrientation = getOrientation() * parentOrientation;
        }
        else
        {
            mDerivedOrientation = getOrientation();
        }

        // update scale
        const Vec3f& parentScale = mParent->getDerivedScale();
        if ( mInheritScale )
        {
            mDerivedScale = parentScale * getScale();
        }
        else
        {
            mDerivedScale = getScale();
        }

		// change position vector based on parent's orientation&  scale
        mDerivedPosition = ( parentScale * getPosition() ) * parentOrientation;

        // add altered position vector to parent's
        mDerivedPosition += mParent->getDerivedPosition();
    }
    else
    {
        // root node, no parent
        mDerivedOrientation = getOrientation();
        mDerivedPosition = getPosition();
        mDerivedScale = getScale();
    }

	mNeedsUpdate = false;
}

void MeshNode::requestUpdate()
{
	mNeedsUpdate = true;

	for ( vector< MeshNodeRef >::iterator it = mChildren.begin();
			it != mChildren.end(); ++it )
	{
		(*it)->requestUpdate();
	}
}

} } // namespace cinder::assimp
