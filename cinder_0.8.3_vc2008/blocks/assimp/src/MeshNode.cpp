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
	mInheritRotation( true ),
	mInheritScale( true ),
	mNeedsUpdate( true )
{
}

MeshNode::MeshNode( const std::string& name ) :
	mName( name ),
	mScale( Vec3f::one() ),
	mInheritRotation( true ),
	mInheritScale( true ),
	mNeedsUpdate( true )
{
}

void MeshNode::setParent( MeshNodeRef parent )
{
	mParent = MeshNodeWeakRef(parent);
	requestUpdate();
}

MeshNodeRef MeshNode::getParent() const
{
	return mParent.lock();
}

void MeshNode::addChild( MeshNodeRef child )
{
	mChildren.push_back( child );
}

void MeshNode::setRotation( const Quatf& q )
{
	mRotation = q;
	mRotation.normalize();
	requestUpdate();
}

const Quatf& MeshNode::getRotation() const
{
	return mRotation;
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

void MeshNode::setInheritRotation( bool inherit )
{
	mInheritRotation = inherit;
	requestUpdate();
}

bool MeshNode::getInheritRotation() const
{
	return mInheritRotation;
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
	mInitialRotation = mRotation;
	mInitialScale = mScale;
}

void MeshNode::resetToInitialState()
{
	mPosition = mInitialPosition;
	mRotation = mInitialRotation;
	mScale = mInitialScale;
	requestUpdate();
}

const Vec3f& MeshNode::getInitialPosition() const
{
	return mInitialPosition;
}

const Quatf& MeshNode::getInitialRotation() const
{
	return mInitialRotation;
}

const Vec3f& MeshNode::getInitialScale() const
{
	return mInitialScale;
}

const Quatf& MeshNode::getWorldRotation() const
{
	if ( mNeedsUpdate )
		update();
	return mWorldRotation;
}

const Vec3f& MeshNode::getWorldPosition() const
{
	if ( mNeedsUpdate )
		update();
	return mWorldPosition;
}

const Vec3f& MeshNode::getWorldScale() const
{
	if ( mNeedsUpdate )
		update();
	return mWorldScale;
}

const Matrix44f& MeshNode::getWorldTransform() const
{
	if ( mNeedsUpdate )
		update();

    mWorldTransform = Matrix44f::createScale( mWorldScale );
    mWorldTransform *= mWorldRotation.toMatrix44();
    mWorldTransform.setTranslate( mWorldPosition );

    return mWorldTransform;
}

void MeshNode::update() const
{
    MeshNodeRef parent = mParent.lock();
    if ( parent )
    {
        // update Rotation
        const Quatf& parentRotation = parent->getWorldRotation();
        if ( mInheritRotation )
        {
            // Combine Rotation with that of parent
            mWorldRotation = getRotation() * parentRotation;
        }
        else
        {
            mWorldRotation = getRotation();
        }

        // update scale
        const Vec3f& parentScale = parent->getWorldScale();
        if ( mInheritScale )
        {
            mWorldScale = parentScale * getScale();
        }
        else
        {
            mWorldScale = getScale();
        }

		// change position vector based on parent's Rotation&  scale
        mWorldPosition = ( parentScale * getPosition() ) * parentRotation;

        // add altered position vector to parent's
        mWorldPosition += parent->getWorldPosition();
    }
    else
    {
        // root node, no parent
        mWorldRotation = getRotation();
        mWorldPosition = getPosition();
        mWorldScale = getScale();
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
