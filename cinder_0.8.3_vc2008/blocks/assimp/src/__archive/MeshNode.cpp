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

MeshNode::MeshNode( const std::string& name ) :
mName( name )
{
}

void MeshNode::setName( const string& name )
{
    mName = name;
}

const string& MeshNode::getName() const
{
    return mName;
}
//
//void MeshNode::transform() const
//{
//    MeshNodeRef parent = mParent.lock();
//    if ( parent )
//    {
//        // update Rotation
//        const Quatf& parentRotation = parent->getRotation();
//        mRotation = getRotation() * parentRotation;
//        // update scale
//        const Vec3f& parentScale = parent->getDerivedScale();
//        mDerivedScale = parentScale * getScale();
//
//        // change position vector based on parent's Rotation&  scale
//        mDerivedPosition = ( parentScale * getPosition() ) * parentRotation;
//
//        // add altered position vector to parent's
//        mDerivedPosition += parent->getDerivedPosition();
//    }
//    else
//    {
//        // root node, no parent
//        mDerivedRotation = getRotation();
//        mDerivedPosition = getPosition();
//        mDerivedScale = getScale();
//    }
//
//    mNeedsUpdate = false;
//}

} } // namespace cinder::assimp
