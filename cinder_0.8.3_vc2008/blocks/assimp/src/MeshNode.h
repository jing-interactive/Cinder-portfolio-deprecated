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

#pragma once

#include <string>
#include <vector>

#include "cinder/Cinder.h"
#include "cinder/Vector.h"
#include "cinder/Quaternion.h"
#include "cinder/Matrix.h"

namespace cinder { namespace assimp {

typedef std::shared_ptr< class MeshNode > MeshNodeRef;

class MeshNode
{
public:
    MeshNode();
    MeshNode( const std::string& name );
    virtual ~MeshNode() {};

    std::vector< std::shared_ptr< class Mesh > > mMeshes;

    void setParent( MeshNodeRef parent );
    MeshNodeRef getParent() const;

    void addChild( MeshNodeRef child );

    void setOrientation( const Quatf& q );
    const Quatf& getOrientation() const;

    void setPosition( const Vec3f& pos );
    const Vec3f& getPosition() const;

    void setScale( const Vec3f& scale );
    const Vec3f& getScale() const;

    void setInheritOrientation( bool inherit );
    bool getInheritOrientation() const;

    void setInheritScale( bool inherit );
    bool getInheritScale() const;

    void setName( const std::string& name );
    const std::string& getName() const;

    void setInitialState();
    void resetToInitialState();
    const Vec3f& getInitialPosition() const;
    const Quatf& getInitialOrientation() const;
    const Vec3f& getInitialScale() const;

    const Quatf& getDerivedOrientation() const;
    const Vec3f& getDerivedPosition() const;
    const Vec3f& getDerivedScale() const;

    const Matrix44f& getDerivedTransform() const;

    void requestUpdate();

protected:
    /// Shared pointer to parent node.
    MeshNodeRef mParent;

    /// Shared pointer vector holding the children.
    std::vector< MeshNodeRef > mChildren;

    /// Name of this node.
    std::string mName;

    /// The orientation of the node relative to its parent.
    Quatf mOrientation;

    /// The position of the node relative to its parent.
    Vec3f mPosition;

    /// Scaling factor applied to this node.
    Vec3f mScale;

    /// Stores whether this node inherits orientation from its parent.
    bool mInheritOrientation;

    mutable bool mNeedsUpdate;

    /// Stores whether this node inherits scale from its parent
    bool mInheritScale;

    /** Cached combined orientation.
    This member is the orientation derived by combining the
    local transformations and those of it's parents.
    */
    mutable Quatf mDerivedOrientation;

    /** Cached combined position.
    This member is the position derived by combining the
    local transformations and those of it's parents.
    */
    mutable Vec3f mDerivedPosition;

    /** Cached combined scale.
    This member is the position derived by combining the
    local transformations and those of it's parents.
    */
    mutable Vec3f mDerivedScale;

    /// The position to use as a base for keyframe animation.
    Vec3f mInitialPosition;
    /// The orientation to use as a base for keyframe animation.
    Quatf mInitialOrientation;
    /// The scale to use as a base for keyframe animation.
    Vec3f mInitialScale;

    /// Cached derived transform as a 4x4 matrix
    mutable Matrix44f mDerivedTransform;

    void update() const;
};

} }

