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

#include "SceneGraph/Node3D.h"

namespace cinder { namespace assimp {

typedef std::shared_ptr< class MeshNode > MeshNodeRef;
typedef std::weak_ptr< class MeshNode > MeshNodeWeakRef;

class MeshNode : public ph::nodes::Node3D
{
public:
    MeshNode(){}
    MeshNode( const std::string& name );

    std::vector< std::shared_ptr< class Mesh > > mMeshes;

    void setName( const std::string& name );
    const std::string& getName() const;

protected:
    /// Name of this node.
    std::string mName;
};

} }

