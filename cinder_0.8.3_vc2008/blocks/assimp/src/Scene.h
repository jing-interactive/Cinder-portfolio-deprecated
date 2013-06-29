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

#include <vector>

#include "../assimp--3.0.1270/include/assimp/types.h"

#include "cinder/Cinder.h"
#include "cinder/Color.h"
#include "cinder/AxisAlignedBox.h"

#include "MeshNode.h"

// forward declaration
struct aiNode;
struct aiMesh;
struct aiScene;
namespace Assimp
{
    class Importer;
} // namespace Assimp

namespace cinder 
{
    class TriMesh;
    namespace gl 
    {
        class Material; 
        class Texture;
    }
} // namespace cinder

namespace cinder { namespace assimp {

inline Vec3f fromAssimp( const aiVector3D &v )
{
    return Vec3f( v.x, v.y, v.z );
}

inline aiVector3D toAssimp( const Vec3f &v )
{
    return aiVector3D( v.x, v.y, v.z );
}

inline Quatf fromAssimp( const aiQuaternion &q )
{
    return Quatf( q.w, q.x, q.y, q.z );
}

inline Matrix44f fromAssimp( const aiMatrix4x4 &m )
{
	return Matrix44f( &m.a1, true );
}

inline aiMatrix4x4 toAssimp( const Matrix44f &m )
{
	return aiMatrix4x4( m.m00, m.m01, m.m02, m.m03,
						m.m10, m.m11, m.m12, m.m13,
						m.m20, m.m21, m.m22, m.m23,
						m.m30, m.m31, m.m32, m.m33 );
}

inline aiQuaternion toAssimp( const Quatf &q )
{
    return aiQuaternion( q.w, q.v.x, q.v.y, q.v.z );
}

inline ColorAf fromAssimp( const aiColor4D &c )
{
    return ColorAf( c.r, c.g, c.b, c.a );
}

inline aiColor4D toAssimp( const ColorAf &c )
{
    return aiColor4D( c.r, c.g, c.b, c.a );
}

inline std::string fromAssimp( const aiString &s )
{
	return std::string( s.C_Str() );
}

class AssimpExc : public std::exception
{
public:
    AssimpExc(const std::string& msg):mMsg(msg){}
    virtual const char * what()
    {
        return mMsg.c_str();
    }

private:
    std::string mMsg;

};

class Mesh;
typedef std::shared_ptr< Mesh > MeshRef;

class Scene
{
public:
    Scene() {}

    //! Constructs and does the parsing of the file from \a filename.
    Scene( fs::path filename );

    //! Updates model animation and skinning.
    void update();
    //! Draws all meshes in the model.
    void draw();

    //! Returns the bounding box of the static, not skinned mesh.
    AxisAlignedBox3f getBoundingBox() const { return mBoundingBox; }

    //! Sets the orientation of this node via a quaternion.
    void setNodeOrientation( const std::string &name, const Quatf &rot );
    //! Returns a quaternion representing the orientation of the node called \a name.
    Quatf getNodeOrientation( const std::string &name );

    //! Returns the node called \a name.
    MeshNodeRef getAssimpNode( const std::string &name );
    //! Returns the node called \a name.
    const MeshNodeRef getAssimpNode( const std::string &name ) const;

    //! Returns the total number of meshes contained by the node called \a name.
    size_t getAssimpNodeNumMeshes( const std::string &name );
    //! Returns the \a n'th cinder::TriMesh contained by the node called \a name.
    TriMesh &getAssimpNodeMesh( const std::string &name, size_t n = 0 );
    //! Returns the \a n'th cinder::TriMesh contained by the node called \a name.
    const TriMesh &getAssimpNodeMesh( const std::string &name, size_t n = 0 ) const;

    //! Returns the texture of the \a n'th mesh in the node called \a name.
    gl::Texture &getAssimpNodeTexture( const std::string &name, size_t n = 0 );
    //! Returns the texture of the \a n'th mesh in the node called \a name.
    const gl::Texture &getAssimpNodeTexture( const std::string &name, size_t n = 0 ) const;

    //! Returns the material of the \a n'th mesh in the node called \a name.
    gl::Material &getAssimpNodeMaterial( const std::string &name, size_t n = 0 );
    //! Returns the material of the \a n'th mesh in the node called \a name.
    const gl::Material &getAssimpNodeMaterial( const std::string &name, size_t n = 0 ) const;

    //! Returns all node names in the model in a std::vector as std::string's.
    const std::vector< std::string > &getNodeNames() { return mNodeNames; }

    //! Enables/disables the usage of materials during draw.
    void enableMaterials( bool enable = true ) { mMaterialsEnabled = enable; }
    //! Disables the usage of materials during draw.
    void disableMaterials() { mMaterialsEnabled = false; }

    //! Enables/disables the usage of textures during draw.
    void enableTextures( bool enable = true ) { mTexturesEnabled = enable; }
    //! Disables the usage of textures during draw.
    void disableTextures() { mTexturesEnabled = false; }

    //! Enables/disables skinning, when the model's bones distort the vertices.
    void enableSkinning( bool enable = true );
    //! Disables skinning, when the model's bones distort the vertices.
    void disableSkinning() { enableSkinning( false ); }

    //! Enables/disables animation.
    void enableAnimation( bool enable = true ) { mAnimationEnabled = enable; }
    //! Disables animation.
    void disableAnimation() { mAnimationEnabled = false; }

    //! Returns the total number of meshes in the model.
    size_t getNumMeshes() const { return mMeshes.size(); }
    //! Returns the \a n'th mesh in the model.
    TriMesh &getMesh( size_t n );
    //! Returns the \a n'th mesh in the model.
    const TriMesh &getMesh( size_t n ) const;

    //! Returns the texture of the \a n'th mesh in the model.
    gl::Texture &getTexture( size_t n );
    //! Returns the texture of the \a n'th mesh in the model.
    const gl::Texture &getTexture( size_t n ) const;

    //! Returns the number of animations in the scene.
    size_t getNumAnimations() const;

    //! Sets the current animation index to \a n.
    void setAnimation( size_t n );

    //! Returns the duration of the \a n'th animation.
    double getAnimationDuration( size_t n ) const;

    //! Sets current animation time.
    void setTime( double t );

private:
    void loadAllMeshes();
    MeshNodeRef loadNodes( const aiNode* nd, MeshNodeRef parentRef = MeshNodeRef() );
    MeshRef convertAiMesh( const aiMesh *mesh );

    void calculateDimensions();
    void calculateBoundingBox( Vec3f *min, Vec3f *max );
    void calculateBoundingBoxForNode( const aiNode *nd, aiVector3D *min, aiVector3D *max, aiMatrix4x4 *trafo );

    void updateAnimation( size_t animationIndex, double currentTime );
    void updateSkinning();
    void updateMeshes();

    std::shared_ptr< Assimp::Importer > mImporterRef; // mScene will be destroyed along with the Importer object
    fs::path mFilePath; /// model path
    const aiScene *mScene;

    AxisAlignedBox3f mBoundingBox;

    MeshNodeRef mRootNode; /// root node of scene

    std::vector< MeshNodeRef > mNodes; /// nodes with meshes
    std::vector< MeshRef > mMeshes; /// all meshes

    std::vector< std::string > mNodeNames;
    std::map< std::string, MeshNodeRef > mNodeMap;

    bool mMaterialsEnabled;
    bool mTexturesEnabled;
    bool mSkinningEnabled;
    bool mAnimationEnabled;

    size_t mAnimationIndex;
    double mAnimationTime;
};

} } // namespace mndl::assimp

