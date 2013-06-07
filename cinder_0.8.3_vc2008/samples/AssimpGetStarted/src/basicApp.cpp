#include "cinder/app/AppBasic.h"
#include "cinder/Rand.h"
#include "assimp/Importer.hpp"
#include "assimp/scene.h"
#include "assimp/postprocess.h"
#include "cinder/gl/Vbo.h"

using namespace ci;
using namespace ci::app;

#include <list>
using namespace std;

namespace
{
    inline ci::Vec3f fromAssimp( const aiVector3D &v )
    {
        return ci::Vec3f( v.x, v.y, v.z );
    }

    inline aiVector3D toAssimp( const ci::Vec3f &v )
    {
        return aiVector3D( v.x, v.y, v.z );
    }

    inline ci::Quatf fromAssimp( const aiQuaternion &q )
    {
        return ci::Quatf( q.w, q.x, q.y, q.z );
    }

    inline aiQuaternion toAssimp( const ci::Quatf &q )
    {
        return aiQuaternion( q.w, q.v.x, q.v.y, q.v.z );
    }

    inline ci::Matrix44f fromAssimp( const aiMatrix4x4 &m )
    {
        return ci::Matrix44f( &m.a1, true );
    }

    inline aiMatrix4x4 toAssimp( const ci::Matrix44f &m )
    {
        return aiMatrix4x4( m.m00, m.m01, m.m02, m.m03,
            m.m10, m.m11, m.m12, m.m13,
            m.m20, m.m21, m.m22, m.m23,
            m.m30, m.m31, m.m32, m.m33 );
    }

    inline ci::ColorAf fromAssimp( const aiColor4D &c )
    {
        return ci::ColorAf( c.r, c.g, c.b, c.a );
    }

    inline aiColor4D toAssimp( const ci::ColorAf &c )
    {
        return aiColor4D( c.r, c.g, c.b, c.a );
    }

    inline std::string fromAssimp( const aiString &s )
    {
        return std::string( s.C_Str() );
    }

    class assimpException : public ci::Exception 
    {
    };

    void loadTriMeshFromAssimp(ci::TriMesh& triMesh, const Assimp::Importer& importer)
    {
        const aiScene* pScene = importer.GetScene();
        if (pScene == NULL)
            throw assimpException();

        uint32_t nMeshes = pScene->mNumMeshes;
        for (uint32_t i=0;i<nMeshes;i++)
        {
            aiMesh* pMesh = pScene->mMeshes[i];
            aiString name = pMesh->mName;
        }
    }

}

void staticMouseDownHandler( MouseEvent event );
void staticMouseUpHandler( MouseEvent event );

// We'll create a new Cinder Application by deriving from the BasicApp class
class BasicApp : public AppBasic {
public:
    void setup()
    {
        mImporter.ReadFile("e:\\__svn_pool\\assimp\\test\\models\\Collada\\duck.dae", aiProcessPreset_TargetRealtime_Fast);
        loadTriMeshFromAssimp(mMesh, mImporter);
    }

    // Cinder will always call this function whenever the user drags the mouse
    void mouseDrag( MouseEvent event );
    void keyDown( KeyEvent event );
    // Cinder calls this function 30 times per second by default
    void draw();

    // This will maintain a list of points which we will draw line segments between
    list<Vec2f>		mPoints;
    Assimp::Importer    mImporter;
    TriMesh         mMesh;
};

void BasicApp::mouseDrag( MouseEvent event )
{
    // add wherever the user drags to the end of our list of points
    mPoints.push_back( event.getPos() );
}

void BasicApp::keyDown( KeyEvent event )
{
    if( event.getChar() == 'f' )
        setFullScreen( ! isFullScreen() );
}

void BasicApp::draw()
{
    gl::setMatricesWindow( getWindowSize() );
    // this pair of lines is the standard way to clear the screen in OpenGL
    gl::clear( Color( 0.1f, 0.1f, 0.1f ) );

    // We'll set the color to an orange color
    glColor3f( 1.0f, 0.5f, 0.25f );

    // now tell OpenGL we've got a series of points it should draw lines between
    glBegin( GL_LINE_STRIP );
    // iterate across our list of points, and pass each one to OpenGL
    for( list<Vec2f>::iterator pointIter = mPoints.begin(); pointIter != mPoints.end(); ++pointIter ) {
        glVertex2f( *pointIter );
    }
    // tell OpenGL to actually draw the lines now
    glEnd();
}

// This line tells Flint to actually create the application
CINDER_APP_BASIC( BasicApp, RendererGl )