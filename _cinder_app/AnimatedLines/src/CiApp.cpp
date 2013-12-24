#include "cinder/app/AppBasic.h"

// opengl
#include "cinder/Camera.h"
#include "cinder/ObjLoader.h"
#include "cinder/gl/gl.h"
#include "cinder/gl/Vbo.h"
#include "cinder/params/Params.h"

// network
#include "cinder/osc/OscListener.h"

// utils
#include "cinder/Plane.h"
#include "cinder/Perlin.h"
#include "cinder/Rand.h"
#include "../../../_common/MiniConfig.h"

using namespace ci;
using namespace ci::app;
using namespace std;

namespace
{
    Vec3f calcCentroid(const Vec3f& a, const Vec3f& b, const Vec3f& c)
    {
        return (a + b + c) / 3;
    }
}

struct MovingNode
{
    Vec3f forward;
    Vec3f up;
    Vec3f pos;

    void moveForward()
    {
        pos += randVec3f() * 0.1f;
    }

    void moveCloseToMesh(const TriMesh& triMesh)
    {
        float minDist = FLT_MAX;
        size_t minIdx = -1;
        for (size_t i=0; i<triMesh.getNumTriangles(); i+=3)
        {
            Vec3f a, b, c;
            triMesh.getTriangleVertices(i, &a, &b, &c);
            Vec3f centroid = calcCentroid(a, b, c);
            float dist = centroid.distanceSquared(pos);
            if (dist < minDist)
            {
                minDist = dist;
                minIdx = i;
            }
        }

        assert(minIdx != -1);
        Vec3f a, b, c;
        triMesh.getTriangleVertices(minIdx, &a, &b, &c);
        Planef plane(a, b, c);
        pos = plane.getClosetPoint(pos);
    }
};

MovingNode node;

struct CiApp : public AppBasic 
{
    void prepareSettings(Settings *settings)
    {
        readConfig();
        
        settings->setWindowPos(0, 0);
        settings->setWindowSize(WIN_WIDTH, WIN_HEIGHT);
    }

    void setup()
    {
        mParams = params::InterfaceGl("params", Vec2i(300, getConfigUIHeight()));
        setupConfigUI(&mParams);

        ObjLoader loader(loadAsset("WusonOBJ.obj"));
        loader.load(&mTriMesh);

        mVboMesh = gl::VboMesh(mTriMesh);

        //mClient.setup(OSC_PORT);
    }

    void shutdown()
    {
        mClient.shutdown();
    }

    void keyUp(KeyEvent event)
    {
        if (event.getCode() == KeyEvent::KEY_ESCAPE)
        {
            quit();
        }
    }

    void update()
    {
        node.moveForward();
        node.moveCloseToMesh(mTriMesh);
    }

    void draw()
    {
        CameraPersp cam(getWindowWidth(), getWindowHeight(), 60.0f);
        gl::setMatrices(cam);
        gl::clear(ColorA::black());

        gl::translate(getWindowWidth() / 2, getWindowHeight() / 2);
        gl::rotate(OBJ_ROTATION);
        gl::scale(OBJ_SCALE, OBJ_SCALE, OBJ_SCALE);
        {
            gl::color(Color::white());
            gl::enableWireframe();
            //gl::draw(mVboMesh);
            gl::disableWireframe();

            gl::color(Color(1, 0, 0));
            gl::drawSphere(node.pos, 0.03);
        }

        mParams.draw();
    }

private:
    osc::Listener       mClient;
    params::InterfaceGl mParams;
    gl::VboMesh         mVboMesh;
    TriMesh             mTriMesh;
};

CINDER_APP_BASIC(CiApp, RendererGl)
