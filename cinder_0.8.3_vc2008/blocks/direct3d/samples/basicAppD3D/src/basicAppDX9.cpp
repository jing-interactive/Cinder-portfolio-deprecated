#include "cinder/app/AppBasic.h"
#include "cinder/Camera.h"
#include "RendererDx9.h"
#include "dx9.h"
#include <boost/intrusive_ptr.hpp>

using namespace ci;
using namespace ci::app; 

using namespace std;

HRESULT hr = S_OK;

class BasicApp : public AppBasic {
public:
    void setup()
    {
        dev = dx9::getDevice();

        ID3DXMesh* tempMesh = NULL;
        V(D3DXCreateTeapot(dev, &tempMesh, NULL));
        teapot = boost::intrusive_ptr<ID3DXMesh>(tempMesh, false);
    }

    void keyDown( KeyEvent event )
    {
        if( event.getChar() == 'f' )
            setFullScreen( ! isFullScreen() );
        if (event.getCode() == KeyEvent::KEY_ESCAPE)
            quit();
    }

    void draw()
    {
        dx9::clear(ColorA(0.5f, 0.5f, 0.5f));
        V(teapot->DrawSubset(0));
    }

    void resize( ResizeEvent event )
    {
        mCam.lookAt( Vec3f( 0.0f, 0.0f, 10.0f ), Vec3f::zero() );
        mCam.setPerspective( 60, getWindowAspectRatio(), 1, 5000 );
        dx9::setMatrices( mCam);
    }

private: 
    CameraPersp	mCam;

    boost::intrusive_ptr<ID3DXMesh> teapot;
    IDirect3DDevice9* dev;
};

CINDER_APP_BASIC( BasicApp, RendererDX9)