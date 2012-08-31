#include "dx9/dx9.h"
#include "dx9/ManagedObject.h"

#pragma comment( lib, "dxerr.lib" )
#pragma comment( lib, "dxguid.lib" )
#pragma comment( lib, "d3d9.lib" )
#ifdef _DEBUG
#pragma comment( lib, "d3dx9d.lib" )
#else
#pragma comment( lib, "d3dx9.lib" )
#endif

#include "cinder/CinderMath.h"
#include "cinder/Vector.h"
#include "cinder/Camera.h"
#include "cinder/TriMesh.h"
#include "cinder/Sphere.h"
//#include "dx9/Texture.h"
#include "cinder/Text.h"
#include "cinder/PolyLine.h"
#include "cinder/Path2d.h"
#include "cinder/Shape2d.h"
#include "cinder/Triangulate.h"

namespace{
HRESULT hr = S_OK;
}

namespace cinder { namespace dx9 {

IDirect3DDevice9* g_device = NULL;

IDirect3DDevice9* getDevice()
{
    return g_device;
}

void clear( const ColorA &color, bool clearDepthBuffer, float clearZ)
{
    V(g_device->Clear(0, NULL, 
        clearDepthBuffer ? ( D3DCLEAR_TARGET | D3DCLEAR_ZBUFFER ) : D3DCLEAR_TARGET,
        D3DCOLOR_COLORVALUE(color.r, color.g, color.b, color.a), clearZ, 0));
}

void setModelView( const Camera &cam )
{
    g_device->SetTransform(D3DTS_VIEW, (D3DMATRIX*)cam.getModelViewMatrix().m);
}

void setProjection( const Camera &cam )
{
    g_device->SetTransform(D3DTS_PROJECTION, (D3DMATRIX*)cam.getProjectionMatrix().m);
}

void setMatrices( const Camera &cam )
{
	setProjection( cam );
	setModelView( cam );
}

void enableWireframe()
{
    g_device->SetRenderState(D3DRS_FILLMODE, D3DFILL_SOLID);
}

void disableWireframe()
{
    g_device->SetRenderState(D3DRS_FILLMODE, D3DFILL_WIREFRAME);
}

void disableDepthRead()
{
	g_device->SetRenderState(D3DRS_ZENABLE, D3DZB_TRUE);
}

void enableDepthRead( bool enable )
{
	if( enable )
		g_device->SetRenderState(D3DRS_ZENABLE, D3DZB_TRUE);
	else
		g_device->SetRenderState(D3DRS_ZENABLE, D3DZB_FALSE);
}

void enableDepthWrite( bool enable )
{
	if( enable )
		g_device->SetRenderState(D3DRS_ZWRITEENABLE, TRUE);
	else
		g_device->SetRenderState(D3DRS_ZWRITEENABLE, FALSE);
}

void disableDepthWrite()
{
	g_device->SetRenderState(D3DRS_ZWRITEENABLE, FALSE);
}

void dxBlendFunction(D3DBLEND src, D3DBLEND dst)
{
    g_device->SetRenderState(D3DRS_SRCBLEND, src);
    g_device->SetRenderState(D3DRS_DESTBLEND, dst);
}

void enableAlphaBlending( bool premultiplied )
{
	g_device->SetRenderState(D3DRS_ALPHABLENDENABLE, TRUE);
	if( ! premultiplied )
    {
        dxBlendFunction(D3DBLEND_SRCALPHA, D3DBLEND_INVSRCALPHA);
    }
	else
    {
		dxBlendFunction(D3DBLEND_ONE , D3DBLEND_INVSRCALPHA );
    }
}

void disableAlphaBlending()
{
	g_device->SetRenderState(D3DRS_ALPHABLENDENABLE, FALSE);
}

void enableAdditiveBlending()
{
	g_device->SetRenderState(D3DRS_ALPHABLENDENABLE, TRUE);
    dxBlendFunction(D3DBLEND_SRCALPHA , D3DBLEND_ONE );
}

void enableAlphaTest( DWORD value, D3DCMPFUNC func )
{
	g_device->SetRenderState(D3DRS_ALPHATESTENABLE, TRUE);
    g_device->SetRenderState(D3DRS_ALPHAREF, value);
    g_device->SetRenderState(D3DRS_ALPHAFUNC, func);
}

void disableAlphaTest()
{
	g_device->SetRenderState(D3DRS_ALPHATESTENABLE, FALSE);
}

void onLostDevice()
{
    getManagedPool().OnLostDevice();
}

void onResetDevice()
{
    getManagedPool().OnResetDevice();
}
} } // namespace cinder::dx9